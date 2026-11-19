import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float64MultiArray, UInt32
from wirehawk_msgs.msg import MotorCommand, MotorState

from wirehawk_control.controller import CDPRController

# MotorState schema version — must match wirehawk_ethercat_bridge bridge.hpp::
# kSchemaVersion. Bump BOTH together whenever MotorState.msg changes. A mismatch
# means the Pi's wirehawk_msgs is stale: its MotorState deserializes to garbage
# and the controller would run blind (field spool break).
SCHEMA_VERSION = 3


class CDPRNode(Node):
    """The backend-agnostic CDPR controller node.

    Task input:     /cmd_vel (Twist)            — from teleop or trajectory planner
    Backend output:  cmd/motors (MotorCommand)  — target free-span lengths [m]
    Backend input:   state/motors (MotorState)  — measured free-span lengths [m]
    Diagnostic:      /cdpr/current_pose         — FK position estimate (from lengths)

    It never reads ground-truth pose. Whether `state/motors` comes from the real
    EtherCAT bridge or the Gazebo bridge is decided by which backend is running —
    this node is agnostic. The raw-encoder <-> free-span conversion lives in the
    bridge.
    """
    def __init__(self):
        super().__init__('cdpr_node')

        # --- world params (single source of truth: cdpr_params_<world>.yaml) ---
        self.declare_parameter('anchors', rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('start_position', rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('workspace_min', rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('workspace_max', rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('max_linear_speed', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('max_linear_accel', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('max_linear_decel', -1.0)
        self.declare_parameter('max_cable_speed', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('max_cable_accel', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('rate_hz', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('cmd_timeout', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('cable_axial_stiffness', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('payload_mass', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('fk_gain', rclpy.Parameter.Type.DOUBLE)

        raw_anchors = self.get_parameter('anchors').value
        if not raw_anchors or len(raw_anchors) % 3 != 0:
            raise RuntimeError("Parameter 'anchors' must be a flattened list of (x, y, z) points.")

        rate_hz = float(self.get_parameter('rate_hz').value)
        self.dt = 1.0 / rate_hz
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)

        self.controller = CDPRController(
            anchors=np.array(raw_anchors, dtype=float).reshape(-1, 3),
            start_pos=np.array(self.get_parameter('start_position').value, dtype=float),
            ws_min=np.array(self.get_parameter('workspace_min').value, dtype=float),
            ws_max=np.array(self.get_parameter('workspace_max').value, dtype=float),
            max_speed=float(self.get_parameter('max_linear_speed').value),
            max_accel=float(self.get_parameter('max_linear_accel').value),
            max_decel=float(self.get_parameter('max_linear_decel').value),
            EA=float(self.get_parameter('cable_axial_stiffness').value),
            mass=float(self.get_parameter('payload_mass').value),
            fk_gain=float(self.get_parameter('fk_gain').value),
            max_cable_speed=float(self.get_parameter('max_cable_speed').value),
            max_cable_accel=float(self.get_parameter('max_cable_accel').value),
        )

        self.last_cmd_time = self.get_clock().now()
        self.measured_lengths = None      # None until the first state/motors arrives
        self.feedback_invalid = False     # True while readback has negative free span (bridge zero/mast_run mismatch)
        self.last_state_time = None

        # --- backend interface (relative names so they can be remapped) ---
        self.cmd_pub = self.create_publisher(MotorCommand, 'cmd/motors', 10)
        self.state_sub = self.create_subscription(
            MotorState, 'state/motors', self.state_cb, 10)

        # Schema-version check on a STABLE std_msgs type: readable even when the
        # MotorState schema is mismatched. Refuse to command motion on mismatch.
        self.schema_mismatch = False
        self.schema_sub = self.create_subscription(
            UInt32, '/wirehawk/schema_version', self.schema_cb, 10)

        # --- task input + diagnostic (legacy names, kept for teleop/planner) ---
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/cdpr/current_pose', 10)
        self.diag_pub = self.create_publisher(Float64MultiArray, '/cdpr/diagnostic', 10)

        self.timer = self.create_timer(self.dt, self.timer_callback)
        self.get_logger().info(
            f"CDPR Node (free-span interface): WS {self.controller.ws_min.tolist()} to "
            f"{self.controller.ws_max.tolist()} | EA={self.controller.EA} N, "
            f"mass={self.controller.mass} kg, gain={self.controller.fk_gain} 1/s")

    def cmd_vel_cb(self, msg: Twist):
        self.controller.set_target_velocity([
            msg.linear.x, msg.linear.y, msg.linear.z])
        self.last_cmd_time = self.get_clock().now()

    def schema_cb(self, msg):
        if msg.data != SCHEMA_VERSION and not self.schema_mismatch:
            self.get_logger().fatal(
                f"MotorState schema mismatch: bridge reports v{msg.data}, this "
                f"controller expects v{SCHEMA_VERSION}. The Pi's wirehawk_msgs "
                f"is stale — rebuild it (colcon build --packages-select "
                f"wirehawk_msgs wirehawk_ethercat_bridge). REFUSING to command motion.")
            self.schema_mismatch = True
            self.controller.stop()
        elif msg.data == SCHEMA_VERSION:
            self.schema_mismatch = False

    def state_cb(self, msg: MotorState):
        # Free-span feedback (metres) is the ONLY measurement this controller trusts.
        lengths = np.asarray(msg.position, dtype=float)
        if np.any(lengths < 0):
            # Negative free span = bridge zero/mast_run mismatch. Drop the bad
            # feedback and hold: publish nothing, so the bridge keeps the last
            # (safe) target.
            self.get_logger().warn(
                f"negative free span {lengths.tolist()} ignored — "
                f"bridge zero/mast_run mismatch; holding position")
            self.controller.stop()
            self.measured_lengths = None
            self.feedback_invalid = True
            self.last_state_time = self.get_clock().now()
            return
        self.measured_lengths = lengths
        self.feedback_invalid = False
        self.last_state_time = self.get_clock().now()

    def timer_callback(self):
        now = self.get_clock().now()

        # Schema mismatch: never command motion (the controller would be blind).
        if self.schema_mismatch:
            return

        # Negative feedback (direction/calibration mismatch): do NOT command
        # motion. The bridge holds the last target — the motors stay put.
        if self.feedback_invalid:
            return

        # Watchdog: zero velocity if the task input stops (stale /cmd_vel).
        if (now - self.last_cmd_time).nanoseconds / 1e9 > self.cmd_timeout:
            self.controller.stop()

        lengths = self.controller.step(self.dt, self.measured_lengths)

        cmd = MotorCommand()
        cmd.header.stamp = now.to_msg()
        cmd.ctrl_word = 0                       # reserved; lifecycle not wired yet
        cmd.position = [float(l) for l in lengths]
        self.cmd_pub.publish(cmd)

        pose = PoseStamped()
        pose.header.stamp = now.to_msg()
        pose.header.frame_id = 'world'
        pose.pose.position.x = float(self.controller.P_est[0])
        pose.pose.position.y = float(self.controller.P_est[1])
        pose.pose.position.z = float(self.controller.P_est[2])
        self.pose_pub.publish(pose)

        # Diagnostic: FK estimate, target, integral length correction, the
        # position error, FK convergence flag, cable tensions (slack indicator)
        # and the measured cable lengths (raw feedback cross-check).
        diag = Float64MultiArray()
        diag.data = (
            [float(x) for x in self.controller.P_est]
            + [float(x) for x in self.controller.target_pos]
            + [float(x) for x in self.controller.L_integral]
            + [float(x) for x in (self.controller.target_pos - self.controller.P_est)]
            + [1.0 if self.controller.fk_converged else 0.0]
            + [float(x) for x in self.controller.tensions]
            + [float(x) for x in self.controller.L_m]
        )
        self.diag_pub.publish(diag)


def main(args=None):
    rclpy.init(args=args)
    node = CDPRNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
