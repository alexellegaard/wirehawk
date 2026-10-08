import os
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float64MultiArray
from wirehawk_msgs.msg import MotorCommand, MotorState
from ament_index_python.packages import get_package_share_directory

from wirehawk_control.controller import CDPRController
from wirehawk_spool.spool_model import load_spec


class CDPRNode(Node):
    """The backend-agnostic CDPR controller node.

    Task input:     /cmd_vel (Twist)            — from teleop or trajectory planner
    Backend output:  cmd/motors (MotorCommand)  — target encoder counts
    Backend input:   state/motors (MotorState)  — measured encoder counts
    Diagnostic:      /cdpr/current_pose         — FK position estimate (from counts)

    It never puts cable lengths on the wire and never reads ground-truth pose.
    Whether `state/motors` comes from the real EtherCAT bridge or the Gazebo
    bridge is decided by which backend is running — this node is agnostic.
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

        # --- spool geometry (single source of truth: wirehawk_spool config) ---
        spool_yaml = os.path.join(
            get_package_share_directory('wirehawk_spool'), 'config', 'spool.yaml')
        spec = load_spec(spool_yaml)

        self.controller = CDPRController(
            anchors=np.array(raw_anchors, dtype=float).reshape(-1, 3),
            start_pos=np.array(self.get_parameter('start_position').value, dtype=float),
            ws_min=np.array(self.get_parameter('workspace_min').value, dtype=float),
            ws_max=np.array(self.get_parameter('workspace_max').value, dtype=float),
            max_speed=float(self.get_parameter('max_linear_speed').value),
            max_accel=float(self.get_parameter('max_linear_accel').value),
            EA=float(self.get_parameter('cable_axial_stiffness').value),
            mass=float(self.get_parameter('payload_mass').value),
            fk_gain=float(self.get_parameter('fk_gain').value),
            max_cable_speed=float(self.get_parameter('max_cable_speed').value),
            max_cable_accel=float(self.get_parameter('max_cable_accel').value),
            spec=spec,
        )

        self.last_cmd_time = self.get_clock().now()
        self.measured_counts = None       # None until the first state/motors arrives
        self.last_state_time = None

        # --- backend interface (relative names so they can be remapped) ---
        self.cmd_pub = self.create_publisher(MotorCommand, 'cmd/motors', 10)
        self.state_sub = self.create_subscription(
            MotorState, 'state/motors', self.state_cb, 10)

        # --- task input + diagnostic (legacy names, kept for teleop/planner) ---
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/cdpr/current_pose', 10)
        self.diag_pub = self.create_publisher(Float64MultiArray, '/cdpr/diagnostic', 10)

        self.timer = self.create_timer(self.dt, self.timer_callback)
        self.get_logger().info(
            f"CDPR Node (counts interface): WS {self.controller.ws_min.tolist()} to "
            f"{self.controller.ws_max.tolist()} | EA={self.controller.EA} N, "
            f"mass={self.controller.mass} kg, gain={self.controller.fk_gain} 1/s")

    def cmd_vel_cb(self, msg: Twist):
        self.controller.set_target_velocity([
            msg.linear.x, msg.linear.y, msg.linear.z])
        self.last_cmd_time = self.get_clock().now()

    def state_cb(self, msg: MotorState):
        # Encoder feedback is the ONLY measurement this controller trusts.
        self.measured_counts = np.asarray(msg.position, dtype=np.int64)
        self.last_state_time = self.get_clock().now()

    def timer_callback(self):
        now = self.get_clock().now()

        # Watchdog: zero velocity if the task input stops (stale /cmd_vel).
        if (now - self.last_cmd_time).nanoseconds / 1e9 > self.cmd_timeout:
            self.controller.stop()

        counts = self.controller.step(self.dt, self.measured_counts)

        cmd = MotorCommand()
        cmd.header.stamp = now.to_msg()
        cmd.ctrl_word = 0                       # reserved; lifecycle not wired yet
        cmd.position = [int(c) for c in counts]
        self.cmd_pub.publish(cmd)

        pose = PoseStamped()
        pose.header.stamp = now.to_msg()
        pose.header.frame_id = 'world'
        pose.pose.position.x = float(self.controller.P_est[0])
        pose.pose.position.y = float(self.controller.P_est[1])
        pose.pose.position.z = float(self.controller.P_est[2])
        self.pose_pub.publish(pose)

        # Diagnostic: FK estimate, target, integral length correction, and the
        # position error they produce — the smoking gun for the FK/integral loop
        # going unstable during fast reversals.
        diag = Float64MultiArray()
        diag.data = (
            [float(x) for x in self.controller.P_est]
            + [float(x) for x in self.controller.target_pos]
            + [float(x) for x in self.controller.L_integral]
            + [float(x) for x in (self.controller.target_pos - self.controller.P_est)]
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
