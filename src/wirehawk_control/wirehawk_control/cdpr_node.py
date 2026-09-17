import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float64
import numpy as np

from wirehawk_control.kinematics import CDPRKinematics


class CDPRNode(Node):
    def __init__(self):
        super().__init__('cdpr_node')

        # Parameters (single source of truth in yaml, typed, no default)
        self.declare_parameter('anchors', rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('start_position', rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('workspace_min', rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('workspace_max', rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('max_linear_speed', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('max_linear_accel', rclpy.Parameter.Type.DOUBLE)  # Slew-rate limit (m/s^2)
        self.declare_parameter('rate_hz', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('cmd_timeout', rclpy.Parameter.Type.DOUBLE)
        # Elastic-FK feedback (calibration copies matching the world SDF cable/payload params)
        self.declare_parameter('cable_axial_stiffness', rclpy.Parameter.Type.DOUBLE)  # EA [N]
        self.declare_parameter('payload_mass', rclpy.Parameter.Type.DOUBLE)           # [kg]
        self.declare_parameter('fk_gain', rclpy.Parameter.Type.DOUBLE)                # [1/s]

        raw_anchors = self.get_parameter('anchors').value
        if not raw_anchors or len(raw_anchors) % 3 != 0:
            raise RuntimeError("Parameter 'anchors' must be a flattened list of (x, y, z) points.")

        anchors = np.array(raw_anchors, dtype=float).reshape(-1, 3)
        start_pos = np.array(self.get_parameter('start_position').value, dtype=float)
        self.ws_min = np.array(self.get_parameter('workspace_min').value, dtype=float)
        self.ws_max = np.array(self.get_parameter('workspace_max').value, dtype=float)
        self.max_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_accel = float(self.get_parameter('max_linear_accel').value)
        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self.EA = float(self.get_parameter('cable_axial_stiffness').value)
        self.mass = float(self.get_parameter('payload_mass').value)
        self.fk_gain = float(self.get_parameter('fk_gain').value)
        self.dt = 1.0 / self.rate_hz

        # Kinematics core
        self.kinematics = CDPRKinematics(anchors=anchors)

        # State
        self.target_pos = np.clip(start_pos, self.ws_min, self.ws_max)  # integrated velocity target
        self.target_vel = np.zeros(3, dtype=float)
        self.filtered_vel = np.zeros(3, dtype=float)
        self.last_cmd_time = self.get_clock().now()

        # Cable-length command state (feedforward + integral sag compensation)
        self.L0 = self.kinematics.compute_commanded_lengths(self.target_pos).copy()
        self.L0_integral = np.zeros(self.kinematics.num_cables, dtype=float)
        self.P_est = self.target_pos.copy()  # FK estimate of the actual position

        # Direct publishers to Gazebo
        self.cable_pubs = [
            self.create_publisher(Float64, f'/cdpr/l{i}', 10)
            for i in range(self.kinematics.num_cables)
        ]

        # Pose publisher (now the FK estimate, not dead reckoning)
        self.pose_pub = self.create_publisher(PoseStamped, '/cdpr/current_pose', 10)

        # Velocity subscriber
        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.timer = self.create_timer(self.dt, self.timer_callback)
        self.get_logger().info(
            f"CDPR Node (elastic-FK feedback): WS {self.ws_min.tolist()} to {self.ws_max.tolist()}"
            f" | EA={self.EA} N, mass={self.mass} kg, gain={self.fk_gain} 1/s"
        )

    def cmd_vel_callback(self, msg: Twist):
        vx = np.clip(msg.linear.x, -self.max_speed, self.max_speed)
        vy = np.clip(msg.linear.y, -self.max_speed, self.max_speed)
        vz = np.clip(msg.linear.z, -self.max_speed, self.max_speed)
        self.target_vel = np.array([vx, vy, vz], dtype=float)
        self.last_cmd_time = self.get_clock().now()

    def timer_callback(self):
        # Watchdog: zero velocity if input stops
        time_since_cmd = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if time_since_cmd > self.cmd_timeout:
            self.target_vel = np.zeros(3, dtype=float)

        # 1. Acceleration rate limiter (slew rate filter)
        vel_diff = self.target_vel - self.filtered_vel
        diff_mag = np.linalg.norm(vel_diff)
        max_dv = self.max_accel * self.dt

        if diff_mag > max_dv:
            self.filtered_vel += (vel_diff / diff_mag) * max_dv
        else:
            self.filtered_vel = np.copy(self.target_vel)

        # 2. Integrate velocity -> target position, clamp within workspace bounds
        self.target_pos += self.filtered_vel * self.dt
        self.target_pos = np.clip(self.target_pos, self.ws_min, self.ws_max)

        # 3. Estimate the actual payload position from the commanded cable lengths
        #    (elastic FK — self-contained, no tension measurement).
        self.P_est, converged = self.kinematics.elastic_forward_kinematics(
            self.L0, self.EA, self.mass, self.P_est
        )
        self.P_est = np.clip(self.P_est, self.ws_min, self.ws_max)

        # 4. Position error: how far the payload really is from the target.
        e = self.target_pos - self.P_est

        # 5. Integral feedback on length: keep winding in/out until the payload is
        #    actually at the target (pretension emerges automatically from the loop).
        #    dL_i/dt = -gain * (u_i . e),  u_i = unit vector from payload to anchor i.
        u = self.kinematics.anchor_unit_vectors(self.P_est)
        proj = u @ e  # (N,) projection of the position error onto each cable direction
        self.L0_integral -= self.fk_gain * proj * self.dt
        self.L0_integral = np.clip(self.L0_integral, -5.0, 5.0)  # anti-windup

        # 6. Commanded lengths = geometric feedforward + accumulated sag compensation.
        L0_ff = self.kinematics.compute_commanded_lengths(self.target_pos)
        self.L0 = np.clip(L0_ff + self.L0_integral, 0.1, 150.0)

        # 7. Publish pose estimate (the FK estimate, not the dead-reckoned target).
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'world'
        pose_msg.pose.position.x = float(self.P_est[0])
        pose_msg.pose.position.y = float(self.P_est[1])
        pose_msg.pose.position.z = float(self.P_est[2])
        self.pose_pub.publish(pose_msg)

        # 8. Publish commanded lengths to the winches.
        for i, pub in enumerate(self.cable_pubs):
            msg = Float64()
            msg.data = float(self.L0[i])
            pub.publish(msg)


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
