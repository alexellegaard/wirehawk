import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import numpy as np

from wirehawk_control.kinematics import CDPRKinematics

class CDPRNode(Node):
    def __init__(self):
        super().__init__('cdpr_node')

        # Parameters
        self.declare_parameter('anchors', rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('start_position', [0.0, 0.0, 1.5])
        self.declare_parameter('workspace_min', [-7.0, -7.0, 0.3])
        self.declare_parameter('workspace_max', [ 7.0,  7.0, 2.7])
        self.declare_parameter('pretension_offset', 0.05)
        self.declare_parameter('max_linear_speed', 1.0)
        self.declare_parameter('rate_hz', 50.0)
        self.declare_parameter('cmd_timeout', 0.2)

        raw_anchors = self.get_parameter('anchors').value
        if not raw_anchors or len(raw_anchors) % 3 != 0:
            raise RuntimeError("Parameter 'anchors' must be a flattened list of (x, y, z) points.")

        anchors = np.array(raw_anchors, dtype=float).reshape(-1, 3)
        start_pos = np.array(self.get_parameter('start_position').value, dtype=float)
        self.ws_min = np.array(self.get_parameter('workspace_min').value, dtype=float)
        self.ws_max = np.array(self.get_parameter('workspace_max').value, dtype=float)
        slack_offset = float(self.get_parameter('pretension_offset').value)
        self.max_speed = float(self.get_parameter('max_linear_speed').value)
        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self.dt = 1.0 / self.rate_hz

        # Kinematics core
        self.kinematics = CDPRKinematics(anchors=anchors, slack_offset=slack_offset)

        # State
        self.current_pos = np.clip(start_pos, self.ws_min, self.ws_max)
        self.target_vel = np.zeros(3, dtype=float)
        self.last_cmd_time = self.get_clock().now()

        # Direct publishers to Gazebo
        self.cable_pubs = [
            self.create_publisher(Float64, f'/cdpr/l{i}', 10)
            for i in range(self.kinematics.num_cables)
        ]

        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.timer = self.create_timer(self.dt, self.timer_callback)
        self.get_logger().info(f"CDPR Node running: Workspace limits {self.ws_min.tolist()} to {self.ws_max.tolist()}")

    def cmd_vel_callback(self, msg: Twist):
        vx = np.clip(msg.linear.x, -self.max_speed, self.max_speed)
        vy = np.clip(msg.linear.y, -self.max_speed, self.max_speed)
        vz = np.clip(msg.linear.z, -self.max_speed, self.max_speed)
        self.target_vel = np.array([vx, vy, vz], dtype=float)
        self.last_cmd_time = self.get_clock().now()

    def timer_callback(self):
        # Watchdog: Stop moving if commands cease
        time_since_cmd = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if time_since_cmd > self.cmd_timeout:
            self.target_vel = np.zeros(3, dtype=float)

        # 1. Integrate and hard-clamp within workspace bounds
        self.current_pos += self.target_vel * self.dt
        self.current_pos = np.clip(self.current_pos, self.ws_min, self.ws_max)

        # 2. Compute commanded lengths with smooth slack allocation
        cmd_lengths = self.kinematics.compute_commanded_lengths(self.current_pos)

        # 3. Publish to winches
        for i, pub in enumerate(self.cable_pubs):
            msg = Float64()
            msg.data = float(cmd_lengths[i])
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