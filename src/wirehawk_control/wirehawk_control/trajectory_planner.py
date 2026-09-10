import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped, PoseStamped
import numpy as np


class TrajectoryPlanner(Node):
    def __init__(self):
        super().__init__('trajectory_planner')

        # Parameters
        self.declare_parameter('max_linear_speed', 0.8)       # m/s
        self.declare_parameter('goal_tolerance', 0.02)        # 2 cm deadband
        self.declare_parameter('decel_distance', 0.3)         # Deceleration zone (m)
        self.declare_parameter('rate_hz', 50.0)

        self.max_speed = float(self.get_parameter('max_linear_speed').value)
        self.tolerance = float(self.get_parameter('goal_tolerance').value)
        self.decel_dist = float(self.get_parameter('decel_distance').value)
        self.rate_hz = float(self.get_parameter('rate_hz').value)

        # State tracking
        self.current_pos = None
        self.target_pos = None

        # Publisher to CDPR node
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribe to ground truth state from cdpr_node
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/cdpr/current_pose',
            self.current_pose_callback,
            10
        )

        # Setpoint goal subscribers
        self.point_sub = self.create_subscription(
            PointStamped,
            '/goal_point',
            self.point_goal_callback,
            10
        )
        self.goal_pose_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.pose_goal_callback,
            10
        )

        # Planning loop
        self.timer = self.create_timer(1.0 / self.rate_hz, self.update_trajectory)
        self.get_logger().info("Trajectory Planner initialized. Waiting for /cdpr/current_pose...")

    def current_pose_callback(self, msg: PoseStamped):
        self.current_pos = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ], dtype=float)

        # Initialize target to current pose upon first message received
        if self.target_pos is None:
            self.target_pos = np.copy(self.current_pos)
            self.get_logger().info(f"Synchronized with CDPR pose at: {self.current_pos.round(3).tolist()}")

    def point_goal_callback(self, msg: PointStamped):
        new_target = np.array([msg.point.x, msg.point.y, msg.point.z], dtype=float)
        self.set_goal(new_target)

    def pose_goal_callback(self, msg: PoseStamped):
        new_target = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z], dtype=float)
        self.set_goal(new_target)

    def set_goal(self, target: np.ndarray):
        if self.current_pos is None:
            self.get_logger().warn("Cannot set goal: Waiting for initial /cdpr/current_pose.")
            return

        self.target_pos = target
        dist = np.linalg.norm(self.target_pos - self.current_pos)
        self.get_logger().info(f"New target: {self.target_pos.round(3).tolist()} | Distance: {dist:.2f} m")

    def update_trajectory(self):
        # Wait until we have a valid state from cdpr_node
        if self.current_pos is None or self.target_pos is None:
            return

        error_vec = self.target_pos - self.current_pos
        dist = np.linalg.norm(error_vec)

        # Target reached: command a stop. cdpr_node's slew-rate limiter ramps us down.
        if dist <= self.tolerance:
            self.vel_pub.publish(Twist())
            return

        direction = error_vec / dist

        # Proportional deceleration profile within braking zone
        if dist < self.decel_dist:
            desired_speed = self.max_speed * (dist / self.decel_dist)
            desired_speed = max(desired_speed, 0.05)  # Prevent stall before reaching tolerance
        else:
            desired_speed = self.max_speed

        # Stream command. NOTE: no rate-limiting here. cdpr_node owns the
        # accel/slew limit, so exactly ONE limiter exists in the chain.
        cmd = Twist()
        cmd.linear.x = float(direction[0] * desired_speed)
        cmd.linear.y = float(direction[1] * desired_speed)
        cmd.linear.z = float(direction[2] * desired_speed)
        self.vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.vel_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
