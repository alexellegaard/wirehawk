"""Bird sweeper — the motion layer for bird deterrence.

Turns a BirdTarget (from the decision layer) into a /cmd_vel stream:
  * valid target  -> sweep THROUGH it at `speed` (5 m/s), horizontal only at a
                     fixed z, then hold once passed — the "scare" is a fast
                     approach that never decelerates toward the bird.
  * no target     -> gentle return to home.

Backend-agnostic: it talks only to cdpr_node via topics (BirdTarget in,
/cdpr/current_pose in, /cmd_vel out) and never knows whether the motors behind
the controller are the sim bridge or the EtherCAT bridge.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from wirehawk_msgs.msg import BirdTarget


class BirdSweeper(Node):
    def __init__(self):
        super().__init__('bird_sweeper')

        self.declare_parameter('home_position', [0.0, 0.0, 1.5])
        self.declare_parameter('return_speed', 1.5)
        self.declare_parameter('z_height', 1.5)
        self.declare_parameter('home_tolerance', 0.1)
        self.declare_parameter('target_timeout', 2.0)
        self.declare_parameter('rate_hz', 30.0)

        self.home = np.array(self.get_parameter('home_position').value, dtype=float)
        self.return_speed = float(self.get_parameter('return_speed').value)
        self.z_height = float(self.get_parameter('z_height').value)
        self.home_tol = float(self.get_parameter('home_tolerance').value)
        self.timeout = float(self.get_parameter('target_timeout').value)
        self.dt = 1.0 / float(self.get_parameter('rate_hz').value)

        # Target state (horizontal plane only; z is fixed by z_height).
        self.target = None
        self.target_speed = 0.0
        self.target_valid = False
        self.last_target_time = self.get_clock().now()

        # Current TCP pose from the controller's FK estimate.
        self.current_pos = None

        # Commanded velocity from the previous tick, used for pass detection.
        self.cmd_vel = np.zeros(2, dtype=float)

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(BirdTarget, '/bird_target', self.target_cb, 10)
        self.create_subscription(PoseStamped, '/cdpr/current_pose', self.pose_cb, 10)
        self.timer = self.create_timer(self.dt, self.tick)

        self.get_logger().info(
            f'bird_sweeper up: home {self.home[:2].tolist()}, '
            f'z={self.z_height} m, return_speed={self.return_speed} m/s')

    def target_cb(self, msg: BirdTarget):
        self.target = np.array([msg.position.x, msg.position.y], dtype=float)
        self.target_speed = float(msg.speed)
        self.target_valid = bool(msg.valid)
        self.last_target_time = self.get_clock().now()

    def pose_cb(self, msg: PoseStamped):
        self.current_pos = np.array([
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
        ], dtype=float)

    def _target_fresh(self) -> bool:
        age = (self.get_clock().now() - self.last_target_time).nanoseconds / 1e9
        return age < self.timeout

    def tick(self):
        if self.current_pos is None:
            return  # no FK pose yet

        tcp_xy = self.current_pos[:2]

        if self.target_valid and self._target_fresh() and self.target is not None:
            vel = self._sweep(tcp_xy)
        else:
            vel = self._return_home(tcp_xy)

        self.cmd_vel = vel
        msg = Twist()
        msg.linear.x = float(vel[0])
        msg.linear.y = float(vel[1])
        msg.linear.z = 0.0
        self.pub.publish(msg)

    def _sweep(self, tcp_xy):
        vec = self.target - tcp_xy
        dist = float(np.linalg.norm(vec))
        if dist < 1e-3:
            return np.zeros(2)
        # Pass detection: if we're already moving away from the target we've
        # swept through it — hold instead of chasing an empty spot.
        if float(np.dot(vec, self.cmd_vel)) < 0.0:
            return np.zeros(2)
        return self.target_speed * (vec / dist)

    def _return_home(self, tcp_xy):
        vec = self.home[:2] - tcp_xy
        dist = float(np.linalg.norm(vec))
        if dist < self.home_tol:
            return np.zeros(2)
        return self.return_speed * (vec / dist)


def main(args=None):
    rclpy.init(args=args)
    node = BirdSweeper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())  # stop on exit
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
