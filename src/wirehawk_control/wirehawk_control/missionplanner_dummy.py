"""Dummy decision layer for validating the bird sweeper before real vision exists.

Publishes a fixed BirdTarget (a fake bird at a configurable position) so the
sweeper can be exercised in the sim. Stands in for the future mission planner,
which will subscribe to the bird detector and publish the real target.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from wirehawk_msgs.msg import BirdTarget


class MissionPlannerDummy(Node):
    def __init__(self):
        super().__init__('missionplanner_dummy')

        self.declare_parameter('target_position', [8.0, 0.0, 1.5])
        self.declare_parameter('speed', 5.0)
        self.declare_parameter('valid', True)
        self.declare_parameter('rate_hz', 10.0)

        pos = self.get_parameter('target_position').value
        self.msg = BirdTarget()
        self.msg.position = Point(x=float(pos[0]), y=float(pos[1]), z=float(pos[2]))
        self.msg.speed = float(self.get_parameter('speed').value)
        self.msg.valid = bool(self.get_parameter('valid').value)

        self.pub = self.create_publisher(BirdTarget, '/bird_target', 10)
        dt = 1.0 / float(self.get_parameter('rate_hz').value)
        self.timer = self.create_timer(dt, self.tick)
        self.get_logger().info(
            f'missionplanner_dummy publishing fixed target '
            f'({self.msg.position.x}, {self.msg.position.y}, {self.msg.position.z}) '
            f'@ {self.msg.speed} m/s (valid={self.msg.valid})')

    def tick(self):
        self.pub.publish(self.msg)


def main(args=None):
    rclpy.init(args=args)
    node = MissionPlannerDummy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
