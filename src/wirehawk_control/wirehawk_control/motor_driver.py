import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import numpy as np

class LocalMotorController(Node):
    def __init__(self):
        super().__init__('motor_driver')

        # Declare mandatory parameters without fallbacks
        self.declare_parameter('anchors', rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('start_position', rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('kp', 20.0)

        raw_anchors = self.get_parameter('anchors').value
        raw_start = self.get_parameter('start_position').value

        if not raw_anchors or len(raw_anchors) % 3 != 0 or len(raw_anchors) < 9:
            self.get_logger().fatal(
                "Parameter 'anchors' is missing or malformed! Must be a flattened array [x,y,z,...] with >= 3 anchors."
            )
            raise RuntimeError("Missing or invalid 'anchors' parameter.")

        if not raw_start or len(raw_start) != 3:
            self.get_logger().fatal(
                "Parameter 'start_position' is missing or malformed! Expected [x, y, z]."
            )
            raise RuntimeError("Missing or invalid 'start_position' parameter.")

        anchors = np.array(raw_anchors, dtype=float).reshape(-1, 3)
        self.num_cables = len(anchors)
        start_pos = np.array(raw_start, dtype=float)
        self.kp = float(self.get_parameter('kp').value)

        # Derive initial cable spool lengths mathematically from startup geometry
        initial_lengths = [
            float(np.linalg.norm(anchors[i] - start_pos))
            for i in range(self.num_cables)
        ]

        self.target_positions = list(initial_lengths)
        self.target_velocities = [0.0] * self.num_cables
        self.current_lengths = list(initial_lengths)

        self.gazebo_pubs = [
            self.create_publisher(Float64, f'/cdpr/l{i}', 10)
            for i in range(self.num_cables)
        ]

        for i in range(self.num_cables):
            self.create_subscription(
                Float64, f'/cdpr/l{i}_target',
                lambda msg, idx=i: self.pos_callback(msg, idx), 10
            )
            self.create_subscription(
                Float64, f'/cdpr/v{i}_target',
                lambda msg, idx=i: self.vel_callback(msg, idx), 10
            )

        self.dt = 0.01  # 100 Hz
        self.timer = self.create_timer(self.dt, self.motor_control_loop)

        self.get_logger().info(
            f"Motor Driver initialized for {self.num_cables} winches. Calibrated initial length: {initial_lengths[0]:.2f}m"
        )

    def pos_callback(self, msg, idx):
        self.target_positions[idx] = msg.data

    def vel_callback(self, msg, idx):
        self.target_velocities[idx] = msg.data

    def motor_control_loop(self):
        for i in range(self.num_cables):
            pos_error = self.target_positions[i] - self.current_lengths[i]
            commanded_velocity = self.target_velocities[i] + (self.kp * pos_error)

            self.current_lengths[i] += commanded_velocity * self.dt

            msg = Float64()
            msg.data = self.current_lengths[i]
            self.gazebo_pubs[i].publish(msg)

def main():
    rclpy.init()
    try:
        node = LocalMotorController()
        rclpy.spin(node)
    except RuntimeError:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()