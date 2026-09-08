import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import numpy as np

class CDPRTrajectoryController(Node):
    def __init__(self):
        super().__init__('cdpr_trajectory_controller')

        self.declare_parameter('anchors', rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('start_position', rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('end_position', [-5.5, -5.5, 2.0])
        self.declare_parameter('duration', 10.0)
        self.declare_parameter('pretension_offset', 0.05)

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

        self.anchors = np.array(raw_anchors, dtype=float).reshape(-1, 3)
        self.num_cables = len(self.anchors)

        self.start_pos = np.array(raw_start, dtype=float)
        self.end_pos = np.array(self.get_parameter('end_position').value, dtype=float)
        self.duration = float(self.get_parameter('duration').value)
        self.pretension_offset = float(self.get_parameter('pretension_offset').value)

        self.pos_pubs = [
            self.create_publisher(Float64, f'/cdpr/l{i}_target', 10)
            for i in range(self.num_cables)
        ]
        self.vel_pubs = [
            self.create_publisher(Float64, f'/cdpr/v{i}_target', 10)
            for i in range(self.num_cables)
        ]

        self.dt = 0.02
        self.time_elapsed = 0.0
        self.is_moving = False

        self.timer = self.create_timer(self.dt, self.control_loop)
        self.start_timer = self.create_timer(2.0, self.start_trajectory)

        self.get_logger().info(
            f"Trajectory Controller initialized with {self.num_cables} anchors."
        )

    def start_trajectory(self):
        self.start_timer.cancel()
        self.is_moving = True
        self.get_logger().info("Executing autonomous trajectory...")

    def control_loop(self):
        if not self.is_moving:
            return

        self.time_elapsed += self.dt
        t = min(self.time_elapsed, self.duration)

        tau = t / self.duration
        if tau >= 1.0:
            s = 1.0
            ds_dt = 0.0
            self.is_moving = False
            self.get_logger().info("Trajectory completed successfully.")
        else:
            s = 3.0 * (tau ** 2) - 2.0 * (tau ** 3)
            ds_dt = (6.0 * tau - 6.0 * (tau ** 2)) / self.duration

        pos_cart = self.start_pos + s * (self.end_pos - self.start_pos)
        vel_cart = ds_dt * (self.end_pos - self.start_pos)

        for i in range(self.num_cables):
            vector_to_anchor = self.anchors[i] - pos_cart
            geom_length = np.linalg.norm(vector_to_anchor)
            unit_vector = vector_to_anchor / geom_length

            target_pos = max(1.0, geom_length - self.pretension_offset)
            target_vel = -float(np.dot(unit_vector, vel_cart))

            pos_msg = Float64()
            pos_msg.data = float(target_pos)
            self.pos_pubs[i].publish(pos_msg)

            vel_msg = Float64()
            vel_msg.data = target_vel
            self.vel_pubs[i].publish(vel_msg)

def main():
    rclpy.init()
    try:
        node = CDPRTrajectoryController()
        rclpy.spin(node)
    except RuntimeError:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()