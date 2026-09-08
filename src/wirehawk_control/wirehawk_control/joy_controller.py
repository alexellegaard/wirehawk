import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64
import numpy as np

class CDPRJoyController(Node):
    def __init__(self):
        super().__init__('cdpr_joy_controller')

        # Declare parameters
        self.declare_parameter('anchors', rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('start_position', rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('max_linear_speed', 1.0)
        self.declare_parameter('deadband', 0.08)

        raw_anchors = self.get_parameter('anchors').value
        raw_start = self.get_parameter('start_position').value

        if not raw_anchors or len(raw_anchors) % 3 != 0 or len(raw_anchors) < 9:
            self.get_logger().fatal("Parameter 'anchors' is missing or malformed!")
            raise RuntimeError("Missing or invalid 'anchors' parameter.")

        if not raw_start or len(raw_start) != 3:
            self.get_logger().fatal("Parameter 'start_position' is missing or malformed!")
            raise RuntimeError("Missing or invalid 'start_position' parameter.")

        self.anchors = np.array(raw_anchors, dtype=float).reshape(-1, 3)
        self.num_cables = len(self.anchors)
        self.pos_cart = np.array(raw_start, dtype=float)

        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.deadband = float(self.get_parameter('deadband').value)

        # Dynamic workspace bounds (80% anchor span)
        min_x, max_x = np.min(self.anchors[:, 0]) * 0.8, np.max(self.anchors[:, 0]) * 0.8
        min_y, max_y = np.min(self.anchors[:, 1]) * 0.8, np.max(self.anchors[:, 1]) * 0.8
        max_z = np.min(self.anchors[:, 2]) - 0.2
        self.bounds_x = (min(min_x, max_x), max(min_x, max_x))
        self.bounds_y = (min(min_y, max_y), max(min_y, max_y))
        self.bounds_z = (0.2, max_z)

        # Publishers matching /cdpr/l{i}
        self.pos_pubs = [
            self.create_publisher(Float64, f'/cdpr/l{i}', 10)
            for i in range(self.num_cables)
        ]
        self.vel_pubs = [
            self.create_publisher(Float64, f'/cdpr/v{i}', 10)
            for i in range(self.num_cables)
        ]

        self.subscription = self.create_subscription(
            Joy, '/joy', self.joy_callback, 10
        )

        self.cart_vel_cmd = np.array([0.0, 0.0, 0.0])
        self.dt = 0.02  # 50 Hz
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info(
            f"CDPR Joy Controller ready. Start target: {self.pos_cart}"
        )

    def joy_callback(self, msg: Joy):
        def apply_deadband(val, thresh):
            return val if abs(val) > thresh else 0.0

        raw_lr = msg.axes[0] if len(msg.axes) > 0 else 0.0
        raw_ud = msg.axes[1] if len(msg.axes) > 1 else 0.0
        raw_z  = msg.axes[4] if len(msg.axes) > 4 else 0.0

        val_lr = apply_deadband(raw_lr, self.deadband)
        val_ud = apply_deadband(raw_ud, self.deadband)
        val_z  = apply_deadband(raw_z, self.deadband)

        # Frame: +X Forward (Left Stick Up/Down), +Y Left, +Z Up
        self.cart_vel_cmd = np.array([
            val_ud * self.max_linear_speed,
            val_lr * self.max_linear_speed,
            val_z  * self.max_linear_speed
        ])

    def control_loop(self):
        # Integrate Cartesian velocity
        if np.linalg.norm(self.cart_vel_cmd) > 1e-4:
            self.pos_cart += self.cart_vel_cmd * self.dt
            self.pos_cart[0] = np.clip(self.pos_cart[0], self.bounds_x[0], self.bounds_x[1])
            self.pos_cart[1] = np.clip(self.pos_cart[1], self.bounds_y[0], self.bounds_y[1])
            self.pos_cart[2] = np.clip(self.pos_cart[2], self.bounds_z[0], self.bounds_z[1])

        # Standard Inverse Kinematics: L0 = Euclidean distance
        for i in range(self.num_cables):
            vector_to_anchor = self.anchors[i] - self.pos_cart
            geom_length = float(np.linalg.norm(vector_to_anchor))

            pos_msg = Float64()
            pos_msg.data = geom_length
            self.pos_pubs[i].publish(pos_msg)

            unit_vector = vector_to_anchor / geom_length
            vel_msg = Float64()
            vel_msg.data = -float(np.dot(unit_vector, self.cart_vel_cmd))
            self.vel_pubs[i].publish(vel_msg)

def main():
    rclpy.init()
    try:
        node = CDPRJoyController()
        rclpy.spin(node)
    except (RuntimeError, KeyboardInterrupt):
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()