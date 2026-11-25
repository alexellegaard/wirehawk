import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from wirehawk_msgs.msg import MotorCommand, MotorState


class SimBridgeNode(Node):
    """Stand-in for the real EtherCAT bridge + drives + winches + encoders.

    Consumes the SAME interface the real bridge consumes (`cmd/motors`, free-span
    metres) and produces the SAME interface it produces (`state/motors`, free-span
    metres). It relays the commanded free spans to Gazebo (via /cdpr/l{i}) and
    echoes them back as perfect-tracking feedback.

    The real bridge converts raw encoder <-> free span (spool model + zero +
    mast_run); the sim has no encoder, so it just passes lengths through. Cable
    stretch is applied by the Gazebo catenary plugin, not here.
    """
    def __init__(self):
        super().__init__('sim_bridge')

        # Four cables, matching the four-pillar Gazebo world (anchors A0..A3).
        self.num_cables = 4

        # Measurement noise (sigma) on the FEEDBACK only — models encoder jitter.
        # Zero-mean, so no systematic drift. Default 0 = exact echo.
        # (A constant sag BIAS on the feedback was tried and rejected: the FK
        # keeps "chasing" the offset and the payload runs away. Sag is a physics
        # effect, already modelled by the catenary plugin's linear_density.)
        self.declare_parameter('noise_sigma', 0.0)
        self.noise_sigma = float(self.get_parameter('noise_sigma').value)
        self.rng = np.random.default_rng()

        self.cmd_sub = self.create_subscription(
            MotorCommand, 'cmd/motors', self.cmd_cb, 10)
        self.state_pub = self.create_publisher(MotorState, 'state/motors', 10)
        # Cable lengths into Gazebo (bridged to gz.msgs.Double by ros_gz_bridge).
        self.length_pubs = [
            self.create_publisher(Float64, f'/cdpr/l{i}', 10)
            for i in range(self.num_cables)
        ]
        self.get_logger().info(
            f"Sim bridge up: free span -> /cdpr/l{{0..{self.num_cables - 1}}}, "
            f"echo -> state/motors")

    def cmd_cb(self, msg: MotorCommand):
        lengths = np.asarray(msg.position, dtype=float)

        # Free span -> Gazebo physics.
        for i in range(min(len(lengths), self.num_cables)):
            m = Float64()
            m.data = float(lengths[i])
            self.length_pubs[i].publish(m)

        # Simulated feedback: command + zero-mean measurement noise. The noise
        # lives in the measurement path only, not the /cdpr/l command.
        fb = lengths
        if self.noise_sigma > 0.0:
            fb = fb + self.rng.normal(0.0, self.noise_sigma, size=lengths.shape)
        state = MotorState()
        state.position = [float(v) for v in fb]
        state.torque = [0] * len(lengths)          # torque synthesis: future
        state.status_word = [0] * len(lengths)     # drive state: future
        state.error_code = [0] * len(lengths)
        self.state_pub.publish(state)


def main(args=None):
    rclpy.init(args=args)
    node = SimBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
