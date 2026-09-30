import os
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from wirehawk_msgs.msg import MotorCommand, MotorState
from ament_index_python.packages import get_package_share_directory

from wirehawk_spool.spool_model import load_spec, counts_to_lengths


class SimBridgeNode(Node):
    """Stand-in for the real EtherCAT bridge + drives + winches + encoders.

    Consumes the SAME interface the real bridge consumes (`cmd/motors`, encoder
    counts) and produces the SAME interface it produces (`state/motors`). It:
      * converts counts -> cable length via the shared spool model,
      * publishes that length to Gazebo (via /cdpr/l{i} + ros_gz_bridge),
      * echoes the counts back as simulated feedback on `state/motors`.

    Fidelity: the virtual winch tracks perfectly (actual counts == commanded
    counts); cable stretch is applied by the Gazebo catenary plugin, not here.
    The real bridge's trapezoidal safety clamp, drive-status synthesis and
    torque feedback are not yet reproduced (documented future work).
    """
    def __init__(self):
        super().__init__('sim_bridge')

        spool_yaml = os.path.join(
            get_package_share_directory('wirehawk_spool'), 'config', 'spool.yaml')
        self.spec = load_spec(spool_yaml)

        # Four cables, matching the four-pillar Gazebo world (anchors A0..A3).
        self.num_cables = 4

        self.cmd_sub = self.create_subscription(
            MotorCommand, 'cmd/motors', self.cmd_cb, 10)
        self.state_pub = self.create_publisher(MotorState, 'state/motors', 10)
        # Cable lengths into Gazebo (bridged to gz.msgs.Double by ros_gz_bridge).
        self.length_pubs = [
            self.create_publisher(Float64, f'/cdpr/l{i}', 10)
            for i in range(self.num_cables)
        ]
        self.get_logger().info(
            f"Sim bridge up: counts -> /cdpr/l{{0..{self.num_cables - 1}}}, "
            f"echo -> state/motors (spool total {self.spec.total_m} m)")

    def cmd_cb(self, msg: MotorCommand):
        counts = np.asarray(msg.position, dtype=np.int64)
        lengths = counts_to_lengths(counts, self.spec)

        # Counts -> length -> Gazebo physics.
        for i in range(min(len(lengths), self.num_cables)):
            m = Float64()
            m.data = float(lengths[i])
            self.length_pubs[i].publish(m)

        # Simulated feedback: perfect-tracking virtual encoder.
        state = MotorState()
        state.position = [int(c) for c in counts]
        state.torque = [0] * len(counts)          # torque synthesis: future
        state.status_word = [0] * len(counts)     # drive state: future
        state.error_code = [0] * len(counts)
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
