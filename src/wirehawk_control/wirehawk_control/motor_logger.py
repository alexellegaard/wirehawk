#!/usr/bin/env python3
"""Always-on logger capturing three streams to timestamped CSVs:

  cmd   <- cmd/motors        : the controller's commanded counts (one version)
  sim   <- sim/state/motors  : the sim bridge's feedback (perfect-tracking echo)
  real  <- real/state/motors : the real EtherCAT bridge's ACTUAL drive feedback

Each row: t, pos0, pos1, ...  (seconds + encoder counts per motor).
Output: <output_dir>/cmd_<stamp>.csv, sim_<stamp>.csv, real_<stamp>.csv.

Topic names are parameters (cmd_topic, sim_state_topic, real_state_topic) so the
exact namespacing can be overridden at launch without editing code.

Run standalone:   ros2 run wirehawk_control motor_logger
"""
import os
import csv
import time

import rclpy
from rclpy.node import Node
from wirehawk_msgs.msg import MotorCommand, MotorState


class MotorLogger(Node):
    def __init__(self):
        super().__init__('motor_logger')
        self.declare_parameter('output_dir', os.path.expanduser('~/wirehawk_logs'))
        self.declare_parameter('flush_period_s', 1.0)
        self.declare_parameter('cmd_topic', '/cmd/motors')
        self.declare_parameter('sim_state_topic', '/sim/state/motors')
        self.declare_parameter('real_state_topic', '/real/state/motors')

        out = self.get_parameter('output_dir').value
        os.makedirs(out, exist_ok=True)
        stamp = time.strftime('%Y%m%d_%H%M%S')
        self.files = {
            'cmd':  open(os.path.join(out, f'cmd_{stamp}.csv'),  'w', newline=''),
            'sim':  open(os.path.join(out, f'sim_{stamp}.csv'),  'w', newline=''),
            'real': open(os.path.join(out, f'real_{stamp}.csv'), 'w', newline=''),
        }
        self.writers = {k: csv.writer(v) for k, v in self.files.items()}
        self.bufs = {'cmd': [], 'sim': [], 'real': []}
        self.t0 = time.monotonic()

        self.create_subscription(
            MotorCommand, self.get_parameter('cmd_topic').value, self.on_cmd, 100)
        self.create_subscription(
            MotorState, self.get_parameter('sim_state_topic').value,
            lambda m: self.on_state('sim', m), 100)
        self.create_subscription(
            MotorState, self.get_parameter('real_state_topic').value,
            lambda m: self.on_state('real', m), 100)
        self.create_timer(self.get_parameter('flush_period_s').value, self.flush)
        self.get_logger().info(
            f'logging cmd[{self.get_parameter("cmd_topic").value}] '
            f'sim[{self.get_parameter("sim_state_topic").value}] '
            f'real[{self.get_parameter("real_state_topic").value}] -> {out}')

    def on_cmd(self, m):
        self.bufs['cmd'].append([time.monotonic() - self.t0] + list(m.position))

    def on_state(self, kind, m):
        self.bufs[kind].append([time.monotonic() - self.t0] + list(m.position))

    def flush(self):
        for k in self.bufs:
            self.writers[k].writerows(self.bufs[k])
            self.bufs[k].clear()
            self.files[k].flush()

    def finalize(self):
        self.flush()
        for f in self.files.values():
            f.close()


def main(args=None):
    rclpy.init(args=args)
    node = MotorLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.finalize()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
