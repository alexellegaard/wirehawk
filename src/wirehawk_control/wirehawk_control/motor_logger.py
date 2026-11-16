#!/usr/bin/env python3
"""Always-on logger capturing streams to timestamped CSVs:

  cmd    <- cmd/motors        : the controller's commanded counts (logical)
  sim    <- sim/state/motors  : the sim bridge's feedback (perfect-tracking echo)
  real   <- real/state/motors : the real EtherCAT bridge's ACTUAL drive feedback (logical)
  target <- real/state/motors : the real bridge's commanded 0x607A target (PHYSICAL counts)
  diag   <- /cdpr/diagnostic  : FK estimate, target, tensions, etc.

Each row: t, pos0, pos1, ...  (seconds + encoder counts per motor).
Output: <output_dir>/<kind>_<stamp>.csv.

The `target` stream is the missing signal for the field oscillation bug: it shows
what the bridge actually told the drive, so we can tell a bridge/interpolator
fault (target oscillates) from a drive-side DCSYNC fault (target flat, drive moves).
"""
import os
import csv
import time

import rclpy
from rclpy.node import Node
from wirehawk_msgs.msg import MotorCommand, MotorState
from std_msgs.msg import Float64MultiArray


class MotorLogger(Node):
    def __init__(self):
        super().__init__('motor_logger')
        self.declare_parameter('output_dir', os.path.expanduser('~/wirehawk_logs'))
        self.declare_parameter('flush_period_s', 1.0)
        self.declare_parameter('cmd_topic', '/cmd/motors')
        self.declare_parameter('sim_state_topic', '/sim/state/motors')
        self.declare_parameter('real_state_topic', '/real/state/motors')
        self.declare_parameter('diag_topic', '/cdpr/diagnostic')

        out = self.get_parameter('output_dir').value
        os.makedirs(out, exist_ok=True)
        stamp = time.strftime('%Y%m%d_%H%M%S')
        self.files = {
            'cmd':  open(os.path.join(out, f'cmd_{stamp}.csv'),  'w', newline=''),
            'sim':  open(os.path.join(out, f'sim_{stamp}.csv'),  'w', newline=''),
            'real': open(os.path.join(out, f'real_{stamp}.csv'), 'w', newline=''),
            'target': open(os.path.join(out, f'target_{stamp}.csv'), 'w', newline=''),
            'diag': open(os.path.join(out, f'diag_{stamp}.csv'), 'w', newline=''),
        }
        self.writers = {k: csv.writer(v) for k, v in self.files.items()}
        self.bufs = {'cmd': [], 'sim': [], 'real': [], 'target': [], 'diag': []}
        self.t0 = time.monotonic()

        self.create_subscription(
            MotorCommand, self.get_parameter('cmd_topic').value, self.on_cmd, 100)
        self.create_subscription(
            MotorState, self.get_parameter('sim_state_topic').value,
            lambda m: self.on_state('sim', m), 100)
        self.create_subscription(
            MotorState, self.get_parameter('real_state_topic').value,
            lambda m: self.on_state('real', m), 100)
        self.create_subscription(
            Float64MultiArray, self.get_parameter('diag_topic').value,
            self.on_diag, 100)
        self.create_timer(self.get_parameter('flush_period_s').value, self.flush)
        self.get_logger().info(
            f'logging cmd[{self.get_parameter("cmd_topic").value}] '
            f'sim[{self.get_parameter("sim_state_topic").value}] '
            f'real[{self.get_parameter("real_state_topic").value}] -> {out}')

    def on_cmd(self, m):
        self.bufs['cmd'].append([time.monotonic() - self.t0] + list(m.position))

    def on_state(self, kind, m):
        t = time.monotonic() - self.t0
        self.bufs[kind].append([t] + list(m.position))
        # The real bridge also publishes the physical target it wrote to 0x607A
        # (the missing signal that distinguishes a bridge/interpolator fault from
        # a drive-side DCSYNC fault). Log it alongside, same timestamp.
        if kind == 'real' and m.target:
            self.bufs['target'].append([t] + list(m.target))

    def on_diag(self, m):
        self.bufs['diag'].append([time.monotonic() - self.t0] + list(m.data))

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
