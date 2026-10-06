#!/usr/bin/env python3
"""Always-on logger for cmd/motors and state/motors.

Writes two timestamped CSVs (one per topic) so every run is captured with no
manual steps. Subscribes to the same *relative* topic names as cdpr_node
('cmd/motors', 'state/motors'), so remapping the controller remaps the logger
too — it works identically on the sim and the real EtherCAT setup.

Run standalone:   ros2 run wirehawk_control motor_logger
Params:
  output_dir      (default ~/wirehawk_logs)
  flush_period_s  (default 1.0) — how often buffered rows hit disk
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

        out = self.get_parameter('output_dir').value
        os.makedirs(out, exist_ok=True)
        stamp = time.strftime('%Y%m%d_%H%M%S')
        self.cmd_f = open(os.path.join(out, f'cmd_{stamp}.csv'), 'w', newline='')
        self.state_f = open(os.path.join(out, f'state_{stamp}.csv'), 'w', newline='')
        self.cmd_w = csv.writer(self.cmd_f)
        self.state_w = csv.writer(self.state_f)
        self.cmd_buf = []
        self.state_buf = []
        self.t0 = time.monotonic()

        self.create_subscription(MotorCommand, 'cmd/motors', self.on_cmd, 100)
        self.create_subscription(MotorState, 'state/motors', self.on_state, 100)
        self.create_timer(self.get_parameter('flush_period_s').value, self.flush)
        self.get_logger().info(
            f'logging cmd/motors + state/motors to {out} '
            f'(cmd_{stamp}.csv, state_{stamp}.csv)')

    def on_cmd(self, m):
        self.cmd_buf.append([time.monotonic() - self.t0] + list(m.position))

    def on_state(self, m):
        self.state_buf.append([time.monotonic() - self.t0] + list(m.position))

    def flush(self):
        self.cmd_w.writerows(self.cmd_buf)
        self.cmd_buf.clear()
        self.state_w.writerows(self.state_buf)
        self.state_buf.clear()
        self.cmd_f.flush()
        self.state_f.flush()

    def finalize(self):
        self.flush()
        self.cmd_f.close()
        self.state_f.close()


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
