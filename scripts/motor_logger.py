#!/usr/bin/env python3
"""Record cmd/motors and state/motors to CSV for a fixed duration — the
no-hassle replacement for the `ros2 topic echo ... & ... ; kill` dance.

Usage:
    python3 scripts/motor_logger.py [duration_s] [cmd_topic] [state_topic] [prefix]

Defaults: 10 s, /cmd/motors, /real/state/motors, /tmp/motor_log_
Writes <prefix>cmd.csv and <prefix>state.csv (each row: t, pos0, pos1, ...).

Run it, do your WASD move, and it stops on its own.
"""
import sys
import csv
import time

import rclpy
from rclpy.node import Node
from wirehawk_msgs.msg import MotorCommand, MotorState


class MotorLogger(Node):
    def __init__(self, duration, cmd_topic, state_topic, prefix):
        super().__init__('motor_logger')
        self.duration = duration
        self.prefix = prefix
        self.t0 = time.monotonic()
        self.cmd_rows = []
        self.state_rows = []
        self.create_subscription(MotorCommand, cmd_topic, self.on_cmd, 10)
        self.create_subscription(MotorState, state_topic, self.on_state, 10)
        self.timer = self.create_timer(0.05, self.maybe_done)

    def on_cmd(self, m):
        self.cmd_rows.append([time.monotonic() - self.t0] + list(m.position))

    def on_state(self, m):
        self.state_rows.append([time.monotonic() - self.t0] + list(m.position))

    def maybe_done(self):
        if time.monotonic() - self.t0 >= self.duration:
            with open(self.prefix + 'cmd.csv', 'w', newline='') as f:
                csv.writer(f).writerows(self.cmd_rows)
            with open(self.prefix + 'state.csv', 'w', newline='') as f:
                csv.writer(f).writerows(self.state_rows)
            self.get_logger().info(
                f'wrote {len(self.cmd_rows)} cmd rows and '
                f'{len(self.state_rows)} state rows to {self.prefix}*.csv')
            self.destroy_node()
            rclpy.shutdown()


def main():
    rclpy.init()
    args = sys.argv[1:]
    duration = float(args[0]) if len(args) > 0 else 10.0
    cmd_topic = args[1] if len(args) > 1 else '/cmd/motors'
    state_topic = args[2] if len(args) > 2 else '/real/state/motors'
    prefix = args[3] if len(args) > 3 else '/tmp/motor_log_'
    rclpy.spin(MotorLogger(duration, cmd_topic, state_topic, prefix))


if __name__ == '__main__':
    main()
