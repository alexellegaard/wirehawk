#!/usr/bin/env python3
"""Open-loop per-motor direction check (safe: cables disconnected).

Drives enabled via the bridge, cdpr_node DOWN (so nothing else commands).

For each motor in turn this commands a small wind-IN (+DELTA logical counts),
pauses so you can watch the rope, then returns to the start. Expected: the rope
winds IN (shortens) on +DELTA. If a motor winds OUT instead, that motor's
`direction` sign in ethercat_bridge.yaml is wrong — flip it and re-calibrate.

Topics match the bridge's internal names. If your bridge is remapped, pass the
same remapping here, e.g. (the runbook bridge publishes on real/state/motors):

    ros2 run wirehawk_control motor_direction_check --ros-args \
        -r state/motors:=real/state/motors
"""
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node

from wirehawk_msgs.msg import MotorCommand, MotorState

DELTA = 1000000       # ~0.48 m on the first layer (within the bridge's 5M max_step)
HOLD_S = 4            # seconds to watch the rope while it winds in
NUM_MOTORS = 4


class DirectionCheck(Node):
    def __init__(self):
        super().__init__('motor_direction_check')
        self.cmd_pub = self.create_publisher(MotorCommand, 'cmd/motors', 10)
        self.state_sub = self.create_subscription(
            MotorState, 'state/motors', self.state_cb, 10)
        self.current = None

    def state_cb(self, msg):
        self.current = np.asarray(msg.position, dtype=np.int64)

    def send(self, pos):
        m = MotorCommand()
        m.position = [int(p) for p in pos]
        self.cmd_pub.publish(m)

    def wait_current(self, timeout=8.0):
        t0 = time.time()
        while self.current is None and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
        return self.current is not None

    def run(self):
        if not self.wait_current():
            self.get_logger().error(
                'no state/motors received — is the bridge running and remapped correctly?')
            return
        base = self.current.copy()
        self.get_logger().info(f'start (logical) counts = {base.tolist()}')
        for i in range(NUM_MOTORS):
            self.get_logger().info(
                f'=== motor {i} (mast {i + 1}): command +{DELTA} counts = wind IN ===')
            target = base.copy()
            target[i] += DELTA
            self.send(target)
            for s in range(HOLD_S, 0, -1):
                self.get_logger().info(f'    watch rope {s}s ...')
                time.sleep(1.0)
            self.send(base)   # return to start
            time.sleep(1.0)
        self.get_logger().info(
            'done. Any motor that wound OUT instead of IN has the wrong '
            '`direction` sign — flip it in ethercat_bridge.yaml and re-calibrate.')


def main():
    rclpy.init(args=sys.argv)
    node = DirectionCheck()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
