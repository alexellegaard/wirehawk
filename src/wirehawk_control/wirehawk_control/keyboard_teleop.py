import sys
import termios
import tty
import select
import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

HELP_MSG = """
--------------------------------------------------
Wirehawk CDPR 3D Keyboard Teleop
--------------------------------------------------
Movement:
        w
   a    s    d     (X / Y planar movement)

        e          (+Z Up)
        q          (-Z Down)

Speed (m/s):
   1:0.2   2:0.5   3:1.0   4:2.0   5:5.0

SPACEBAR : Stop (ramped)
CTRL+C   : Quit
--------------------------------------------------
"""

# Normalized direction vectors (vx, vy, vz)
MOVE_BINDINGS = {
    'w': ( 1.0,  0.0,  0.0),
    's': (-1.0,  0.0,  0.0),
    'a': ( 0.0,  1.0,  0.0),
    'd': ( 0.0, -1.0,  0.0),
    'e': ( 0.0,  0.0,  1.0),
    'q': ( 0.0,  0.0, -1.0),
}

SPEED_BINDINGS = {
    '1': 0.2, '2': 0.5, '3': 1.0, '4': 2.0, '5': 5.0,
}


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')

        # Publish explicitly to /cmd_vel
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Rate-limited velocity: the smoothing lives HERE (control layer), so
        # sim and real both receive a smooth /cmd_vel and the bridges stay dumb
        # pass-throughs + a motor-level safety clamp. Params come from the
        # cdpr_params_<world>.yaml (single source of truth).
        self.declare_parameter('max_linear_speed', 5.0)   # m/s cap
        self.declare_parameter('max_linear_accel', 2.0)   # m/s^2 ramp rate
        self.declare_parameter('rate_hz', 30.0)
        self.max_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_accel = float(self.get_parameter('max_linear_accel').value)
        self.dt = 1.0 / float(self.get_parameter('rate_hz').value)

        # Default linear speed (m/s), target velocity (keys), actual velocity (ramped)
        self.speed = 0.5
        self.target_vel = np.zeros(3, dtype=float)
        self.vel = np.zeros(3, dtype=float)

        # High publish rate (30 Hz) keeps the controller watchdog happy
        self.timer = self.create_timer(self.dt, self.timer_publish)

        # Terminal state setup for non-blocking single key reads
        self.orig_term_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        print(HELP_MSG)
        self._print_status('idle')

    def _print_status(self, note=''):
        """Single status line: current speed + commanded velocity, updated in place."""
        vel = f"Move -> X:{self.vel[0]:+.1f} | Y:{self.vel[1]:+.1f} | Z:{self.vel[2]:+.1f} m/s"
        print(f"\r[Speed {self.speed:g} m/s] {vel}  {note}", end='', flush=True)

    def timer_publish(self):
        # Ramp the actual velocity toward the key target, bounded by max_accel.
        dv = self.target_vel - self.vel
        mag = float(np.linalg.norm(dv))
        max_dv = self.max_accel * self.dt
        if mag > max_dv:
            self.vel = self.vel + (dv / mag) * max_dv
        else:
            self.vel = self.target_vel.copy()

        msg = Twist()
        msg.linear.x = float(self.vel[0])
        msg.linear.y = float(self.vel[1])
        msg.linear.z = float(self.vel[2])
        self.pub.publish(msg)

    def run(self):
        try:
            while rclpy.ok():
                # Spin once to process ROS callbacks (non-blocking)
                rclpy.spin_once(self, timeout_sec=0.01)

                # Check if a character is ready on stdin
                rlist, _, _ = select.select([sys.stdin], [], [], 0.02)
                if not rlist:
                    continue

                key = sys.stdin.read(1)

                # Exit cleanly on Ctrl+C
                if key == '\x03':
                    break

                if key in MOVE_BINDINGS:
                    dx, dy, dz = MOVE_BINDINGS[key]
                    self.target_vel = np.array([dx, dy, dz], dtype=float) * self.speed
                    self._print_status()

                elif key in SPEED_BINDINGS:
                    self.speed = SPEED_BINDINGS[key]
                    # Rescale active target if currently moving (use true magnitude)
                    norm = float(np.linalg.norm(self.target_vel))
                    if norm > 0.0:
                        self.target_vel = (self.target_vel / norm) * self.speed
                        self._print_status()
                    else:
                        self._print_status('(applies on next move key)')

                elif key == ' ':
                    self.target_vel = np.zeros(3)
                    self._print_status('STOPPED')

        finally:
            # Restore the terminal FIRST — a failed publish must not strand it in cbreak.
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.orig_term_settings)
            try:
                self.pub.publish(Twist())  # one last zero-velocity stop
            except Exception:
                pass
            print('\nExited teleop.')


def main(args=None):
    rclpy.init(args=args)
    teleop_node = KeyboardTeleop()
    try:
        teleop_node.run()
    except Exception as e:
        print(f'\nError: {e}')
    finally:
        teleop_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
