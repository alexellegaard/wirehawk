import sys
import termios
import tty
import select
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

Speed Adjustment:
   1 : 0.2 m/s (Slow / Fine precision)
   2 : 0.5 m/s (Medium)
   3 : 1.0 m/s (Fast)

SPACEBAR : Stop immediately
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
    '1': 0.2,
    '2': 0.5,
    '3': 1.0,
}


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')

        # Publish explicitly to /cmd_vel
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Default linear speed (m/s)
        self.speed = 0.5
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0

        # High publish rate (30 Hz) keeps the controller watchdog happy
        self.timer = self.create_timer(1.0 / 30.0, self.timer_publish)

        # Terminal state setup for non-blocking single key reads
        self.orig_term_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        print(HELP_MSG)
        print(f"Current Speed: {self.speed} m/s")

    def timer_publish(self):
        msg = Twist()
        msg.linear.x = self.vx
        msg.linear.y = self.vy
        msg.linear.z = self.vz
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
                    self.vx = dx * self.speed
                    self.vy = dy * self.speed
                    self.vz = dz * self.speed
                    print(f"\rMove -> X:{self.vx:+.1f} | Y:{self.vy:+.1f} | Z:{self.vz:+.1f} m/s", end="", flush=True)

                elif key in SPEED_BINDINGS:
                    self.speed = SPEED_BINDINGS[key]
                    # Rescale active velocities if currently moving
                    norm = max(abs(self.vx), abs(self.vy), abs(self.vz))
                    if norm > 0.0:
                        self.vx = (self.vx / norm) * self.speed
                        self.vy = (self.vy / norm) * self.speed
                        self.vz = (self.vz / norm) * self.speed
                    print(f"\rSpeed updated: {self.speed} m/s                         ", end="", flush=True)

                elif key == ' ':
                    self.vx = 0.0
                    self.vy = 0.0
                    self.vz = 0.0
                    print(f"\rSTOPPED                                           ", end="", flush=True)

        finally:
            # Send one last zero velocity stop message
            stop_msg = Twist()
            self.pub.publish(stop_msg)
            # Restore terminal attributes so terminal isn't broken upon exit
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.orig_term_settings)
            print("\nExited teleop.")


def main(args=None):
    rclpy.init(args=args)
    teleop_node = KeyboardTeleop()
    try:
        teleop_node.run()
    except Exception as e:
        print(f"\nError: {e}")
    finally:
        teleop_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()