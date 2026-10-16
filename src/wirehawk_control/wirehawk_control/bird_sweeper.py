"""Bird sweeper — the motion layer for bird deterrence.

Turns a BirdTarget (from the decision layer) into a /cmd_vel stream:
  * valid target  -> follow the arrival LINE through the bird at full speed.
                     Fly along the arrival direction (the angle that leaves
                     enough runoff behind the bird to brake) with a lateral
                     correction pulling the TCP onto the line. A closed-loop law
                     recomputed every tick: continuous in time, tracks a moving
                     bird, and converges to a closest-approach of scare_radius.
  * no target     -> gentle return to home.

Backend-agnostic: it talks only to cdpr_node via topics (BirdTarget in,
/cdpr/current_pose in, /cmd_vel out) and never knows whether the motors behind
the controller are the sim bridge or the EtherCAT bridge.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from wirehawk_msgs.msg import BirdTarget


class BirdSweeper(Node):
    def __init__(self):
        super().__init__('bird_sweeper')

        self.declare_parameter('home_position', [0.0, 0.0, 1.5])
        self.declare_parameter('return_speed', 1.5)
        self.declare_parameter('z_height', 1.5)
        self.declare_parameter('home_tolerance', 0.1)
        self.declare_parameter('decel_distance', 1.0)
        self.declare_parameter('target_timeout', 2.0)
        self.declare_parameter('rate_hz', 30.0)
        # Arrival-angle geometry. These MUST match cdpr_node's workspace_min/max
        # and max_linear_decel (same source of truth: cdpr_params_<world>.yaml).
        self.declare_parameter('workspace_min', [-19.0, -19.0, 0.3])
        self.declare_parameter('workspace_max', [19.0, 19.0, 2.7])
        self.declare_parameter('max_decel', 2.0)
        self.declare_parameter('lateral_gain', 0.5)   # [1/m] line-following pull-back gain; 1/k = turn-in distance to the line
        self.declare_parameter('scare_radius', 0.0)   # [m] flyby offset; 0 = pass through for now (tracking error dominates)

        self.home = np.array(self.get_parameter('home_position').value, dtype=float)
        self.return_speed = float(self.get_parameter('return_speed').value)
        self.z_height = float(self.get_parameter('z_height').value)
        self.home_tol = float(self.get_parameter('home_tolerance').value)
        self.decel_dist = float(self.get_parameter('decel_distance').value)
        self.timeout = float(self.get_parameter('target_timeout').value)
        self.dt = 1.0 / float(self.get_parameter('rate_hz').value)
        self.ws_min = np.array(self.get_parameter('workspace_min').value, dtype=float)
        self.ws_max = np.array(self.get_parameter('workspace_max').value, dtype=float)
        self.max_decel = float(self.get_parameter('max_decel').value)
        self.lateral_gain = float(self.get_parameter('lateral_gain').value)
        self.scare_radius = float(self.get_parameter('scare_radius').value)

        # Target state (horizontal plane only; z is fixed by z_height).
        self.target = None
        self.target_speed = 0.0
        self.target_valid = False
        self.passed = False
        self.last_target_time = self.get_clock().now()

        # Current TCP pose from the controller's FK estimate.
        self.current_pos = None

        # Commanded velocity from the previous tick, used for pass detection.
        self.cmd_vel = np.zeros(2, dtype=float)

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(BirdTarget, '/bird_target', self.target_cb, 10)
        self.create_subscription(PoseStamped, '/cdpr/current_pose', self.pose_cb, 10)
        self.timer = self.create_timer(self.dt, self.tick)

        self.get_logger().info(
            f'bird_sweeper up: home {self.home[:2].tolist()}, '
            f'z={self.z_height} m, return_speed={self.return_speed} m/s')

    def target_cb(self, msg: BirdTarget):
        new_target = np.array([msg.position.x, msg.position.y], dtype=float)
        new_valid = bool(msg.valid)
        # Re-engage on a fresh target: a new position, or a new valid edge.
        if self.target is None or not np.array_equal(new_target, self.target) \
                or (new_valid and not self.target_valid):
            self.passed = False
        self.target = new_target
        self.target_speed = float(msg.speed)
        self.target_valid = new_valid
        self.last_target_time = self.get_clock().now()

    def pose_cb(self, msg: PoseStamped):
        self.current_pos = np.array([
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
        ], dtype=float)

    def _target_fresh(self) -> bool:
        age = (self.get_clock().now() - self.last_target_time).nanoseconds / 1e9
        return age < self.timeout

    def tick(self):
        if self.current_pos is None:
            return  # no FK pose yet

        tcp_xy = self.current_pos[:2]

        if self.target_valid and self._target_fresh() and self.target is not None:
            vel = self._sweep(tcp_xy)
        else:
            vel = self._return_home(tcp_xy)

        self.cmd_vel = vel
        msg = Twist()
        msg.linear.x = float(vel[0])
        msg.linear.y = float(vel[1])
        msg.linear.z = 0.0
        self.pub.publish(msg)

    def _arrival_vector(self):
        """Velocity the sweep must arrive at the bird with, as a fraction of
        v_max (|·| <= 1): the centre-to-bird direction, each toward-wall
        component capped at the brake-able fraction, leftover filled tangentially.

        0 walls -> straight through; 1 wall -> tilted (tangential along it);
        2 walls -> a short vector (slower arrival). Continuous across all three.
        """
        nat = self.target - self.home[:2]
        n = np.linalg.norm(nat)
        if n < 1e-9:
            return np.array([1.0, 0.0])
        nat = nat / n
        vmax = self.target_speed
        a = self.max_decel

        sx = 1.0 if nat[0] >= 0.0 else -1.0
        sy = 1.0 if nat[1] >= 0.0 else -1.0
        dist_x = (self.ws_max[0] - self.target[0]) if sx > 0 else (self.target[0] - self.ws_min[0])
        dist_y = (self.ws_max[1] - self.target[1]) if sy > 0 else (self.target[1] - self.ws_min[1])
        cap_x = np.sqrt(2.0 * a * max(dist_x, 0.0)) / vmax
        cap_y = np.sqrt(2.0 * a * max(dist_y, 0.0)) / vmax

        ax = min(abs(nat[0]), cap_x)
        ay = min(abs(nat[1]), cap_y)
        if ax * ax + ay * ay < 1.0 - 1e-9:
            if ax >= cap_x - 1e-9:      # x capped -> fill tangentially in y
                ay = min(np.sqrt(max(1.0 - ax * ax, 0.0)), cap_y)
            else:                        # y capped -> fill tangentially in x
                ax = min(np.sqrt(max(1.0 - ay * ay, 0.0)), cap_x)
        return np.array([sx * ax, sy * ay])

    def _arrival_direction(self):
        """Unit direction of the arrival velocity (normalises _arrival_vector).
        The corner's |·| < 1 shortening is left to the controller's soft boundary
        to enforce; here we only need the direction."""
        v = self._arrival_vector()
        n = np.linalg.norm(v)
        if n < 1e-9:
            return np.array([1.0, 0.0])
        return v / n

    def _sweep(self, tcp_xy):
        """Line-following law: fly along the arrival line through the bird at
        v_max, with a lateral correction proportional to the lateral
        displacement vector n. Closed-loop (recomputed every tick) and
        continuous in time, so it tracks a moving bird and converges to a
        closest-approach of exactly scare_radius.

          v = v_max * (a_hat - k * n) / |a_hat - k * n|

        where a_hat is the arrival direction (unit), k the lateral gain, and
        n = (p - b) - ((p - b) . a_hat) a_hat the lateral displacement of the
        TCP p from the line through the bird b (perpendicular component; |n| is
        the lateral distance)."""
        if self.passed:
            return np.zeros(2)  # already swept through; hold

        p = np.asarray(tcp_xy, dtype=float)
        b = self.target
        a_hat = self._arrival_direction()
        # If the TCP is already past the bird (far side of the arrival line),
        # flip the line direction so it comes back AT the bird instead of
        # flying away from it. s is the along-line progress; n below is
        # sign-invariant, so only the "forward" sense changes.
        s = float(np.dot(p - b, a_hat))
        if s > 0.0:
            a_hat = -a_hat
        # n = lateral displacement: the perpendicular component of (p - b),
        # pointing from the line toward the TCP. |n| is the lateral distance.
        # (scare_radius offset = shift the line; 0 for now, so no r-term)
        n = (p - b) - np.dot(p - b, a_hat) * a_hat

        direction = a_hat - self.lateral_gain * n
        dmag = float(np.linalg.norm(direction))
        if dmag < 1e-9:
            direction = a_hat
        else:
            direction = direction / dmag
        v = self.target_speed * direction

        # Pass detection: once we're moving away from the target we've swept
        # through it — latch and hold instead of re-engaging.
        vec = b - p
        if float(np.dot(vec, self.cmd_vel)) < 0.0:
            self.passed = True
            return np.zeros(2)
        return v

    def _return_home(self, tcp_xy):
        vec = self.home[:2] - tcp_xy
        dist = float(np.linalg.norm(vec))
        if dist < self.home_tol:
            return np.zeros(2)
        # Proportional deceleration inside the braking zone so the TCP stops
        # at home instead of overshooting the deadband and oscillating.
        if dist < self.decel_dist:
            speed = self.return_speed * (dist / self.decel_dist)
            speed = max(speed, 0.05)
        else:
            speed = self.return_speed
        return speed * (vec / dist)


def main(args=None):
    rclpy.init(args=args)
    node = BirdSweeper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())  # stop on exit
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
