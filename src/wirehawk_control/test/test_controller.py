"""Controller core tests (pure numpy): closed-loop settle with counts feedback."""
import unittest
import numpy as np

from wirehawk_spool.spool_model import WinchSpec, counts_to_lengths
from wirehawk_control.controller import CDPRController


def make_controller():
    # 20x20_3m world params (matching cdpr_params_20x20_3m.yaml).
    anchors = np.array([
        [10.0, 10.0, 3.0], [10.0, -10.0, 3.0],
        [-10.0, -10.0, 3.0], [-10.0, 10.0, 3.0]])
    return CDPRController(
        anchors=anchors,
        start_pos=[0.0, 0.0, 1.5],
        ws_min=[-7.0, -7.0, 0.3], ws_max=[7.0, 7.0, 2.7],
        max_speed=1.0, max_accel=0.5, max_decel=0.5,
        EA=98100.0, mass=3.0, fk_gain=0.5,
        max_cable_speed=10485760.0, max_cable_accel=6553600.0,
        spec=WinchSpec())


class TestController(unittest.TestCase):
    def test_settles_with_perfect_tracking_feedback(self):
        c = make_controller()
        dt = 1.0 / 50.0
        # Resume first: seed the command + latch the target over a few readbacks.
        # (The elastic FK needs one fallback tick from the exact-home slack
        # config — zero tension, singular Jacobian — before it converges, so a
        # single step is not enough to complete the resume.)
        for _ in range(5):
            c.step(dt, c.command_counts)
        self.assertTrue(c.target_latched, "resume should latch the target")
        target = np.array([2.0, 1.0, 1.2])
        c.target_pos = target.copy()
        for _ in range(1000):
            # Perfect-tracking virtual winch: measured counts == last command.
            measured = c.command_counts
            c.step(dt, measured)
            self.assertTrue(np.isfinite(c.P_est).all(), "non-finite FK estimate")
        err = float(np.linalg.norm(c.P_est - target))
        self.assertLess(err, 0.05, f"settle error {err:.4f} m, P_est={c.P_est}, target={target}")

    def test_command_counts_in_range(self):
        c = make_controller()
        dt = 1.0 / 50.0
        c.target_pos = np.array([0.5, -0.5, 1.6])
        counts = c.step(dt, None)          # no feedback yet -> nominal lengths
        L = counts_to_lengths(counts, c.spec)
        self.assertTrue(np.isfinite(L).all())
        self.assertTrue(np.all(L > 0.1))
        self.assertTrue(np.all(L <= c.spec.total_m))

    def test_feedback_changes_estimate(self):
        # Longer measured lengths (more cable out) -> payload estimate drops.
        c = make_controller()
        c.target_pos = np.array([0.0, 0.0, 1.5])
        L_nominal = c.L_m.copy()
        L_slacker = L_nominal + 0.2
        from wirehawk_spool.spool_model import lengths_to_counts
        counts = lengths_to_counts(L_slacker, c.spec)
        c.step(1.0 / 50.0, counts)
        self.assertLess(c.P_est[2], 1.5, "slack cable should lower the FK estimate")


    def test_resume_seeds_command_from_feedback(self):
        # Resume startup: the first non-home feedback must seed the command to
        # the measured position (hold in place), not leave it at the home
        # command (which would slew the payload home).
        c = make_controller()
        dt = 1.0 / 50.0
        actual = np.array([2.0, 1.0, 1.2])
        lengths = c.kinematics.compute_commanded_lengths(actual)
        from wirehawk_spool.spool_model import lengths_to_counts
        measured = np.asarray(lengths_to_counts(lengths, c.spec), dtype=np.int64)

        c.step(dt, measured)
        np.testing.assert_array_equal(c.command_counts, measured)

        for _ in range(200):
            c.step(dt, measured)
        self.assertTrue(c.target_latched, "target should latch once FK converges")
        self.assertLess(np.linalg.norm(c.target_pos - c.P_est), 0.05)
        # Elastic FK settles at the sagged pose (cables stretch under load),
        # so compare against `actual` with a loose tolerance.
        self.assertLess(np.linalg.norm(c.P_est - actual), 0.2)


if __name__ == '__main__':
    unittest.main()
