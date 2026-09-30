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
        max_speed=1.0, max_accel=0.5,
        EA=98100.0, mass=3.0, fk_gain=0.5,
        spec=WinchSpec())


class TestController(unittest.TestCase):
    def test_settles_with_perfect_tracking_feedback(self):
        c = make_controller()
        target = np.array([2.0, 1.0, 1.2])
        c.target_pos = target.copy()
        dt = 1.0 / 50.0
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


if __name__ == '__main__':
    unittest.main()
