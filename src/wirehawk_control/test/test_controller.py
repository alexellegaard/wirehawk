"""Controller core tests (pure numpy): closed-loop settle with free-span feedback."""
import unittest
import numpy as np

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
        max_cable_speed=6.0, max_cable_accel=4.0)


class TestController(unittest.TestCase):
    def test_settles_with_perfect_tracking_feedback(self):
        c = make_controller()
        dt = 1.0 / 50.0
        # Resume first: seed the command + latch the target over a few readbacks.
        for _ in range(5):
            c.step(dt, c.command_lengths)
        self.assertTrue(c.target_latched, "resume should latch the target")
        target = np.array([2.0, 1.0, 1.2])
        # Drive toward the target via the velocity input (the realistic path the
        # teleop/planner uses) — a direct target_pos write is a step the slew
        # limiter would never produce, and it overshoots the cable-accel clamp.
        for _ in range(3000):
            c.set_target_velocity(np.clip((target - c.P_est) * 2.0, -0.5, 0.5))
            # Perfect-tracking virtual winch: measured lengths == last command.
            c.step(dt, c.command_lengths)
            self.assertTrue(np.isfinite(c.P_est).all(), "non-finite FK estimate")
        err = float(np.linalg.norm(c.P_est - target))
        self.assertLess(err, 0.05, f"settle error {err:.4f} m, P_est={c.P_est}, target={target}")

    def test_command_lengths_in_range(self):
        c = make_controller()
        dt = 1.0 / 50.0
        c.target_pos = np.array([0.5, -0.5, 1.6])
        lengths = c.step(dt, None)          # no feedback yet -> nominal lengths
        self.assertTrue(np.isfinite(lengths).all())
        self.assertTrue(np.all(lengths > 0.1))
        self.assertTrue(np.all(lengths <= 60.0))

    def test_feedback_changes_estimate(self):
        # Longer measured lengths (more cable out) -> payload estimate drops.
        c = make_controller()
        c.target_pos = np.array([0.0, 0.0, 1.5])
        L_nominal = c.L_m.copy()
        L_slacker = L_nominal + 0.2
        c.step(1.0 / 50.0, L_slacker)
        self.assertLess(c.P_est[2], 1.5, "slack cable should lower the FK estimate")

    def test_resume_seeds_command_from_feedback(self):
        # Resume startup: the first non-home feedback must seed the command to
        # the measured lengths (hold in place), not leave it at the home command
        # (which would slew the payload home).
        c = make_controller()
        dt = 1.0 / 50.0
        actual = np.array([2.0, 1.0, 1.2])
        lengths = c.kinematics.compute_commanded_lengths(actual)
        c.step(dt, lengths)
        np.testing.assert_allclose(c.command_lengths, lengths)

        for _ in range(200):
            c.step(dt, lengths)
        self.assertTrue(c.target_latched, "target should latch once FK converges")
        # The latched target is the geometric (taut-subset) estimate; the elastic
        # FK then sags ~6 cm under the 3 kg mass, so allow that.
        self.assertLess(np.linalg.norm(c.target_pos - c.P_est), 0.1)
        self.assertLess(np.linalg.norm(c.P_est - actual), 0.2)


if __name__ == '__main__':
    unittest.main()
