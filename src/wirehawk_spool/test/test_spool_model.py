"""Unit tests for the shared spool model (pure numpy, no ROS)."""
import unittest
import numpy as np

from wirehawk_spool.spool_model import (
    WinchSpec, length_to_counts, counts_to_length,
    lengths_to_counts, counts_to_lengths,
)


class TestSpoolModel(unittest.TestCase):
    def setUp(self):
        self.spec = WinchSpec()

    def test_round_trip_scalar(self):
        for L in [0.0, 0.5, 2.0, 10.0, 20.0, 40.0, 60.0, 65.0]:
            C = length_to_counts(L, self.spec)
            L2 = counts_to_length(C, self.spec)
            self.assertLess(abs(L2 - L), 1e-3, f"L={L} -> {L2} (counts {C})")

    def test_monotonic(self):
        # counts decrease as paid-out length increases (0 at fully unwound).
        Cs = [length_to_counts(L, self.spec) for L in np.linspace(0, self.spec.total_m, 50)]
        self.assertTrue(all(b < a for a, b in zip(Cs, Cs[1:])))

    def test_bounds(self):
        self.assertEqual(length_to_counts(self.spec.total_m, self.spec), 0)  # fully unwound
        self.assertAlmostEqual(counts_to_length(0, self.spec), self.spec.total_m, places=6)
        C_max = length_to_counts(0.0, self.spec)                              # fully wound
        self.assertAlmostEqual(counts_to_length(C_max, self.spec), 0.0, delta=1e-3)

    def test_capacity_exceeded_raises(self):
        with self.assertRaises(ValueError):
            length_to_counts(self.spec.total_m + 0.1, self.spec)

    def test_vectorized_round_trip(self):
        L = np.array([1.0, 5.0, 20.0, 45.0, 64.0])
        C = lengths_to_counts(L, self.spec)
        L2 = counts_to_lengths(C, self.spec)
        self.assertTrue(np.allclose(L, L2, atol=1e-3))

    def test_spec_validation(self):
        with self.assertRaises(ValueError):
            WinchSpec(pitch_m=0.001)   # pitch smaller than rope diameter
        with self.assertRaises(ValueError):
            WinchSpec(flange_d_m=0.02)  # flange too small for the layers


if __name__ == '__main__':
    unittest.main()
