import numpy as np

GRAVITY = 9.81


class CDPRKinematics:
    """
    Inverse kinematics for an N-cable CDPR with gravity-balancing pretension.

    The controller commands cable *rest lengths*. A soft-spring cable model
    (tension = K*(L_act - L0) + C*L_dot) only carries load once it stretches, so
    commanding the bare geometric length leaves zero tension at the setpoint and
    the payload sags below it. We therefore command each cable slightly SHORT of
    its geometric length, by the stretch needed to pre-load the static tension
    that balances gravity (minimum-norm cable-force distribution).
    """
    def __init__(self, anchors: np.ndarray, payload_mass: float = 10.0,
                 stiffness: float = 5000.0):
        self.anchors = np.asarray(anchors, dtype=float)
        self.num_cables = len(self.anchors)
        self.payload_mass = float(payload_mass)
        self.stiffness = float(stiffness)

    def inverse_kinematics(self, pos: np.ndarray) -> np.ndarray:
        """Nominal Euclidean distance from each anchor to payload: L_i = ||a_i - p||."""
        diffs = self.anchors - pos
        return np.linalg.norm(diffs, axis=1)

    def _pretension_forces(self, pos: np.ndarray) -> np.ndarray:
        """
        Minimum-norm cable-tension distribution that balances gravity at `pos`.

        Structure matrix A has the payload->anchor unit vectors as columns; the
        wrench balance A @ T = w (w = [0, 0, m*g]) is underdetermined for N > 3
        cables, and lstsq returns the minimum-norm (maximally balanced) solution.
        """
        geom = self.inverse_kinematics(pos)
        u = (self.anchors - pos) / geom[:, None]
        w = np.array([0.0, 0.0, self.payload_mass * GRAVITY])
        tensions = np.linalg.lstsq(u.T, w, rcond=None)[0]
        return np.clip(tensions, 0.0, None)

    def compute_commanded_lengths(self, pos: np.ndarray) -> np.ndarray:
        """
        Commanded rest length per cable: geometric length minus the pre-load
        stretch, so the payload hangs exactly at `pos` with no sag.
        """
        geom = self.inverse_kinematics(pos)
        pretension = self._pretension_forces(pos)
        return geom - pretension / self.stiffness
