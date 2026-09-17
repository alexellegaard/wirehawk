import numpy as np

_GRAVITY = 9.80665  # m/s^2 — physical constant, matches the SDF <gravity>


class CDPRKinematics:
    """
    Geometric inverse kinematics + elastic forward kinematics for an N-cable CDPR.

    IK is pure geometry (L_i = ||a_i - p||). FK is a *self-contained* elastic
    solve: given the measured cable lengths L_i, the cable axial stiffness EA and
    the payload mass, find the payload position P where the straight elastic cable
    forces balance gravity. Cable self-weight sag is NOT modelled (its effect on
    position is sub-mm for this system); axial stretch (the dominant correction)
    is. Tension is an internal variable of the solve, not a measurement.
    """
    def __init__(self, anchors: np.ndarray):
        self.anchors = np.asarray(anchors, dtype=float)
        self.num_cables = len(self.anchors)

    def inverse_kinematics(self, pos: np.ndarray) -> np.ndarray:
        """Nominal Euclidean distance from each anchor to the payload."""
        diffs = self.anchors - pos
        return np.linalg.norm(diffs, axis=1)

    def compute_commanded_lengths(self, pos: np.ndarray) -> np.ndarray:
        """Direct 1:1 mapping to geometric lengths."""
        return self.inverse_kinematics(pos)

    def anchor_unit_vectors(self, pos: np.ndarray) -> np.ndarray:
        """Unit vectors u_i = (a_i - p)/||a_i - p|| pointing from payload to anchors."""
        d = self.anchors - pos
        c = np.linalg.norm(d, axis=1)
        c = np.maximum(c, 1e-9)
        return d / c[:, None]

    def elastic_forward_kinematics(self, lengths, EA, mass, P_init,
                                   gravity=_GRAVITY, max_iter=40, tol=1e-9,
                                   max_jump=1.0):
        """
        Estimate the payload position from the measured cable lengths by solving the
        elastic force balance (straight elastic cables, no self-weight sag):

            sum_i T_i * u_i = m * g * z_hat

        with  T_i = EA * max(0, C_i - L_i)/L_i,  C_i = ||a_i - P||,  u_i = (a_i - P)/C_i.

        Self-contained: needs only anchors, lengths, EA and payload mass. Tension is
        computed from the elastic law inside the solve, never measured. Falls back to
        a least-squares geometric estimate when Newton does not converge, or when it
        converges to a spurious equilibrium far from the initial guess (deep-slack
        configurations admit a non-physical below-floor root).

        Returns (P, converged).
        """
        lengths = np.asarray(lengths, dtype=float)
        P = np.array(P_init, dtype=float)
        gz = np.array([0.0, 0.0, mass * gravity])

        def residual(p):
            d = self.anchors - p
            C = np.linalg.norm(d, axis=1)
            C = np.maximum(C, 1e-9)
            u = d / C[:, None]
            stretch = C - lengths
            T = EA * np.maximum(stretch, 0.0) / np.maximum(lengths, 1e-9)
            return (T[:, None] * u).sum(axis=0) - gz

        converged = False
        for _ in range(max_iter):
            R = residual(P)
            if np.linalg.norm(R) < tol:
                converged = True
                break
            J = np.zeros((3, 3))
            for j in range(3):
                step = 1e-6 * abs(P[j]) + 1e-9
                Pp = P.copy(); Pp[j] += step
                Pm = P.copy(); Pm[j] -= step
                J[:, j] = (residual(Pp) - residual(Pm)) / (2.0 * step)
            try:
                dP = np.linalg.solve(J, -R)
            except np.linalg.LinAlgError:
                break
            if not np.all(np.isfinite(dP)):
                break
            P = P + dP

        # Reject a converged-but-spurious root far from the trusted initial guess
        # (deep-slack admits a below-floor equilibrium). Checked on the FINAL result,
        # not intermediate steps — the first Newton step at the slack boundary is
        # legitimately large (ill-conditioned Jacobian) but converges back nearby.
        if converged and np.linalg.norm(P - P_init) > max_jump:
            converged = False

        if not converged:
            P = self.geometric_forward_kinematics(lengths, P_init)

        return P, converged

    def geometric_forward_kinematics(self, lengths, P_init):
        """
        Least-squares trilateration: min_P sum_i (||a_i - P|| - L_i)^2.
        Gauss-Newton, used as the initial guess / fallback when the elastic solve
        has no equilibrium (all cables slack).
        """
        lengths = np.asarray(lengths, dtype=float)
        P = np.array(P_init, dtype=float)
        for _ in range(20):
            d = self.anchors - P
            C = np.linalg.norm(d, axis=1)
            C = np.maximum(C, 1e-9)
            u = d / C[:, None]          # (N,3) unit vectors from payload to anchors
            r = C - lengths             # (N,) residuals
            J = -u                      # dr_i/dP = -u_i
            JtJ = J.T @ J
            Jtr = J.T @ r
            try:
                dP = np.linalg.solve(JtJ, -Jtr)
            except np.linalg.LinAlgError:
                break
            if not np.all(np.isfinite(dP)):
                break
            P = P + dP
            if np.linalg.norm(dP) < 1e-9:
                break
        return P
