import numpy as np

_GRAVITY = 9.80665  # m/s^2 — physical constant, matches the SDF <gravity>


class CDPRKinematics:
    """
    Geometric inverse kinematics + two forward-kinematics estimators for an
    N-cable CDPR. The controller consumes free-span cable lengths (anchor/pulley
    -> TCP, in metres) — the raw-encoder -> free-span conversion lives in the
    bridge, not here.

    IK is pure geometry (L_i = ||a_i - p||). FK has three flavours:
      * `trilateration` — pure distance least-squares, no forces, no mass. Needs
        only a rough seed. Used as the fallback and for the startup subset search.
      * `static_equilibrium_fk` — a self-contained elastic solve: given the
        measured lengths L_i, the cable axial stiffness EA and the payload mass,
        find the position P where the straight elastic cable forces balance
        gravity. Cable self-weight sag is NOT modelled; axial stretch is. Tension
        is an internal variable of the solve, not a measurement.
      * `taut_subset_fk` — startup estimate: trilaterate each 3-cable subset,
        keep the candidates where the excluded cable is slack, pick the lowest z
        (minimum potential energy). Robust to one loose cable and no initial guess.
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

    def cable_tensions(self, pos: np.ndarray, lengths: np.ndarray, EA: float) -> np.ndarray:
        """Elastic cable tensions T_i = EA * max(C_i - L_i, 0) / L_i at a position.

        T_i == 0 means the cable is slack (the measured length exceeds the
        straight-line distance to the anchor), which is the observable sign that
        the payload is hanging from fewer than all cables — the FK loses
        observability. Used as the workspace-boundary slack indicator.
        """
        lengths = np.asarray(lengths, dtype=float)
        pos = np.asarray(pos, dtype=float)
        d = self.anchors - pos
        c = np.linalg.norm(d, axis=1)
        c = np.maximum(c, 1e-9)
        stretch = c - lengths
        return EA * np.maximum(stretch, 0.0) / np.maximum(lengths, 1e-9)

    def trilateration(self, lengths, P_init, idx=None):
        """
        Least-squares trilateration: min_P sum_i (||a_i - P|| - L_i)^2 over the
        anchors in `idx` (default: all). Pure geometry — no mass, no tension, no
        equilibrium assumption. Gauss-Newton; needs a rough seed only.
        """
        lengths = np.asarray(lengths, dtype=float)
        if idx is None:
            idx = list(range(self.num_cables))
        A = self.anchors[idx]
        L = lengths
        P = np.array(P_init, dtype=float)
        for _ in range(50):
            d = A - P
            C = np.linalg.norm(d, axis=1)
            C = np.maximum(C, 1e-9)
            u = d / C[:, None]          # (n,3) unit vectors from payload to anchors
            r = C - L                   # (n,) residuals
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

    def static_equilibrium_fk(self, lengths, EA, mass, P_init,
                              gravity=_GRAVITY, max_iter=40, tol=1e-9,
                              max_jump=1.0):
        """
        Estimate the payload position from the measured cable lengths by solving
        the elastic force balance (straight elastic cables, no self-weight sag):

            sum_i T_i * u_i = m * g * z_hat

        with  T_i = EA * max(0, C_i - L_i)/L_i,  C_i = ||a_i - P||,  u_i = (a_i - P)/C_i.

        Self-contained: needs only anchors, lengths, EA and payload mass. Tension
        is computed from the elastic law inside the solve, never measured. Falls
        back to a least-squares geometric estimate when Newton does not converge,
        or when it converges to a spurious equilibrium far from the initial guess
        (deep-slack configurations admit a non-physical below-floor root).

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
            # A slack cable has zero tension, so the elastic force balance has no
            # unique equilibrium (ill-conditioned Newton). Fall back to the ROBUST
            # subset trilateration, which ignores the slack cable — NOT the full
            # least-squares, which averages the slack in and lands ~1 m off
            # (below floor). Return its validity flag (True = a consistent taut
            # subset was found).
            P, converged = self.taut_subset_fk(lengths, P_init)

        return P, converged

    def taut_subset_fk(self, lengths, P_init=None):
        """
        Startup FK: trilaterate each 3-cable subset, keep the candidates where the
        excluded cable is slack (its measured length >= the straight-line distance
        to the candidate, within tolerance), and pick the lowest z — the payload
        hangs as low as the taut cables allow (minimum potential energy). Handles
        one loose/slack cable and needs no initial guess.

        Returns (P, converged). converged=True when a consistent subset was found.
        """
        lengths = np.asarray(lengths, dtype=float)
        if P_init is None:
            P_init = np.array([self.anchors[:, 0].mean(),
                               self.anchors[:, 1].mean(), 1.5], dtype=float)
        best = None
        best_z = np.inf
        for excl in range(self.num_cables):
            keep = [i for i in range(self.num_cables) if i != excl]
            P = self.trilateration(lengths[keep], P_init, idx=keep)
            d = np.linalg.norm(self.anchors - P, axis=1)
            slack = lengths - d
            # the excluded cable must be slack (loose): L_excl >= d_excl - tol
            if slack[excl] >= -0.05:
                if P[2] < best_z:
                    best = P
                    best_z = P[2]
        if best is not None:
            return best, True
        # No consistent 3-cable subset: the measured lengths are mutually
        # inconsistent (two or more slack cables, or a hardware fault). Do NOT
        # degrade to the full least-squares — it averages the inconsistency into
        # a wrong pose. Return the seed unconverged so the controller holds.
        return np.array(P_init, dtype=float), False
