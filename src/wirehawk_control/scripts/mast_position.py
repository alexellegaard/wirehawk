#!/usr/bin/env python3
"""
Wirehawk mast-position estimator (2D; heights are NOT estimated).

Given the six pairwise horizontal distances between the four masts, estimate
their (x, y) positions by closed-form reconstruction (seed) followed by
weighted least squares.

Masts are numbered 1-4 in CLOCKWISE DAISY-CHAIN order (mast 1 = the master,
where the control box is mounted):
    mast 1 -> mast 2 -> mast 3 -> mast 4 (the tail), clockwise.

Output frame (matches cdpr_params_field.yaml and the Gazebo world):
    +x = "right" when standing behind mast 1 looking into the workspace,
    +y = "up".  mast 4 (the tail) lands at +x,+y (up-right), mast 1 at +x,-y.

Six distances (4 sides + 2 diagonals):
    d01 : mast 1 <-> mast 2   (side)
    d12 : mast 2 <-> mast 3   (side)
    d23 : mast 3 <-> mast 4   (side)
    d30 : mast 4 <-> mast 1   (side)
    d02 : mast 1 <-> mast 3   (diagonal)
    d13 : mast 2 <-> mast 4   (diagonal)

Each distance may be a single float or a list of floats (repeated
measurements, e.g. once from each end).  Repeats are averaged and their sample
standard deviation becomes the least-squares weight (1/sigma^2).

The six distances pin the quadrilateral up to rigid motion + reflection.  The
reconstruction fixes the gauge a0=(0,0), a1 on +x, a2 in the +y half-plane;
the field-frame transform below (centering + reflection + rotation) then places
it with +x = right, +y = up.  Do not change MIRROR / ROTATE_DEG without
re-checking the handedness against the config.

Stdlib only.  Run:  python3 mast_position.py
"""

import math
import random

# ----------------------------------------------------------------------------
# INPUT
# ----------------------------------------------------------------------------
MEASUREMENTS = {
    "d01": [16.765],        # mast 1 <-> mast 2   (side)
    "d12": [20.135],        # mast 2 <-> mast 3   (side)
    "d23": [20.699],        # mast 3 <-> mast 4   (side)
    "d30": [25.175],        # mast 4 <-> mast 1   (side)
    "d02": [25.415],        # mast 1 <-> mast 3   (diagonal)
    "d13": [31.494],        # mast 2 <-> mast 4   (diagonal)
}

# Field-frame transform: +x = "right" (standing behind mast 1), +y = "up",
# centroid at the origin.  Net effect below is a reflection across the x-axis
# then a 180 deg rotation => x -> -x (y unchanged), so +x ends up "right".
MIRROR = True               # reflect across the x-axis (flip handedness)
ROTATE_DEG = 180.0          # CCW rotation about the origin [deg]

Z_PLACEHOLDER = 0.0  # z value written into the printed `anchors:` line only

MONTE_CARLO_SAMPLES = 300   # uncertainty estimate; set 0 to skip
INSTRUMENT_SIGMA = 0.002    # assumed laser noise (m) fallback sigma

# ----------------------------------------------------------------------------
# Core math
# ----------------------------------------------------------------------------
PAIRS = [
    (0, 1, "d01"),
    (1, 2, "d12"),
    (2, 3, "d23"),
    (3, 0, "d30"),
    (0, 2, "d02"),
    (1, 3, "d13"),
]
KEYS = [k for _, _, k in PAIRS]
NAMES = ["mast 1", "mast 2", "mast 3", "mast 4"]


def dist(p, q):
    return math.hypot(p[0] - q[0], p[1] - q[1])


def stats(values):
    n = len(values)
    m = sum(values) / n
    if n < 2:
        return m, 0.0
    var = sum((v - m) ** 2 for v in values) / (n - 1)
    return m, math.sqrt(var)


def closed_form(d):
    """Closed-form reconstruction from the 6 mean distances (dict)."""
    d01 = d["d01"]
    if d01 <= 0:
        raise ValueError("d01 must be positive")

    a0 = (0.0, 0.0)
    a1 = (d01, 0.0)

    def place(r0, r1):
        x = (d01 * d01 + r0 * r0 - r1 * r1) / (2 * d01)
        yy = r0 * r0 - x * x
        if yy < -1e-9:
            raise ValueError("triangle inequality violated -- check measurements")
        if yy < 0:
            yy = 0.0
        return x, math.sqrt(yy)

    x2, y2 = place(d["d02"], d["d12"])      # mast 3 in the +y half-plane
    x3, y3 = place(d["d30"], d["d13"])
    plus = (x3, y3)
    minus = (x3, -y3)
    a3 = plus if abs(dist(plus, (x2, y2)) - d["d23"]) <= \
                abs(dist(minus, (x2, y2)) - d["d23"]) else minus

    return [a0, a1, (x2, y2), a3]


def unpack(p):
    return [(0.0, 0.0), (p[0], 0.0), (p[1], p[2]), (p[3], p[4])]


def residuals(p, targets, weights):
    a = unpack(p)
    r, w = [], []
    for (i, j, _k), t, wt in zip(PAIRS, targets, weights):
        r.append(dist(a[i], a[j]) - t)
        w.append(wt)
    return r, w


def cost(p, targets, weights):
    r, w = residuals(p, targets, weights)
    return sum(wi * ri * ri for ri, wi in zip(r, w))


def _jacobian(p, targets, weights, h=1e-6):
    n = len(p)
    r0, _ = residuals(p, targets, weights)
    m = len(r0)
    Jt = [[0.0] * n for _ in range(m)]
    for a in range(n):
        pp, pm = p[:], p[:]
        pp[a] += h
        pm[a] -= h
        rp, _ = residuals(pp, targets, weights)
        rm, _ = residuals(pm, targets, weights)
        for i in range(m):
            Jt[i][a] = (rp[i] - rm[i]) / (2 * h)
    return r0, Jt


def _solve(A, b):
    n = len(b)
    M = [A[i][:] + [b[i]] for i in range(n)]
    for col in range(n):
        piv = max(range(col, n), key=lambda r: abs(M[r][col]))
        if abs(M[piv][col]) < 1e-12:
            raise ValueError("singular normal-equations system")
        M[col], M[piv] = M[piv], M[col]
        for r in range(col + 1, n):
            f = M[r][col] / M[col][col]
            for c in range(col, n + 1):
                M[r][c] -= f * M[col][c]
    x = [0.0] * n
    for r in range(n - 1, -1, -1):
        x[r] = (M[r][n] - sum(M[r][c] * x[c] for c in range(r + 1, n))) / M[r][r]
    return x


def fit(p0, targets, weights, max_iter=200, tol=1e-12):
    p = p0[:]
    lam = 1e-6
    c = cost(p, targets, weights)
    for _ in range(max_iter):
        r, Jt = _jacobian(p, targets, weights)
        n, m = len(p), len(r)
        A = [[0.0] * n for _ in range(n)]
        g = [0.0] * n
        for i in range(m):
            wi = weights[i]
            for a in range(n):
                Jia = Jt[i][a]
                g[a] += Jia * wi * r[i]
                for b in range(n):
                    A[a][b] += Jia * wi * Jt[i][b]
        Aa = [A[a][:] for a in range(n)]
        for a in range(n):
            Aa[a][a] += lam
        dp = _solve(Aa, [-g[a] for a in range(n)])
        pnew = [p[a] + dp[a] for a in range(n)]
        cnew = cost(pnew, targets, weights)
        if cnew < c:
            p = pnew
            if abs(c - cnew) < tol * max(1.0, abs(c)):
                break
            lam = max(lam / 3.0, 1e-12)
            c = cnew
        else:
            lam *= 10.0
            if lam > 1e12:
                break
    return p


def transform(pts):
    """Center -> reflect (x-axis) -> rotate, placing corners in the field frame."""
    cx = sum(x for x, _ in pts) / len(pts)
    cy = sum(y for _, y in pts) / len(pts)
    out = [(x - cx, y - cy) for x, y in pts]
    if MIRROR:
        out = [(x, -y) for x, y in out]
    if ROTATE_DEG:
        th = math.radians(ROTATE_DEG)
        ct, st = math.cos(th), math.sin(th)
        out = [(x * ct - y * st, x * st + y * ct) for x, y in out]
    return out


# ----------------------------------------------------------------------------
# Main
# ----------------------------------------------------------------------------
def main():
    means, sigmas = {}, {}
    for key in KEYS:
        v = MEASUREMENTS[key]
        vals = v if isinstance(v, list) else [v]
        m, s = stats(vals)
        means[key] = m
        sigmas[key] = s

    targets = [means[k] for _, _, k in PAIRS]
    weights = [1.0 / (sigmas[k] ** 2) if sigmas[k] > 0 else 1.0 for _, _, k in PAIRS]

    try:
        seed = closed_form(means)
    except ValueError as e:
        print(f"ERROR: {e}")
        return

    p = fit([seed[1][0], seed[2][0], seed[2][1], seed[3][0], seed[3][1]],
            targets, weights)
    fitted_local = unpack(p)

    pts = transform(fitted_local)

    print("Measured vs fitted distances (m):")
    print(f"  {'pair':<6}{'reads':<24}{'mean':>10}{'sigma':>10}{'fitted':>10}{'resid':>10}")
    for (i, j, key), t in zip(PAIRS, targets):
        v = MEASUREMENTS[key]
        reads = v if isinstance(v, list) else [v]
        reads_s = ", ".join(f"{x:.3f}" for x in reads)
        fd = dist(pts[i], pts[j])
        print(f"  {key:<6}{reads_s:<24}{means[key]:>10.4f}{sigmas[key]:>10.4f}"
              f"{fd:>10.4f}{fd - t:>+10.4f}")

    print("\nMast positions (m), field frame (+x = right, +y = up):")
    for nm, (x, y) in zip(NAMES, pts):
        print(f"  {nm}: ({x: .4f}, {y: .4f})")

    flat = []
    for x, y in pts:
        flat += [f"{x:.4f}", f"{y:.4f}", f"{Z_PLACEHOLDER:g}"]
    print(f"\nYAML anchors line (z = {Z_PLACEHOLDER:g} placeholder -- heights not estimated):")
    print(f"  anchors: [{', '.join(flat)}]")

    if MONTE_CARLO_SAMPLES > 0:
        rnd = random.Random(0)
        clouds = [[] for _ in range(4)]
        for _ in range(MONTE_CARLO_SAMPLES):
            dsim = {}
            for key in KEYS:
                m, s = means[key], sigmas[key]
                noise = rnd.gauss(0.0, s if s > 0 else INSTRUMENT_SIGMA)
                dsim[key] = m + noise
            try:
                sd = closed_form(dsim)
            except ValueError:
                continue
            q = fit([sd[1][0], sd[2][0], sd[2][1], sd[3][0], sd[3][1]],
                    [dsim[k] for _, _, k in PAIRS],
                    [1.0] * len(PAIRS))
            qq = transform(unpack(q))
            for k in range(4):
                clouds[k].append(qq[k])

        print(f"\nMonte Carlo position uncertainty (1-sigma, {MONTE_CARLO_SAMPLES} samples):")
        for k, nm in enumerate(NAMES):
            c = clouds[k]
            if not c:
                continue
            n = len(c)
            mx = sum(x for x, _ in c) / n
            my = sum(y for _, y in c) / n
            sx = math.sqrt(sum((x - mx) ** 2 for x, _ in c) / n)
            sy = math.sqrt(sum((y - my) ** 2 for _, y in c) / n)
            sr = math.sqrt(sx * sx + sy * sy)
            print(f"  {nm}: sigma_x={sx:.4f}  sigma_y={sy:.4f}  radial={sr:.4f} m")


if __name__ == "__main__":
    main()
