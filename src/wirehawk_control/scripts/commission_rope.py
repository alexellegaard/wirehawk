#!/usr/bin/env python3
"""Measure the TCP home using the rope as a tape measure (F30 jog + encoder).

Procedure (masts in ANY order):

  Phase 1 — zeros:
    Retract a mast so its rope end sits exactly at the pulley, then press that
    mast's number (1-4). Repeat for all four, interleaved with jogging each
    one out to the TCP and attaching it.

  Phase 2 — all attached:
    Once all four are attached, fine-jog to balance the tension / reduce sag,
    then press ENTER. The script snapshots all four TCP counts at that moment,
    converts each delta to a free-span length via the spool model, trilaterates
    the home TCP with the controller's own geometric FK, and writes
    start_position into cdpr_params_<world>.yaml.

Run on the Pi (EtherCAT master):  python3 commission_rope.py <world>

Dependencies are the same as calibrate.py (stdlib + pyyaml) so it runs on the
Pi's bare python3 — no numpy needed.

MAST_RUN below is the ONE physical constant you set once: the fixed cable
length from the winch drum to the top pulley [m] (~3 m). It fixes the drum
layer for the counts->length mapping; +/-0.5 m is plenty.
"""
import math
import os
import re
import subprocess
import sys
import termios
import tty

import yaml

REPO = os.path.expanduser("~/wirehawk")
A6_ENCODER = os.path.join(REPO, "src", "ethercat", "a6_encoder")
BRIDGE_YAML = os.path.join(
    REPO, "src", "wirehawk_ethercat_bridge", "config", "ethercat_bridge.yaml")

# --- mast run (ONE-TIME [m]): winch-drum -> top-pulley cable length. If the drum
# sits at the mast base and the pulley at the top (3 m), this is ~3 m. Sensitivity
# is ~16 cm per metre of error, so +/-0.5 m is plenty for a rough calibration.
MAST_RUN = 3.0

# --- physical mast number -> EtherCAT slave index. The anchors array is in SLAVE
# order (a0=mast1, a1=mast4, a2=mast3, a3=mast2 — the CW daisy-chain), NOT physical
# mast order. You press the PHYSICAL mast number (1-4); this maps it to the slave
# whose encoder/anchor it belongs to. Update if you re-wire the daisy-chain.
MAST_TO_SLAVE = [0, 3, 2, 1]   # mast1->slave0, mast2->slave3, mast3->slave2, mast4->slave1


# --- spool model (inlined from wirehawk_spool/spool_model.py — keep in sync) ---
def _layer_circumference(spec, layer):
    return math.pi * (spec["core_d_m"] + (2 * layer - 1) * spec["rope_d_m"])


def _wound_to_turns(wound, spec):
    remaining, turns = wound, 0.0
    for layer in range(1, spec["max_layers"] + 1):
        if remaining <= 1e-12:
            break
        circ = _layer_circumference(spec, layer)
        length = min(remaining, spec["turns_per_layer"] * circ)
        turns += length / circ
        remaining -= length
    if remaining > 1e-9:
        raise ValueError("wound exceeds spool capacity")
    return turns


def _turns_to_wound(turns, spec):
    remaining, wound = turns, 0.0
    for layer in range(1, spec["max_layers"] + 1):
        circ = _layer_circumference(spec, layer)
        if remaining <= spec["turns_per_layer"] + 1e-12:
            wound += max(0.0, remaining) * circ
            remaining = 0.0
            break
        wound += spec["turns_per_layer"] * circ
        remaining -= spec["turns_per_layer"]
    else:
        raise ValueError("turns exceeds spool capacity")
    return wound


def length_to_counts(length_m, spec):
    return int(round(_wound_to_turns(spec["total_m"] - length_m, spec)
                     * spec["counts_per_rev"] * spec["gear_ratio"]))


def counts_to_length(counts, spec):
    return spec["total_m"] - _turns_to_wound(
        counts / (spec["counts_per_rev"] * spec["gear_ratio"]), spec)


# --- geometric FK (least-squares trilateration — the controller's own fallback) ---
def _solve3(a, b):
    """Gaussian elimination with partial pivoting (3x3)."""
    M = [a[i][:] + [b[i]] for i in range(3)]
    for col in range(3):
        piv = max(range(col, 3), key=lambda r: abs(M[r][col]))
        if abs(M[piv][col]) < 1e-12:
            raise ValueError("singular normal equations")
        M[col], M[piv] = M[piv], M[col]
        for r in range(col + 1, 3):
            f = M[r][col] / M[col][col]
            for c in range(col, 4):
                M[r][c] -= f * M[col][c]
    x = [0.0] * 3
    for r in range(2, -1, -1):
        x[r] = (M[r][3] - sum(M[r][c] * x[c] for c in range(r + 1, 3))) / M[r][r]
    return x


def geometric_fk(anchors, lengths, P_init):
    P = [float(P_init[0]), float(P_init[1]), float(P_init[2])]
    for _ in range(50):
        r = [0.0] * 4
        J = [[0.0] * 3 for _ in range(4)]
        for i in range(4):
            dx = P[0] - anchors[i][0]
            dy = P[1] - anchors[i][1]
            dz = P[2] - anchors[i][2]
            C = math.sqrt(dx * dx + dy * dy + dz * dz)
            C = max(C, 1e-9)
            r[i] = C - lengths[i]
            J[i][0] = dx / C
            J[i][1] = dy / C
            J[i][2] = dz / C
        # normal equations J^T J dP = -J^T r
        A = [[0.0] * 3 for _ in range(3)]
        g = [0.0] * 3
        for i in range(4):
            for a in range(3):
                g[a] += J[i][a] * r[i]
                for b in range(3):
                    A[a][b] += J[i][a] * J[i][b]
        try:
            dP = _solve3(A, [-g[0], -g[1], -g[2]])
        except ValueError:
            break
        P[0] += dP[0]
        P[1] += dP[1]
        P[2] += dP[2]
        if math.sqrt(dP[0] ** 2 + dP[1] ** 2 + dP[2] ** 2) < 1e-9:
            break
    return P


def read_raw_positions():
    out = subprocess.run(
        ["sudo", A6_ENCODER, "eth0"],
        capture_output=True, text=True, check=True,
    ).stdout
    pos = [int(m.group(1)) for m in re.finditer(r"pos\s+0x6064\s*=\s*(-?\d+)", out)]
    if len(pos) != 4:
        raise RuntimeError(f"expected 4 slaves, parsed {len(pos)}")
    return pos


def read_direction():
    txt = open(BRIDGE_YAML).read()
    m = re.search(r"direction:\s*\[([^\]]*)\]", txt)
    if not m:
        return [1, 1, 1, 1]
    d = [(-1 if int(x.strip()) < 0 else 1) for x in m.group(1).split(",")]
    return (d + [1, 1, 1, 1])[:4]


def getch():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main():
    if len(sys.argv) != 2:
        print(__doc__)
        sys.exit(2)
    world = sys.argv[1]

    cfg_path = os.path.join(REPO, "src", "wirehawk_control", "config",
                            f"cdpr_params_{world}.yaml")
    params = yaml.safe_load(open(cfg_path))["cdpr_node"]["ros__parameters"]
    flat = params["anchors"]
    anchors = [flat[i:i + 3] for i in range(0, len(flat), 3)]
    spool = yaml.safe_load(open(
        os.path.join(REPO, "src", "wirehawk_spool", "config", "spool.yaml")))[
        "spool"]
    direction = read_direction()

    # ---- Phase 1: zeros ----
    zeros = {}
    print("\nPHASE 1 — record each mast's ZERO (rope end at the pulley).")
    print("Retract a mast to the pulley, press its number (1-4). Any order;")
    print("re-press a number to redo it. Then jog that mast out and attach it.")
    while len(zeros) < 4:
        ch = getch()
        if ch in "1234":
            mast = int(ch)
            slave = MAST_TO_SLAVE[mast - 1]
            c = read_raw_positions()[slave]
            zeros[mast] = c
            done = ",".join(str(n) for n in sorted(zeros))
            todo = ",".join(str(n) for n in range(1, 5) if n not in zeros)
            print(f"mast {mast} zero recorded = {c}   (done: {done}; left: {todo})")
        else:
            print(f"(ignored {ch!r} — press a mast number 1-4)")

    print("\nAll four zeros recorded. Jog the ropes out, attach to the TCP,")
    print("fine-tune to balance tension / reduce sag, then press ENTER.")

    # ---- Phase 2: all attached ----
    while True:
        ch = getch()
        if ch in "\r\n":
            break
        print(f"(waiting for ENTER, ignored {ch!r})")

    tcp = read_raw_positions()
    lengths = [None] * 4
    print("\nmeasured free spans:")
    for mast in range(1, 5):
        slave = MAST_TO_SLAVE[mast - 1]
        dC_logical = (tcp[slave] - zeros[mast]) * direction[slave]
        L = counts_to_length(dC_logical + length_to_counts(MAST_RUN, spool), spool) - MAST_RUN
        lengths[slave] = L
        print(f"  mast {mast} (slave {slave + 1}): zero={zeros[mast]}  tcp={tcp[slave]}  "
              f"dC={tcp[slave] - zeros[mast]:+d}  free span = {L:.3f} m")

    P_init = params["start_position"]
    home = geometric_fk(anchors, lengths, P_init)
    print(f"\nfree spans = {[round(x, 3) for x in lengths]}")
    print(f"home TCP   = {[round(x, 4) for x in home]}")
    # Safety gate: the TCP must sit BELOW the pulleys (z=3.0 m). A home with
    # z outside the workspace band means the spans are inconsistent (wrong mast
    # number, wrong MAST_RUN, or a bad zero) — and writing it would produce a
    # bad counts_offset and a runaway motor. Refuse to write.
    if not (0.3 <= home[2] <= 2.7):
        print(f"\n!! REJECTED: home z = {home[2]:.3f} m is outside the workspace band 0.3-2.7 m")
        print(f"   The pulleys are at 3.0 m, so this is physically impossible — the")
        print(f"   calibration is wrong. Check: mast numbers (MAST_TO_SLAVE), MAST_RUN,")
        print(f"   and that each zero was taken with the rope end exactly at the pulley.")
        print(f"   NOT writing start_position. Fix and re-run.")
        sys.exit(1)

    txt = open(cfg_path).read()
    txt = re.sub(r"start_position:\s*\[[^\]]*\]",
                 f"start_position: [{home[0]:.4f}, {home[1]:.4f}, {home[2]:.4f}]",
                 txt, count=1)
    open(cfg_path, "w").write(txt)
    print(f"wrote start_position to {cfg_path}")


if __name__ == "__main__":
    main()
