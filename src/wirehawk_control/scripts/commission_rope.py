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

MAST_RUN below is the ONE physical constant you set once: the fixed cable
length from the winch drum to the top pulley, per motor [m] (~3-5 m). It fixes
the drum layer for the counts->length mapping; +/-0.5 m is plenty.
"""
import math
import os
import re
import subprocess
import sys
import termios
import tty

import numpy as np
import yaml

REPO = os.path.expanduser("~/wirehawk")
A6_ENCODER = os.path.join(REPO, "src", "ethercat", "a6_encoder")
BRIDGE_YAML = os.path.join(
    REPO, "src", "wirehawk_ethercat_bridge", "config", "ethercat_bridge.yaml")

# --- mast run (ONE-TIME [m]): winch-drum -> top-pulley cable length. If the drum
# sits at the mast base and the pulley at the top (3 m), this is ~3 m. Sensitivity
# is ~16 cm per metre of error, so +/-0.5 m is plenty for a rough calibration.
MAST_RUN = 3.0


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
def geometric_fk(anchors, lengths, P_init):
    P = np.array(P_init, dtype=float)
    for _ in range(50):
        d = anchors - P
        C = np.linalg.norm(d, axis=1)
        C = np.maximum(C, 1e-9)
        u = d / C[:, None]
        r = C - lengths
        J = -u
        JtJ = J.T @ J
        Jtr = J.T @ r
        try:
            dP = np.linalg.solve(JtJ, -Jtr)
        except np.linalg.LinAlgError:
            break
        P = P + dP
        if np.linalg.norm(dP) < 1e-9:
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
    anchors = np.array(
        [params["anchors"][i:i + 3] for i in range(0, len(params["anchors"]), 3)],
        dtype=float)
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
            i = int(ch) - 1
            c = read_raw_positions()[i]
            zeros[i] = c
            done = ",".join(str(n + 1) for n in sorted(zeros))
            todo = ",".join(str(n + 1) for n in range(4) if n not in zeros)
            print(f"mast {ch} zero recorded = {c}   (done: {done}; left: {todo})")
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
    lengths = []
    print("\nmeasured free spans:")
    for i in range(4):
        dC_logical = (tcp[i] - zeros[i]) * direction[i]
        L = counts_to_length(dC_logical + length_to_counts(MAST_RUN, spool), spool) - MAST_RUN
        lengths.append(L)
        print(f"  mast {i + 1}: zero={zeros[i]}  tcp={tcp[i]}  "
              f"dC={tcp[i] - zeros[i]:+d}  free span = {L:.3f} m")

    lengths = np.array(lengths)
    P_init = np.array(params["start_position"], dtype=float)
    home = geometric_fk(anchors, lengths, P_init)
    print(f"\nfree spans = {np.round(lengths, 3)}")
    print(f"home TCP   = {np.round(home, 4)}")

    txt = open(cfg_path).read()
    txt = re.sub(r"start_position:\s*\[[^\]]*\]",
                 f"start_position: [{home[0]:.4f}, {home[1]:.4f}, {home[2]:.4f}]",
                 txt, count=1)
    open(cfg_path, "w").write(txt)
    print(f"wrote start_position to {cfg_path}")


if __name__ == "__main__":
    main()
