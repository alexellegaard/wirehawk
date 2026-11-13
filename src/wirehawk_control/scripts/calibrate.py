#!/usr/bin/env python3
"""One-shot encoder calibration: read the absolute encoders, compute the bridge
counts_offset, and write it into ethercat_bridge.yaml — no manual copy-paste.

Run on the EtherCAT master (the Pi), where ./a6_encoder can reach the drives:

    python3 calibrate.py <world>        # e.g. 40x40_3m

Does:
  1. read the raw 0x6064 position of all 4 slaves (via `sudo ./a6_encoder eth0`)
  2. compute counts(IK(home)) from the world anchors + home + spool geometry
  3. counts_offset = raw - counts(IK(home)), so at home the bridge publishes the
     logical home counts and the controller's FK sees home
  4. write counts_offset into ethercat_bridge.yaml in place

The drum-winding (layer) model is inlined from
wirehawk_spool/spool_model.py:length_to_counts — keep the two in sync.
"""
import math
import os
import re
import subprocess
import sys

import yaml

REPO = os.path.expanduser("~/wirehawk")
A6_ENCODER = os.path.join(REPO, "src", "ethercat", "a6_encoder")
BRIDGE_YAML = os.path.join(
    REPO, "src", "wirehawk_ethercat_bridge", "config", "ethercat_bridge.yaml")


def length_to_counts(length_m, spool):
    """Inlined from wirehawk_spool/spool_model.py:length_to_counts."""
    wound = spool["total_m"] - length_m
    turns = 0.0
    remaining = wound
    for layer in range(1, spool["max_layers"] + 1):
        if remaining <= 1e-12:
            break
        circ = math.pi * (spool["core_d_m"] + (2 * layer - 1) * spool["rope_d_m"])
        length = min(remaining, spool["turns_per_layer"] * circ)
        turns += length / circ
        remaining -= length
    if remaining > 1e-9:
        raise ValueError(f"cable {wound:.3f} m wound exceeds spool capacity")
    return int(round(turns * spool["counts_per_rev"] * spool["gear_ratio"]))


def read_raw_positions():
    out = subprocess.run(
        ["sudo", A6_ENCODER, "eth0"],
        capture_output=True, text=True, check=True,
    ).stdout
    pos = [int(m.group(1)) for m in re.finditer(r"pos\s+0x6064\s*=\s*(-?\d+)", out)]
    if len(pos) != 4:
        raise RuntimeError(f"expected 4 slaves, parsed {len(pos)}:\n{out}")
    return pos


def main():
    if len(sys.argv) != 2:
        print(__doc__)
        sys.exit(2)
    world = sys.argv[1]

    params = yaml.safe_load(open(
        os.path.join(REPO, "src", "wirehawk_control", "config",
                     f"cdpr_params_{world}.yaml")))["cdpr_node"]["ros__parameters"]
    anchors = [params["anchors"][i:i + 3] for i in range(0, len(params["anchors"]), 3)]
    home = params["start_position"]
    spool = yaml.safe_load(open(
        os.path.join(REPO, "src", "wirehawk_spool", "config", "spool.yaml")))["spool"]

    home_counts = [length_to_counts(math.dist(a, home), spool) for a in anchors]
    raw = read_raw_positions()

    # Direction-aware offset. The bridge publishes logical = (raw - offset) *
    # direction (see soem_rt_loop.cpp), so for the logical to read
    # counts(IK(home)) at home for ANY winding sign the offset must be
    #     offset = raw - direction * counts(IK(home))
    # A naive raw - counts(IK(home)) is only right for direction = +1. For a
    # flipped motor it produces a NEGATIVE feedback count, which
    # counts_to_lengths rejects — and worse, if it ever gets through, the
    # controller can command a runaway wind-in (this broke a spool in the field).
    txt = open(BRIDGE_YAML).read()
    m = re.search(r"direction:\s*\[([^\]]*)\]", txt)
    direction = ([(-1 if int(x.strip()) < 0 else 1) for x in m.group(1).split(",")]
                 if m else [1, 1, 1, 1])
    while len(direction) < 4:
        direction.append(1)
    offset = [r - d * c for r, d, c in zip(raw, direction, home_counts)]

    txt = re.sub(r"counts_offset:\s*\[[^\]]*\]", f"counts_offset: {offset}", txt, count=1)
    open(BRIDGE_YAML, "w").write(txt)

    print(f"world           = {world}")
    print(f"raw (0x6064)    = {raw}")
    print(f"counts(IK(home))= {home_counts}")
    print(f"direction       = {direction}")
    print(f"counts_offset   = {offset}")
    print(f"written to      = {BRIDGE_YAML}")
    print("(commit + push this file later to persist it past the Pi's pull-only reset)")


if __name__ == "__main__":
    main()
