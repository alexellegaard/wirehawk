#!/usr/bin/env python3
"""One-time zero-ticks logger: read the raw encoder position of all 4 slaves at
"rope end at the pulley" and write them into ethercat_bridge.yaml as zero_ticks.

Run on the EtherCAT master (the Pi), where ./a6_encoder can reach the drives:

    python3 log_zeros.py

The zero ticks are CONSTANTS — they only change if a spool is replaced or a
drive is re-homed (battery-backed multi-turn encoder => survives power cycles).
The bridge subtracts them (plus mast_run) to publish the free span, so this is
the one physical measurement the bridge needs. There is no per-session
calibration: log once, and the bridge publishes free-span lengths forever after.
"""
import os
import re
import subprocess

REPO = os.path.expanduser("~/wirehawk")
A6_ENCODER = os.path.join(REPO, "src", "ethercat", "a6_encoder")
BRIDGE_YAML = os.path.join(
    REPO, "src", "wirehawk_ethercat_bridge", "config", "ethercat_bridge.yaml")


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
    print("Reading raw 0x6064 positions (each rope end must be AT the pulley)...")
    pos = read_raw_positions()
    print(f"zero_ticks (slave order 0=mast1,1=mast3,2=mast2,3=mast4) = {pos}")

    txt = open(BRIDGE_YAML).read()
    txt = re.sub(r"zero_ticks:\s*\[[^\]]*\]", f"zero_ticks: {pos}", txt, count=1)
    open(BRIDGE_YAML, "w").write(txt)
    print(f"wrote zero_ticks to {BRIDGE_YAML}")
    print("(commit + push this file to persist it past the Pi's pull-only reset)")


if __name__ == "__main__":
    main()
