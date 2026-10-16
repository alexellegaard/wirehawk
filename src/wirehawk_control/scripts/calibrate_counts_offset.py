#!/usr/bin/env python3
"""Compute the bridge counts_offset so the absolute encoder maps to the spool
zero (cable-length) frame.

Usage:
    calibrate_counts_offset.py <world> <raw0> <raw1> <raw2> <raw3>

<world> matches cdpr_params_<world>.yaml (anchors + start_position).
<rawN>  are the raw absolute encoder positions from `sudo ./a6_encoder eth0`
        (the 0x6063/0x6064 readback, BEFORE any offset).

counts_offset = raw - counts(IK(home)), so at the home pose the bridge
publishes logical counts == counts(IK(home)) and the controller's FK sees home.
Write the printed list into ethercat_bridge.yaml `counts_offset:`.
"""
import os
import sys

import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory

from wirehawk_spool.spool_model import lengths_to_counts, load_spec
from wirehawk_control.kinematics import CDPRKinematics


def main():
    if len(sys.argv) != 6:
        print(__doc__)
        sys.exit(2)
    world = sys.argv[1]
    raw = [int(x) for x in sys.argv[2:6]]

    share = get_package_share_directory('wirehawk_control')
    params = yaml.safe_load(
        open(os.path.join(share, 'config', f'cdpr_params_{world}.yaml')))['cdpr_node']['ros__parameters']
    anchors = np.array(params['anchors'], dtype=float).reshape(-1, 3)
    home = np.array(params['start_position'], dtype=float)

    spec = load_spec(os.path.join(
        get_package_share_directory('wirehawk_spool'), 'config', 'spool.yaml'))

    kin = CDPRKinematics(anchors)
    L_home = kin.compute_commanded_lengths(home)
    counts_home = lengths_to_counts(L_home, spec)

    offset = [int(r) - int(c) for r, c in zip(raw, counts_home)]
    print(f"world          = {world}")
    print(f"home           = {home.tolist()}")
    print(f"L_home         = {[round(float(x), 3) for x in L_home]}")
    print(f"counts(IK(home))= {[int(c) for c in counts_home]}")
    print(f"raw            = {raw}")
    print(f"counts_offset  = {offset}")


if __name__ == '__main__':
    main()
