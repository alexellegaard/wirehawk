#!/usr/bin/env python3
"""Field commissioning: write the anchor layout into the world YAML + SDF.

Given the anchor (x, y) positions (from mast_position.py), the pulley height and
an optional TCP-home measurement, update `cdpr_params_field.yaml` (anchors,
workspace box, start_position) and the Gazebo world (pillar poses + payload home).

This script owns ONLY those geometry fields. Every other control param (mass,
tcp_attachment_length, speed/accel limits, rate, gain, …) is PRESERVED from the
existing yaml by targeted in-place edits — it is NOT regenerated, so re-running
this after a mast move never silently reverts a hand-edited value.

Usage:
  1. Fill ANCHORS_XY from mast_position.py (SLAVE order a0=mast1 … a3=mast4).
  2. Optionally fill DISTS_3D or DISTS_2D (+ TCP_HEIGHT) for the home; leave both
     None to keep the existing start_position.
  3. python3 commission_field.py

Requires numpy (the ROS2 control env has it).
"""
import os
import re
import numpy as np

# ---- CONFIG (edit these) ---------------------------------------------------
WORLD = "field"                  # output: cdpr_params_<WORLD>.yaml
PULLEY_Z = 3.0                   # anchor/top-pulley height [m] (measure it; don't assume mast height)
ANCHORS_XY = [                   # from mast_position.py, SLAVE (daisy-chain) order a0..a3:
                                 # mast1 -> mast2 -> mast3 -> mast4 (clockwise). +x = right (field view).
    (7.3222,  -11.1349),         # a0 = mast 1
    (-9.2198, -11.1349),         # a1 = mast 2
    (-9.0137,  8.7221),          # a2 = mast 3
    (10.9113,  13.5478),         # a3 = mast 4
]

# --- TCP home: fill ONE, or neither (then the existing start_position is kept) ---
DISTS_3D = None                  # e.g. [25.7, 24.9, 25.3, 25.1]  (laser, pulley->hawk)
DISTS_2D = None                  # e.g. [24.9, 25.2, 24.8, 25.4]  (base -> ground proj.)
TCP_HEIGHT = 1.2                 # hawk height above ground [m] (2D path only)

# --- workspace ---
MARGIN = 1.0                     # inset from the anchor bounding box [m]
Z_MIN, Z_MAX = 0.3, 2.7          # workspace z-band [m]
# ----------------------------------------------------------------------------


def trilaterate_3d(anchors, dists):
    p = np.zeros(3)
    for _ in range(200):
        r = np.array([np.linalg.norm(p - a) for a in anchors])
        f = r - np.asarray(dists, dtype=float)
        J = np.array([(p - a) / ri for a, ri in zip(anchors, r)])
        H = J.T @ J + 1e-3 * np.eye(3)
        dp = -np.linalg.solve(H, J.T @ f)
        p = p + dp
        if np.linalg.norm(dp) < 1e-6:
            break
    return p


def trilaterate_2d(anchors_xy, dists):
    p2 = np.zeros(2)
    for _ in range(200):
        r = np.array([np.linalg.norm(p2 - a) for a in anchors_xy])
        f = r - np.asarray(dists, dtype=float)
        J = np.array([(p2 - a) / ri for a, ri in zip(anchors_xy, r)])
        H = J.T @ J + 1e-3 * np.eye(2)
        dp = np.linalg.solve(H, J.T @ f)
        p2 = p2 - dp
        if np.linalg.norm(dp) < 1e-6:
            break
    return p2


def _replace_list(text, key, values):
    """Replace every `key: [...]` line in the yaml text with the given values."""
    pat = re.compile(rf'^(\s*{re.escape(key)}:\s*)\[[^\]]*\]', re.M)
    vals = ", ".join(f"{v:.4f}" for v in values)
    return pat.subn(lambda m: f"{m.group(1)}[{vals}]", text)


def main():
    anchors_xy = np.array(ANCHORS_XY, dtype=float)
    anchors = np.column_stack([anchors_xy, np.full(4, PULLEY_Z)])

    new_start = None
    src = "existing start_position (no TCP distances supplied)"
    if DISTS_3D is not None:
        new_start = trilaterate_3d(anchors, DISTS_3D)
        src = "3D distances (pulley -> hawk)"
    elif DISTS_2D is not None:
        p2 = trilaterate_2d(anchors_xy, DISTS_2D)
        new_start = np.array([p2[0], p2[1], TCP_HEIGHT])
        src = "2D ground distances + measured height"

    ws_min = np.array([anchors_xy[:, 0].min() + MARGIN,
                       anchors_xy[:, 1].min() + MARGIN, Z_MIN])
    ws_max = np.array([anchors_xy[:, 0].max() - MARGIN,
                       anchors_xy[:, 1].max() - MARGIN, Z_MAX])

    out_path = os.path.expanduser(
        f"~/wirehawk/src/wirehawk_control/config/cdpr_params_{WORLD}.yaml")
    with open(out_path) as f:
        yaml_txt = f.read()

    # In-place edits: only touch the geometry fields this script owns.
    flat = [c for xyz in anchors for c in xyz]
    yaml_txt, na = _replace_list(yaml_txt, "anchors", flat)
    yaml_txt, nw1 = _replace_list(yaml_txt, "workspace_min", ws_min)
    yaml_txt, nw2 = _replace_list(yaml_txt, "workspace_max", ws_max)
    ns = 0
    if new_start is not None:
        yaml_txt, ns = _replace_list(yaml_txt, "start_position", new_start)

    with open(out_path, "w") as f:
        f.write(yaml_txt)

    print(f"home source  = {src}")
    if new_start is not None:
        print(f"home         = ({new_start[0]:.3f}, {new_start[1]:.3f}, {new_start[2]:.3f}) m")
    print(f"workspace    = min {ws_min.tolist()}  max {ws_max.tolist()}")
    print(f"written      = {out_path}")
    print(f"  anchors x{na}, workspace_min x{nw1}, workspace_max x{nw2}, "
          f"start_position x{ns} (everything else preserved)")

    # --- also update the Gazebo world (pillar poses + payload home) so sim == field ---
    sdf_path = os.path.expanduser(
        "~/wirehawk/src/wirehawk_gazebo/worlds/wirehawk_world_field.sdf")
    if os.path.exists(sdf_path):
        txt = open(sdf_path).read()
        for i, (x, y) in enumerate(ANCHORS_XY, start=1):
            txt = re.sub(rf'(<model name="pillar_{i}">\s*<static>true</static>\s*<pose>)[^<]+(</pose>)',
                         rf'\g<1>{x:.4f} {y:.4f} 1.5 0 0 0\g<2>', txt, count=1, flags=re.S)
        if new_start is not None:
            txt = re.sub(r'(<model name="payload">\s*<pose>)[^<]+(</pose>)',
                         rf'\g<1>{new_start[0]:.4f} {new_start[1]:.4f} {new_start[2]:.4f} 0 0 0\g<2>',
                         txt, count=1, flags=re.S)
        open(sdf_path, "w").write(txt)
        print(f"sdf updated  = {sdf_path}")


if __name__ == "__main__":
    main()
