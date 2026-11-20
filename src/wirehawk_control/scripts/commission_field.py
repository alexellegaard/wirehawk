#!/usr/bin/env python3
"""Field commissioning: trilaterate the TCP home + write the world YAML.

Given the anchor (x, y) from mast_position.py, the pulley height, and the
measured TCP distances, compute the hawk's home (x, y, z) and emit
cdpr_params_<world>.yaml (anchors + workspace box + start_position).

Two measurement paths (fill exactly one):
  3D:  DISTS_3D = [d0, d1, d2, d3]   laser pulley_i -> hawk  [m]
  2D:  DISTS_2D = [d0, d1, d2, d3]   mast base_i -> hawk ground projection [m]
       TCP_HEIGHT = z of the hawk above the ground [m]

Run:  python3 commission_field.py
Requires numpy + pyyaml (the ROS2 control env has both).
"""
import math
import os
import re
import numpy as np

# ---- CONFIG (edit these) ---------------------------------------------------
WORLD = "field"                  # output: cdpr_params_<WORLD>.yaml
PULLEY_Z = 3.0                   # anchor/top-pulley height [m]
ANCHORS_XY = [                   # from mast_position.py, SLAVE (daisy-chain) order a0..a3:
                                 # mast1 -> mast2 -> mast3 -> mast4 (clockwise). +x = right (field view).
    (7.3222,  -11.1349),         # a0 = mast 1 = slave 1
    (-9.2198, -11.1349),         # a1 = mast 2 = slave 2
    (-9.0137,  8.7221),          # a2 = mast 3 = slave 3
    (10.9113,  13.5478),         # a3 = mast 4 = slave 4
]

# --- TCP measurement: fill ONE of these, or neither (rope method measures it) ---
DISTS_3D = None                  # e.g. [25.7, 24.9, 25.3, 25.1]  (laser, pulley->hawk)
DISTS_2D = None                  # e.g. [24.9, 25.2, 24.8, 25.4]  (base -> ground proj.)
TCP_HEIGHT = 1.2                 # hawk height above ground [m] (2D path only)
# With DISTS_3D = DISTS_2D = None the home is a centroid placeholder; the real
# home is then measured by commission_rope.py (rope-as-tape-measure).

# --- workspace ---
MARGIN = 1.0                     # inset from the anchor bounding box [m]
Z_MIN, Z_MAX = 0.3, 2.7         # workspace z-band [m]

# --- control limits (carried over from the 40x40 world) ---
MAX_LINEAR_SPEED, MAX_LINEAR_ACCEL, MAX_LINEAR_DECEL = 5.0, 3.0, 2.0
MAX_CABLE_SPEED, MAX_CABLE_ACCEL = 10485760.0, 6553600.0
EA, MASS, FK_GAIN = 98100.0, 3.0, 0.5
RATE_HZ, CMD_TIMEOUT = 50.0, 0.2
# ----------------------------------------------------------------------------


def trilaterate_3d(anchors, dists):
    """Least-squares (Gauss-Newton) solve for p minimizing sum(|p-a_i| - d_i)^2."""
    p = np.zeros(3)  # anchors are centroid-centred, so centroid == origin
    for _ in range(200):
        r = np.array([np.linalg.norm(p - a) for a in anchors])
        f = r - np.asarray(dists, dtype=float)
        J = np.array([(p - a) / ri for a, ri in zip(anchors, r)])
        g = J.T @ f
        H = J.T @ J + 1e-3 * np.eye(3)
        dp = -np.linalg.solve(H, g)
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
        dp = -np.linalg.solve(H, J.T @ f)
        p2 = p2 + dp
        if np.linalg.norm(dp) < 1e-6:
            break
    return p2


def main():
    anchors_xy = np.array(ANCHORS_XY, dtype=float)
    anchors = np.column_stack([anchors_xy, np.full(4, PULLEY_Z)])

    if DISTS_3D is not None:
        home = trilaterate_3d(anchors, DISTS_3D)
        src = "3D distances (pulley -> hawk)"
    elif DISTS_2D is not None:
        p2 = trilaterate_2d(anchors_xy, DISTS_2D)
        home = np.array([p2[0], p2[1], TCP_HEIGHT])
        src = f"2D ground distances + measured height {TCP_HEIGHT} m"
    else:
        home = np.array([0.0, 0.0, TCP_HEIGHT])
        src = "centroid placeholder (no TCP distances supplied)"

    ws_min = np.array([anchors_xy[:, 0].min() + MARGIN,
                       anchors_xy[:, 1].min() + MARGIN, Z_MIN])
    ws_max = np.array([anchors_xy[:, 0].max() - MARGIN,
                       anchors_xy[:, 1].max() - MARGIN, Z_MAX])

    flat_anchors = [f"{x:.4f}" for xyz in anchors for x in xyz]
    out_path = os.path.expanduser(
        f"~/wirehawk/src/wirehawk_control/config/cdpr_params_{WORLD}.yaml")
    os.makedirs(os.path.dirname(out_path), exist_ok=True)

    yaml_text = f"""# Wirehawk control params — world: {WORLD} (field, non-square quadrilateral)
# Anchors in SLAVE (daisy-chain) order: mast1 -> mast2 -> mast3 -> mast4 (clockwise).
cdpr_node:
  ros__parameters:
    anchors: [{', '.join(flat_anchors)}]
    start_position: [{home[0]:.4f}, {home[1]:.4f}, {home[2]:.4f}]
    workspace_min: [{ws_min[0]:.4f}, {ws_min[1]:.4f}, {ws_min[2]:.4f}]
    workspace_max: [{ws_max[0]:.4f}, {ws_max[1]:.4f}, {ws_max[2]:.4f}]
    max_linear_speed: {MAX_LINEAR_SPEED}
    max_linear_accel: {MAX_LINEAR_ACCEL}
    max_linear_decel: {MAX_LINEAR_DECEL}
    max_cable_speed: {MAX_CABLE_SPEED}
    max_cable_accel: {MAX_CABLE_ACCEL}
    rate_hz: {RATE_HZ}
    cmd_timeout: {CMD_TIMEOUT}
    cable_axial_stiffness: {EA}
    payload_mass: {MASS}
    fk_gain: {FK_GAIN}

trajectory_planner:
  ros__parameters:
    max_linear_speed: {MAX_LINEAR_SPEED}
    max_linear_accel: {MAX_LINEAR_ACCEL}
    goal_tolerance: 0.02
    decel_distance: 0.3
    rate_hz: {RATE_HZ}

keyboard_teleop:
  ros__parameters:
    max_linear_speed: {MAX_LINEAR_SPEED}
    max_linear_accel: 4.0
    rate_hz: 30.0

bird_sweeper:
  ros__parameters:
    workspace_min: [{ws_min[0]:.4f}, {ws_min[1]:.4f}, {ws_min[2]:.4f}]
    workspace_max: [{ws_max[0]:.4f}, {ws_max[1]:.4f}, {ws_max[2]:.4f}]
    max_decel: {MAX_LINEAR_DECEL}
    lateral_gain: 0.5
    scare_radius: 0.0
"""

    with open(out_path, "w") as f:
        f.write(yaml_text)

    # --- also update the Gazebo world (pillar poses + payload home) so sim == field ---
    sdf_path = os.path.expanduser(
        "~/wirehawk/src/wirehawk_gazebo/worlds/wirehawk_world_field.sdf")
    if os.path.exists(sdf_path):
        txt = open(sdf_path).read()
        for i, (x, y) in enumerate(ANCHORS_XY, start=1):
            txt = re.sub(rf'(<model name="pillar_{i}">\s*<static>true</static>\s*<pose>)[^<]+(</pose>)',
                         rf'\g<1>{x:.4f} {y:.4f} 1.5 0 0 0\g<2>', txt, count=1, flags=re.S)
        txt = re.sub(r'(<model name="payload">\s*<pose>)[^<]+(</pose>)',
                     rf'\g<1>{home[0]:.4f} {home[1]:.4f} {home[2]:.4f} 0 0 0\g<2>', txt, count=1, flags=re.S)
        open(sdf_path, "w").write(txt)
        print(f"sdf updated  = {sdf_path}")

    print(f"home source = {src}")
    print(f"home        = ({home[0]:.3f}, {home[1]:.3f}, {home[2]:.3f}) m")
    print(f"workspace   = min {ws_min.tolist()}  max {ws_max.tolist()}")
    print(f"written     = {out_path}")


if __name__ == "__main__":
    main()
