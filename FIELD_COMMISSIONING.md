# Wirehawk — Field Guide

Two processes. Use **B** day-to-day; use **A** when you (re)measure the system.

| | When | What it does |
|---|---|---|
| **A. Calibration** | first setup, masts moved, or want a fresh home | re-measure geometry + TCP home, write config |
| **B. Normal start** | already calibrated | power on → bridge → controller → drive |

Machines: **Pi** = field EtherCAT master (pull-only). **Laptop** = your XPS (runs the controller). Both share one ROS2 network over DDS.

---

## Process A — Calibration

### A1. Measure the masts (laptop)

Shoot all six pairwise mast distances, ideally twice each (once from each end).
The **diagonal `d02`** is the one that was wrong before — it dominates the anchor error (~0.3 m).

Edit `~/mast_position.py` → `MEASUREMENTS` (six values), then:

```bash
python3 ~/mast_position.py
```

- [ ] Prints `Anchor positions (m)` for a0..a3.
- [ ] Residuals ≤ ~0.05 m. A residual ~0.3 m = a measurement is still wrong.

### A2. Write anchors + config (laptop)

Edit `~/wirehawk/src/wirehawk_control/scripts/commission_field.py`:

- `ANCHORS_XY` ← the 4 anchors from A1, in **SLAVE order** — a0=mast1, a1=mast4, a2=mast3, a3=mast2 (the CW daisy-chain). If the masts haven't moved, leave as-is.
- `DISTS_3D = None` and `DISTS_2D = None` (the rope method measures the home, not this file).

```bash
python3 ~/wirehawk/src/wirehawk_control/scripts/commission_field.py
```

- [ ] Writes `cdpr_params_field.yaml` (anchors + placeholder home) and updates the SDF.

### A3. Commit + deploy to the Pi

Laptop:

```bash
cd ~/wirehawk && git add -A && git commit -m "field calibration: anchors" && git push
```

Pi (pull-only):

```bash
cd ~/wirehawk && git fetch origin && git reset --hard origin/control-bridge-integration && git clean -fdx src/ethercat && make -C src/ethercat && source /opt/ros/jazzy/setup.bash && colcon build --packages-select wirehawk_msgs wirehawk_ethercat_bridge
```

### A4. Motor direction check — FIRST SETUP ONLY (skip if already known)

Ropes **disconnected from the TCP**. This needs the bridge running and `cdpr_node` down.

Pi — start the bridge:

```bash
sudo -E bash -c 'source /opt/ros/jazzy/setup.bash && source ~/wirehawk/install/setup.bash && ros2 run wirehawk_ethercat_bridge ethercat_bridge_node --ros-args -r state/motors:=real/state/motors --params-file ~/wirehawk/src/wirehawk_ethercat_bridge/config/ethercat_bridge.yaml'
```

Laptop — run the check:

```bash
cd ~/wirehawk && source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 run wirehawk_control motor_direction_check --ros-args -r state/motors:=real/state/motors
```

- [ ] Each motor **winds IN** on `+delta`. If one winds OUT, flip its `direction` sign in `ethercat_bridge.yaml`, commit+push, and Pi re-pull+rebuild (A3).
- [ ] **2-vs-2 split (no single odd one out):** the jog test only gives *relative* direction — it can't say which pair is correct. Guess one pair as `+1`, then let the A8 FK check arbitrate: a wrong direction lands the FK **meters** off home. If so, flip *both* motors of that pair and re-launch — no re-calibration needed.
- [ ] Current known-good: `direction: [-1, -1, -1, -1]`.

**Stop the bridge** before A5 (F30 jog and the bridge's CSP mode conflict).

### A5. Rope home — measure the TCP with the rope (Pi)

The ropes are free (not attached to the TCP). For each mast: retract so the rope end sits at the pulley, press its number (1–4, any order); jog it out (F30) and attach to the TCP. When all four are attached and balanced, press ENTER.

```bash
python3 ~/wirehawk/src/wirehawk_control/scripts/commission_rope.py field
```

- [ ] Prints the four free spans and the home `(x, y, z)`, writes `start_position` into the YAML.

### A6. Encoder offset (Pi)

Immediately after A5, with the TCP still at the home (**do not move it**):

```bash
python3 ~/wirehawk/src/wirehawk_control/scripts/calibrate.py field
```

- [ ] Prints + writes `counts_offset` into `ethercat_bridge.yaml`.

Battery sanity (optional): power-cycle one drive and re-read to confirm the multi-turn survived:

```bash
cd ~/wirehawk/src/ethercat && sudo ./a6_encoder eth0
```

### A7. Transfer the calibration + build

The Pi **cannot push** to GitHub (auth failed). Pull the two generated YAMLs to the laptop and commit there instead:

Laptop:

```bash
scp vision@vision.local:~/wirehawk/src/wirehawk_control/config/cdpr_params_field.yaml ~/wirehawk/src/wirehawk_control/config/
scp vision@vision.local:~/wirehawk/src/wirehawk_ethercat_bridge/config/ethercat_bridge.yaml ~/wirehawk/src/wirehawk_ethercat_bridge/config/
cd ~/wirehawk && git add -A && git commit -m "field calibration: home + offset" && git push
source /opt/ros/jazzy/setup.bash && colcon build --packages-select wirehawk_control wirehawk_gazebo
```

- [ ] `start_position` and `counts_offset` now live in the repo (committed + pushed from the laptop).

*(Long-term: put an SSH key on the Pi so it can push directly and the scp step goes away.)*

### A8. Launch + verify

Pi — bridge (same command as A4). Laptop:

```bash
cd ~/wirehawk && source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch wirehawk_control cdpr.launch.py world:=field backend:=real
```

Checks:

1. **FK** — `ros2 topic echo /cdpr/current_pose`
   - [ ] Reads ≈ the measured home (±0.3 m fine). Meters off → anchor order or a direction pair is wrong.
2. **Direction (definitive)** — slow **+z (up)**
   - [ ] All four cables **shorten**. Any slack → flip that `direction`, re-commit, re-pull, re-launch.
3. **Handedness** — nudge **+x**
   - [ ] Hawk moves the physical direction you expect.

---

## Process B — Normal start (already calibrated)

### B1. Power on

- [ ] Drives + 24 V PSU on (releases the holding brakes).

### B2. Bridge (Pi)

```bash
sudo -E bash -c 'source /opt/ros/jazzy/setup.bash && source ~/wirehawk/install/setup.bash && ros2 run wirehawk_ethercat_bridge ethercat_bridge_node --ros-args -r state/motors:=real/state/motors --params-file ~/wirehawk/src/wirehawk_ethercat_bridge/config/ethercat_bridge.yaml'
```

### B3. Controller (laptop)

```bash
cd ~/wirehawk && source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch wirehawk_control cdpr.launch.py world:=field backend:=real
```

### B4. Drive (laptop)

```bash
ros2 run wirehawk_control keyboard_teleop --ros-args --params-file ~/wirehawk/install/wirehawk_control/share/wirehawk_control/config/cdpr_params_field.yaml
```

- WASD, slow and gentle. Stay well inside the hull.
- Watch `/cdpr/diagnostic`: FK-convergence flag + four tensions (tension near zero = slack cable = near the edge).
- `trajectory_planner` and `keyboard_teleop` must **not** run at the same time — the planner holds its pose and cancels teleop.

---

## Safety (read before touching the hardware)

1. **Ropes disconnected from the TCP** for every first-enable / direction test.
2. **No abrupt motions** — gentle ramped commands only. A hard step command can destroy a spool.
3. **E-stop** cuts power to everything (motion stops + brakes engage). It is the hard stop.
4. **Known gap:** the brakes are currently released by 24 V straight from the PSU (not via the drive), so a *drive* fault does not engage the brake — the bridge's following-error watchdog disables torque but not the brake. Do not couple the TCP until the brake relay is re-wired to the drive's fault output.
5. **Pi can't push** — anything `calibrate.py` / `commission_rope.py` writes on the Pi (YAML) must be pulled to the laptop (A7 `scp`) and committed there, or the next `git reset --hard` wipes it.
6. **Never move the hawk** between A5 (rope home) and A6 (offset), or the offset is wrong.
7. **Schema version handshake** — the bridge publishes `/wirehawk/schema_version`; the controller refuses to command motion on a mismatch. If you see "schema mismatch" in the controller log, the Pi's `wirehawk_msgs` is stale — rebuild it (A3). A stale schema makes every feedback message deserialize to garbage, so the controller runs blind and the drives can move unwatched (this broke a spool).

---

## Files touched

| File | Machine | What edits it |
|---|---|---|
| `~/mast_position.py` | laptop | `MEASUREMENTS` (6 distances) — you |
| `src/wirehawk_control/scripts/commission_field.py` | laptop | `ANCHORS_XY`, `DISTS_*` — you |
| `src/wirehawk_control/config/cdpr_params_field.yaml` | laptop→Pi | generated (`commission_field.py`, `commission_rope.py`) |
| `src/wirehawk_ethercat_bridge/config/ethercat_bridge.yaml` | laptop→Pi | `direction` (you), `counts_offset` (`calibrate.py`) |
| `src/wirehawk_gazebo/worlds/wirehawk_world_field.sdf` | laptop | generated (`commission_field.py`) |

### Quick command reference

| Do this | Run this (machine) |
|---|---|
| Estimate mast anchors | `python3 ~/mast_position.py` (laptop) |
| Write anchors + config | `python3 ~/wirehawk/src/wirehawk_control/scripts/commission_field.py` (laptop) |
| Measure TCP home (rope) | `python3 ~/wirehawk/src/wirehawk_control/scripts/commission_rope.py field` (Pi) |
| Encoder offset | `python3 ~/wirehawk/src/wirehawk_control/scripts/calibrate.py field` (Pi) |
| Read raw encoders | `cd ~/wirehawk/src/ethercat && sudo ./a6_encoder eth0` (Pi) |
| Direction check | `ros2 run wirehawk_control motor_direction_check --ros-args -r state/motors:=real/state/motors` (laptop) |
| Start bridge | `sudo -E bash -c 'source /opt/ros/jazzy/setup.bash && source ~/wirehawk/install/setup.bash && ros2 run wirehawk_ethercat_bridge ethercat_bridge_node --ros-args -r state/motors:=real/state/motors --params-file ~/wirehawk/src/wirehawk_ethercat_bridge/config/ethercat_bridge.yaml'` (Pi) |
| Start controller | `ros2 launch wirehawk_control cdpr.launch.py world:=field backend:=real` (laptop) |
| Teleop (WASD) | `ros2 run wirehawk_control keyboard_teleop --ros-args --params-file ~/wirehawk/install/wirehawk_control/share/wirehawk_control/config/cdpr_params_field.yaml` (laptop) |
| Pi pull (deploy) | `cd ~/wirehawk && git fetch origin && git reset --hard origin/control-bridge-integration && git clean -fdx src/ethercat && make -C src/ethercat && source /opt/ros/jazzy/setup.bash && colcon build --packages-select wirehawk_msgs wirehawk_ethercat_bridge` (Pi) |
| Laptop build | `cd ~/wirehawk && source /opt/ros/jazzy/setup.bash && colcon build --packages-select wirehawk_control wirehawk_gazebo` (laptop) |
