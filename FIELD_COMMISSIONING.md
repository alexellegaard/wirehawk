# Wirehawk — Field Guide

Two processes. Use **B** day-to-day; use **A** when the physical setup changes.

| | When | What it does |
|---|---|---|
| **A. Calibration** | spool replaced, drive re-homed, or masts moved | re-measure geometry + log the zero ticks |
| **B. Normal start** | already calibrated | power on → bridge → controller → drive |

Machines: **Pi** = field EtherCAT master (pull-only). **Laptop** = your XPS (runs the controller). Both share one ROS2 network over DDS.

**How the numbers flow now:** the bridge converts raw encoder → **free-span cable length** (anchor/pulley → TCP, in metres) itself, using three constants it owns — `zero_ticks`, `mast_run`, `direction`. It publishes free-span metres; the controller never sees a raw encoder count. The controller's `taut_subset_fk` trilaterates the four free spans into the start position on its own, so there is **no home measurement and no `counts_offset`** anymore.

---

## Process A — Calibration

### A1. Measure the masts (laptop)

Shoot all six pairwise mast distances, ideally twice each (once from each end). The **diagonal `d02`** is the one that was wrong before — it dominates the anchor error (~0.3 m).

Edit `~/mast_position.py` → `MEASUREMENTS` (six values), then:

```bash
python3 ~/mast_position.py
```

- [ ] Prints `Anchor positions (m)` for a0..a3.
- [ ] Residuals ≤ ~0.05 m. A residual ~0.3 m = a measurement is still wrong.

### A2. Write anchors + config (laptop)

Edit `~/wirehawk/src/wirehawk_control/scripts/commission_field.py`:

- `ANCHORS_XY` ← the 4 anchors from A1, in **SLAVE order** — a0=mast1, a1=mast2, a2=mast3, a3=mast4 (the CW daisy-chain). If the masts haven't moved, leave as-is.

```bash
python3 ~/wirehawk/src/wirehawk_control/scripts/commission_field.py
```

- [ ] Writes `cdpr_params_field.yaml` (anchors) and updates the SDF.

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

- [ ] Each motor **winds IN** on `+0.5 m`. If one winds OUT, flip its `direction` sign in `ethercat_bridge.yaml`, commit+push, and Pi re-pull+rebuild (A3).
- [ ] Current known-good: `direction: [-1, -1, -1, -1]`.

**Stop the bridge** before A5 (F30 jog and the bridge's CSP mode conflict).

### A5. Log the zero ticks (Pi) — ONE-TIME per physical change

Retract each rope so its end sits exactly at the pulley, then run:

```bash
python3 ~/wirehawk/src/wirehawk_control/scripts/log_zeros.py
```

- [ ] Prints + writes `zero_ticks` into `ethercat_bridge.yaml` (slave order).
- [ ] The zero ticks only change when a spool is replaced or a drive re-homed (battery-backed multi-turn encoder survives power cycles). **No re-zeroing per session.**

### A6. Commit + push (Pi, one-time)

```bash
cd ~/wirehawk && git add -A && git commit -m "field calibration: zero ticks" && git push origin control-bridge-integration
```

Laptop pull + build:

```bash
cd ~/wirehawk && git pull && source /opt/ros/jazzy/setup.bash && colcon build --packages-select wirehawk_control wirehawk_gazebo_bridge
```

### A7. Launch + verify

Pi — bridge (same command as A4). Laptop:

```bash
cd ~/wirehawk && source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch wirehawk_control cdpr.launch.py world:=field backend:=real
```

Checks:

1. **FK** — `ros2 topic echo /cdpr/current_pose`
   - [ ] Reads a plausible in-field position (the taut-subset FK finds it from the free spans). Meters off → anchor order or a direction pair is wrong.
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
- `trajectory_planner` and `keyboard_teleop` must **not** run at the same time.

---

## Safety (read before touching the hardware)

1. **Ropes disconnected from the TCP** for every first-enable / direction test.
2. **No abrupt motions** — gentle ramped commands only. A hard step command can destroy a spool.
3. **E-stop** cuts power to everything (motion stops + brakes engage). It is the hard stop.
4. **Known gap:** the brakes are currently released by 24 V straight from the PSU (not via the drive), so a *drive* fault does not engage the brake — the bridge's following-error watchdog disables torque but not the brake. Do not couple the TCP until the brake relay is re-wired to the drive's fault output.
5. **Zero ticks are constants** — log them once (A5) and commit. If the free span the bridge publishes is wrong (e.g. negative, or the FK lands metres off), re-check `zero_ticks` / `mast_run` / `direction` in `ethercat_bridge.yaml`.
6. **Schema version handshake** — the bridge publishes `/wirehawk/schema_version`; the controller refuses to command motion on a mismatch. If you see "schema mismatch", the Pi's `wirehawk_msgs` is stale — rebuild it (A3). A stale schema makes every feedback message deserialize to garbage, so the controller runs blind and the drives can move unwatched (this broke a spool).

---

## Files touched

| File | Machine | What edits it |
|---|---|---|
| `~/mast_position.py` | laptop | `MEASUREMENTS` (6 distances) — you |
| `src/wirehawk_control/scripts/commission_field.py` | laptop | `ANCHORS_XY` — you |
| `src/wirehawk_control/config/cdpr_params_field.yaml` | laptop | `anchors`, `workspace_*`, `payload_mass` — you / `commission_field.py` |
| `src/wirehawk_ethercat_bridge/config/ethercat_bridge.yaml` | laptop→Pi | `zero_ticks` (`log_zeros.py`), `mast_run`, `direction` (you) |
| `src/wirehawk_gazebo/worlds/wirehawk_world_field.sdf` | laptop | generated (`commission_field.py`) |

### Quick command reference

| Do this | Run this (machine) |
|---|---|
| Estimate mast anchors | `python3 ~/mast_position.py` (laptop) |
| Write anchors + config | `python3 ~/wirehawk/src/wirehawk_control/scripts/commission_field.py` (laptop) |
| Log zero ticks (once) | `python3 ~/wirehawk/src/wirehawk_control/scripts/log_zeros.py` (Pi) |
| Read raw encoders | `cd ~/wirehawk/src/ethercat && sudo ./a6_encoder eth0` (Pi) |
| Direction check | `ros2 run wirehawk_control motor_direction_check --ros-args -r state/motors:=real/state/motors` (laptop) |
| Start bridge | `sudo -E bash -c 'source /opt/ros/jazzy/setup.bash && source ~/wirehawk/install/setup.bash && ros2 run wirehawk_ethercat_bridge ethercat_bridge_node --ros-args -r state/motors:=real/state/motors --params-file ~/wirehawk/src/wirehawk_ethercat_bridge/config/ethercat_bridge.yaml'` (Pi) |
| Start controller | `ros2 launch wirehawk_control cdpr.launch.py world:=field backend:=real` (laptop) |
| Teleop (WASD) | `ros2 run wirehawk_control keyboard_teleop --ros-args --params-file ~/wirehawk/install/wirehawk_control/share/wirehawk_control/config/cdpr_params_field.yaml` (laptop) |
| Pi pull (deploy) | `cd ~/wirehawk && git fetch origin && git reset --hard origin/control-bridge-integration && git clean -fdx src/ethercat && make -C src/ethercat && source /opt/ros/jazzy/setup.bash && colcon build --packages-select wirehawk_msgs wirehawk_ethercat_bridge` (Pi) |
| Laptop build | `cd ~/wirehawk && source /opt/ros/jazzy/setup.bash && colcon build --packages-select wirehawk_control wirehawk_gazebo_bridge` (laptop) |
