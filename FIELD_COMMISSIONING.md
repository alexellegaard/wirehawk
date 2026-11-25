# Wirehawk — Field Commissioning (from ground up)

Use this when starting fresh or after the physical setup changes (spool replaced, drive re-homed, masts moved, rope changed). For the day-to-day calibrated run, use **`FIELD_OPERATION.md`** instead — no pulling, no building, no re-measuring.

## Machines

- **Pi** — field EtherCAT master, **pull-only** (never edit here). Runs `ethercat_bridge_node`.
- **Laptop** — the controller, the sim, and all commissioning tools.

Both share one ROS2 network over DDS. The bridge converts raw encoder → **free-span cable length** (anchor/pulley → TCP, in metres) using three constants it owns — `zero_ticks`, `mast_run`, `direction` — so the controller never sees a raw encoder count, and it finds the start position itself via FK.

---

## 1. Get the code + build

Laptop:

```bash
cd ~/wirehawk && git checkout control-bridge-integration && git pull
source /opt/ros/jazzy/setup.bash
colcon build
```

Pi (pull-only):

```bash
cd ~/wirehawk && git fetch origin && git reset --hard origin/control-bridge-integration && git clean -fdx src/ethercat
make -C src/ethercat
source /opt/ros/jazzy/setup.bash
colcon build --packages-select wirehawk_msgs wirehawk_ethercat_bridge
```

If a machine doesn't have the repo yet, `git clone <url> ~/wirehawk` first, then the same build. **Build `wirehawk_msgs` and the bridge together** — a stale msgs build makes the controller drop every feedback message and run blind.

## 2. Measure the masts (laptop)

Shoot all six pairwise mast distances. Edit `src/wirehawk_control/scripts/mast_position.py` → `MEASUREMENTS` (six values, labeled by mast pair), then:

```bash
python3 src/wirehawk_control/scripts/mast_position.py
```

- [ ] Prints mast 1–4 positions in the field frame (`+x` = right standing behind mast 1, `+y` = up).
- [ ] Residuals ≤ ~0.05 m. A residual ~0.3 m = a measurement is still wrong.

## 3. Write anchors + config (laptop)

Edit `src/wirehawk_control/scripts/commission_field.py` → `ANCHORS_XY` (the 4 anchors from step 2, in **SLAVE order** — a0=mast1, a1=mast2, a2=mast3, a3=mast4, the clockwise daisy-chain). If the masts haven't moved, leave as-is.

```bash
python3 src/wirehawk_control/scripts/commission_field.py
```

- [ ] Writes `cdpr_params_field.yaml` (anchors) and updates the SDF.

## 4. Motor direction check — first setup only (skip if the yaml already has a known-good `direction`)

Ropes **disconnected from the TCP**. Bridge running, `cdpr_node` down.

Pi — start the bridge:

```bash
sudo -E bash -c 'source /opt/ros/jazzy/setup.bash && source ~/wirehawk/install/setup.bash && ros2 run wirehawk_ethercat_bridge ethercat_bridge_node --ros-args -r state/motors:=real/state/motors --params-file ~/wirehawk/src/wirehawk_ethercat_bridge/config/ethercat_bridge.yaml'
```

Laptop — run the check:

```bash
cd ~/wirehawk && source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 run wirehawk_control motor_direction_check --ros-args -r state/motors:=real/state/motors
```

- [ ] Each motor **winds IN** on `+0.5 m`. If one winds OUT, flip its `direction` sign in `ethercat_bridge.yaml`.

**Stop the bridge** before step 5 (the F30 jog and the bridge's CSP mode conflict).

## 5. Log the zero ticks (Pi) — one-time per physical change

Retract each rope so its end sits exactly at the pulley, then:

```bash
python3 ~/wirehawk/src/wirehawk_control/scripts/log_zeros.py
```

- [ ] Prints + writes `zero_ticks` into `ethercat_bridge.yaml` (slave order).
- [ ] Zero ticks only change when a spool is replaced or a drive re-homed (battery-backed multi-turn encoder survives power cycles). **No re-zeroing per session.**

## 6. Commit + deploy

Laptop:

```bash
cd ~/wirehawk && git add -A && git commit -m "field calibration: <describe what changed>" && git push
```

Pi (pull-only):

```bash
cd ~/wirehawk && git fetch origin && git reset --hard origin/control-bridge-integration && git clean -fdx src/ethercat && make -C src/ethercat && source /opt/ros/jazzy/setup.bash && colcon build --packages-select wirehawk_msgs wirehawk_ethercat_bridge
```

## 7. First launch + verify

Pi — bridge (same command as step 4). Laptop:

```bash
cd ~/wirehawk && source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch wirehawk_control cdpr.launch.py world:=field backend:=real
```

Checks:

1. **FK** — `ros2 topic echo /cdpr/current_pose`
   - [ ] Plausible in-field position. Metres off → anchor order or a direction pair is wrong.
2. **Direction (definitive)** — slow **+z (up)**
   - [ ] All four cables **shorten**. Any slack → flip that `direction`, re-commit, re-pull, re-launch.
3. **Handedness** — nudge **+x**
   - [ ] The hawk moves the expected physical direction.

---

## Safety (read before touching hardware)

1. **Ropes disconnected from the TCP** for every first-enable / direction test.
2. **No abrupt motions** — gentle ramped commands only. A hard step can destroy a spool.
3. **E-stop** cuts power (motion stops + brakes engage) — the hard stop.
4. **Known gap:** the brakes are released by 24 V straight from the PSU (not via the drive), so a *drive* fault does **not** engage the brake — the bridge's following-error watchdog disables torque but not the brake. Do not couple the TCP until the brake relay is re-wired to the drive's fault output.
5. **Zero ticks are constants** — log once (step 5) and commit. If the bridge publishes a wrong free span (negative, or the FK lands metres off), re-check `zero_ticks` / `mast_run` / `direction` in `ethercat_bridge.yaml`.
6. **Schema handshake** — the bridge publishes `/wirehawk/schema_version`; the controller refuses to command motion on a mismatch ("schema mismatch" = the Pi's `wirehawk_msgs` is stale — rebuild it, step 1). A stale schema blinds the controller while the drives keep moving unwatched.

## Files touched

| File | Machine | What edits it |
|---|---|---|
| `src/wirehawk_control/scripts/mast_position.py` | laptop | `MEASUREMENTS` (6 distances) — you |
| `src/wirehawk_control/scripts/commission_field.py` | laptop | `ANCHORS_XY` — you |
| `src/wirehawk_control/config/cdpr_params_field.yaml` | laptop | `anchors`, `workspace_*`, `payload_mass`, `tcp_attachment_length` — you / `commission_field.py` |
| `src/wirehawk_ethercat_bridge/config/ethercat_bridge.yaml` | laptop → Pi | `zero_ticks` (`log_zeros.py`), `mast_run`, `direction` — you |
| `src/wirehawk_gazebo/worlds/wirehawk_world_field.sdf` | laptop | generated (`commission_field.py`) + launch-templated |
