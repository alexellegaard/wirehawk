# Wirehawk — Field Commissioning Runbook

Ordered, copy-paste checklist to get the real system running on site, first try.
Ticked boxes are the "look-for" / acceptance for each step.

---

## Phase 1 — Re-measure the geometry

1. Re-shoot **all six mast distances** (ideally twice each, once from each end).
   - **4↔2 is the one that was wrong** — nail it. It dominates the anchor error (~0.3 m).
2. *(Optional)* Laser-level each mast's **pulley height** for per-mast z. Skip → keep `z = 3.0`.

## Phase 2 — Compute the anchors

Edit the six values, then run:

```bash
python3 ~/mast_position.py
```

- [ ] Copy the 4 anchor `(x, y)` (a0 = mast1 … a3 = mast4).
- [ ] Residuals ≤ ~0.05 m (if a residual is ~0.3 m, a measurement is still wrong).

## Phase 3 — Home + config (YAML + SDF)

1. **Measure where the hawk hangs right now** — laser the 4 pulley→hawk distances.
2. Edit `~/wirehawk/src/wirehawk_control/scripts/commission_field.py`:
   - `ANCHORS_XY` ← the 4 anchor `(x, y)` from Phase 2
   - `DISTS_3D` ← the 4 pulley→hawk distances
   - `PULLEY_Z` ← 3.0 (or your measured pulley height)
   - `MARGIN` ← 1.0 (workspace inset)
3. Run it:

```bash
python3 ~/wirehawk/src/wirehawk_control/scripts/commission_field.py
```

- [ ] Prints `home = (x, y, z)` and writes `cdpr_params_field.yaml` + updates `wirehawk_world_field.sdf`.
- [ ] **Do not move the hawk** between this measurement and Phase 6 calibration.

## Phase 4 — Motor direction

1. Power on all four drives.
2. Jog each motor with the **same arrow** (pick UP), note whether the cable winds IN or OUT.
   - No universal "UP = wind in" — it depends on C00.01 + drum side. All four should agree.
3. Edit `~/wirehawk/src/wirehawk_ethercat_bridge/config/ethercat_bridge.yaml`:

```yaml
direction: [1, 1, 1, 1]   # set -1 for any motor whose cable winds OUT when the others wind IN
```

- [ ] The odd motors out are flagged `-1`. (Definitive confirmation comes in Phase 7.)
- [ ] **2-vs-2 split (no odd one out):** the jog test only gives *relative* direction — it can't say which pair is correct. Guess one pair as `+1`, then let the Phase-7 FK check arbitrate: a wrong direction inverts a cable length and the FK lands **meters** off home (not ±0.3 m). If so, flip *both* motors of that pair and re-launch — no re-calibration needed.

## Phase 5 — Commit + build

```bash
cd ~/wirehawk
git add -A && git commit -m "field commissioning" && git push
```

**Pi** (pull-only):

```bash
cd ~/wirehawk && git fetch origin && git reset --hard origin/control-bridge-integration && git clean -fdx src/ethercat && make -C src/ethercat
source /opt/ros/jazzy/setup.bash && colcon build --packages-select wirehawk_ethercat_bridge
```

**Laptop**:

```bash
cd ~/wirehawk && source /opt/ros/jazzy/setup.bash && colcon build --packages-select wirehawk_control wirehawk_gazebo
```

## Phase 6 — Calibrate (Pi)

1. Confirm the hawk hasn't moved since Phase 3.
2. Run:

```bash
cd ~/wirehawk && python3 src/wirehawk_control/scripts/calibrate.py field
```

- [ ] Prints `counts_offset = [...]` and writes it into `ethercat_bridge.yaml`.
3. **Battery sanity:** power-cycle one drive, re-read, confirm its position is unchanged:

```bash
cd ~/wirehawk/src/ethercat && sudo ./a6_encoder eth0
```

- [ ] Position identical across the power cycle (battery holds the multi-turn).

## Phase 7 — Launch + verify

**Pi** — start the bridge:

```bash
sudo -E bash -c 'source /opt/ros/jazzy/setup.bash && source ~/wirehawk/install/setup.bash && ros2 run wirehawk_ethercat_bridge ethercat_bridge_node --ros-args -r state/motors:=real/state/motors --params-file ~/wirehawk/src/wirehawk_ethercat_bridge/config/ethercat_bridge.yaml'
```

**Laptop**:

```bash
cd ~/wirehawk && source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch wirehawk_control cdpr.launch.py world:=field backend:=real
```

Checks:

1. **FK** — `ros2 topic echo /cdpr/current_pose`
   - [ ] Reads ≈ the measured home (±0.3 m is fine until anchors are re-shot). Way off → anchor order or calibration wrong.
   - [ ] **This is also the absolute direction check:** if it's *meters* off (not ±0.3 m), a pair of directions is wrong — flip that pair in `direction` and re-launch.
2. **Direction (definitive)** — command a slow **+z (up)**.
   - [ ] All four cables **shorten**. Any that go slack → flip that `direction` to `-1`, re-commit, re-pull, re-launch.
3. **Handedness** — nudge **+x**.
   - [ ] Hawk moves the physical direction you expect.

## Phase 8 — The run

- WASD, slow and gentle.
- **Stay well inside the hull** (workspace is still a loose box — its corners are slack zones).
- Watch `/cdpr/diagnostic`: FK-convergence flag + the four tensions (a tension near zero = a slack cable = near the edge).

---

### Files touched, and where

| File | What you edit |
|---|---|
| `~/mast_position.py` | `MEASUREMENTS` (6 distances) |
| `src/wirehawk_control/scripts/commission_field.py` | `ANCHORS_XY`, `DISTS_3D`, `PULLEY_Z`, `MARGIN` |
| `src/wirehawk_control/config/cdpr_params_field.yaml` | generated — don't hand-edit |
| `src/wirehawk_ethercat_bridge/config/ethercat_bridge.yaml` | `direction` (hand), `counts_offset` (via `calibrate.py`) |
| `src/wirehawk_gazebo/worlds/wirehawk_world_field.sdf` | generated — don't hand-edit |

### Biggest first-try risks

1. **4↔2 measured wrong** → ~0.3 m anchor error (visible, not fatal).
2. **Motor direction** → garbage FK, but caught cleanly by the +z test before anything fast.
3. **Moving the hawk** between the Phase-3 measurement and Phase-6 calibration → wrong `counts_offset`.
4. **Forgetting `colcon build`** after editing a YAML → "Error opening YAML file".
