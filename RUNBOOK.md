# Wirehawk runbook — controller, sim, and real motors

## Machines

- **Pi (`vision`)** — the EtherCAT master. Runs `ethercat_bridge_node`, the only thing that talks to the drives.
- **Laptop** — everything else: `cdpr_node` (controller), `sim_bridge`, Gazebo, `motor_logger`, and the driver (teleop / bird / planner).

Common laptop preamble (every terminal):

```bash
cd ~/wirehawk && source /opt/ros/jazzy/setup.bash && source install/setup.bash
```

## The controller always publishes `cmd/motors`

`cdpr_node` publishes `cmd/motors` once, and **every bridge that is running receives it**:

- the **sim bridge** (`sim_bridge`, launched by `cdpr.launch.py`) echoes it back as perfect-tracking feedback;
- the **real bridge** (`ethercat_bridge_node`, on the Pi) drives the actual motors.

The `backend` launch arg only decides **which feedback the controller reads** (`sim` echo or `real` encoder). Which motors actually move depends only on whether the Pi bridge is running.

## The four scenarios

| # | Pi bridge (real motors) | Sim bridge (Gazebo) | `backend` | Controller reads | Motors do |
|---|---|---|---|---|---|
| (a) | OFF | ON | `sim` | sim echo | nothing (no bridge) |
| (b) | ON | OFF | `real` | real encoder | resume-hold |
| (c) | ON | ON | `sim` | sim echo (home) | **slew to home — dangerous** |
| (d) | ON | ON | `real` | real encoder | resume-hold |

### (a) sim only — motors off

No Pi bridge. Controller commands the sim echo and reads it back. Pure simulation.

Laptop:

```bash
ros2 launch wirehawk_control cdpr.launch.py world:=40x40_3m backend:=sim
```

then a driver (second terminal) — see "Driving" below.

### (b) real motors only — no sim

Pi bridge up, but **no** `cdpr.launch.py` (that would add the sim). Controller reads the real encoder, no Gazebo visualization.

Pi:

```bash
sudo -E bash -c 'source /opt/ros/jazzy/setup.bash; source ~/wirehawk/install/setup.bash; ros2 run wirehawk_ethercat_bridge ethercat_bridge_node --ros-args -r state/motors:=real/state/motors --params-file ~/wirehawk/src/wirehawk_ethercat_bridge/config/ethercat_bridge.yaml'
```

Laptop (manual controller node, NOT the launch file):

```bash
ros2 run wirehawk_control cdpr_node --ros-args \
  --params-file ~/wirehawk/src/wirehawk_control/config/cdpr_params_40x40_3m.yaml \
  -r state/motors:=/real/state/motors
```

then a driver.

### (c) both — feedback from sim (DANGEROUS)

Both bridges up, `backend:=sim`. The sim echo starts at home, so the controller commands home and the **real motors slew there at full speed**. Do not run with motors powered.

Pi: bridge (as in (b)).
Laptop:

```bash
ros2 launch wirehawk_control cdpr.launch.py world:=40x40_3m backend:=sim
```

### (d) both — feedback from real (the normal real run)

Both bridges up, `backend:=real`. Controller reads the real encoder; the sim bridge is a spectator. Resume-holds the motors.

Pi: bridge (as in (b)).
Laptop:

```bash
ros2 launch wirehawk_control cdpr.launch.py world:=40x40_3m backend:=real
```

then a driver.

## Driving — pick exactly ONE (they all publish `/cmd_vel`)

```bash
# WASD
ros2 run wirehawk_control keyboard_teleop --ros-args \
  --params-file ~/wirehawk/src/wirehawk_control/config/cdpr_params_40x40_3m.yaml

# Bird (dummy bird at (X,Y))
ros2 run wirehawk_control bird_sweeper --ros-args \
  --params-file ~/wirehawk/src/wirehawk_control/config/cdpr_params_40x40_3m.yaml
ros2 run wirehawk_control missionplanner_dummy --ros-args \
  -p target_position:=[X,Y,1.5] -p speed:=5.0

# Goal-based planner
ros2 run wirehawk_control trajectory_planner --ros-args \
  --params-file ~/wirehawk/src/wirehawk_control/config/cdpr_params_40x40_3m.yaml
```

## Rules

- **Power the drives on before** the Pi bridge / `a6_*` tools (else `no slaves`).
- **Never run `backend:=sim` with the motors powered** — that is scenario (c), the full-speed slew.
- **One `/cmd_vel` source at a time** — `keyboard_teleop`, `bird_sweeper`, `trajectory_planner` fight each other.
- Calibrate once (Pi): `python3 src/wirehawk_control/scripts/calibrate.py 40x40_3m`. With spools on, keep motion gentle (heavier inertia than a bare shaft).
