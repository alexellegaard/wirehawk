# Wirehawk — Field Operation (calibrated system)

Use this for the day-to-day run once the system is calibrated. No pulling, no building, no re-measuring. If the physical setup changed, go back to **`FIELD_COMMISSIONING.md`**.

## 1. Power on

- [ ] Drives + 24 V PSU on (releases the holding brakes).

## 2. Bridge (Pi)

```bash
sudo -E bash -c 'source /opt/ros/jazzy/setup.bash && source ~/wirehawk/install/setup.bash && ros2 run wirehawk_ethercat_bridge ethercat_bridge_node --ros-args -r state/motors:=real/state/motors --params-file ~/wirehawk/src/wirehawk_ethercat_bridge/config/ethercat_bridge.yaml'
```

## 3. Controller (laptop)

```bash
cd ~/wirehawk && source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch wirehawk_control cdpr.launch.py world:=field backend:=real
```

## 4. Drive (laptop, second terminal)

```bash
cd ~/wirehawk && source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 run wirehawk_control keyboard_teleop --ros-args --params-file ~/wirehawk/install/wirehawk_control/share/wirehawk_control/config/cdpr_params_field.yaml
```

- WASD, slow and gentle. Stay well inside the hull.
- Watch `/cdpr/diagnostic`: FK-convergence flag + four tensions (tension near zero = slack cable = near the edge).
- **One `/cmd_vel` source at a time** — `keyboard_teleop` and `trajectory_planner` must not run together.

## Rules

1. **`backend:=real` for the physical setup. Never `backend:=sim` with the drives powered** — the sim echo starts at home, so the controller commands the real motors to home as a full-speed slew.
2. **Ropes disconnected from the TCP** for any first-enable / direction test.
3. **No abrupt motions** — gentle ramped commands only.
4. **E-stop** = hard stop (cuts power, brakes engage). Note the known brake gap: a *drive* fault does not engage the brake until the relay is re-wired (see `FIELD_COMMISSIONING.md`).
