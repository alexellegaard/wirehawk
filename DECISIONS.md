# Wirehawk — Decisions Log

Short decisions with "why". Newest on top. If a decision is to be reconsidered, bring measured
data/evidence — not just an opinion.

---

## 2026-09-29 — EtherCAT bridge: dedicated C++ rclcpp node (RT thread + topic bridge), kinematic-only

**Decision:** Implement the EtherCAT master as a dedicated C++ rclcpp package
(`wirehawk_ethercat_bridge`) running the SOEM CSP loop in a dedicated RT thread, exposing motor
positions/torque/status over ROS2 topics. The existing Python control (`wirehawk_control`) stays
Python and talks to it over topics.

**Why:** The RT loop must be C (SOEM + SCHED_FIFO + pinned core), which can't live in a Python
process (GIL, no realtime guarantee). A separate process also lets the sim/control run on the
laptop while the EtherCAT master runs on the Pi 5, communicating over ROS2 topics on the shared
WiFi (fine for the bench; deploy co-located later). The bridge is a dumb motor bridge — encoder
counts, no cable/spool/CDPR geometry — so the meters↔counts conversion and geometry stay in the
control layer (single source of truth). Kinematic-only: position command, torque is read-only
feedback (0x6077 already in the TxPDO). The RT loop enforces max_speed/max_accel as a hard
safety envelope via a trapezoidal interpolator.

**Consequence:** custom messages in a separate `wirehawk_msgs` package (the interface contract
between C++ bridge and Python control); the bridge handles N motors (1 bench, 2 for the 2D CDPR,
4 for the full robot).

## 2026-09-23 — ROS2–EtherCAT architecture: own SOEM bridge (not fastcat), mirror fastcat's structure

**Decision:** Build our own SOEM + ROS2 EtherCAT bridge for control. Do not use NASA JPL's
fastcat/fcat stack, but mirror its three structural ideas.

**Why:** fastcat's driver layer (jsd) only supports Elmo Gold/Platinum drives and Beckhoff I/O —
not our A6-EC1000H2B1-M17 (Leadshine/StepperOnline). Using fastcat would require writing a jsd
driver for the A6 (more work than a raw SOEM bridge) plus learning its YAML/topic conventions.
Raw SOEM is also the learning path we want.

**Consequence (what we mirror):** (1) layering SOEM I/O → device abstraction → low-level control
(1 kHz, RT thread) → ROS2 (high-level: FK/trajectory); (2) split-init: ROS node setup before
EtherCAT init, so slave watchdogs don't time out during slow ROS startup; (3) separate `state/`
(telemetry) and `cmd/` (commands) topics with SDO as services.
