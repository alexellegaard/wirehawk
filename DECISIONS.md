# Wirehawk — Decisions Log

Short decisions with "why". Newest on top. If a decision is to be reconsidered, bring measured
data/evidence — not just an opinion.

---

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
