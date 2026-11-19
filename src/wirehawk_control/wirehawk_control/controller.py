"""Backend-agnostic CDPR controller core (pure numpy, no ROS).

Converts a task-space velocity target into per-winch free-span cable-length
commands and reconstructs the payload pose from *measured* free spans via forward
kinematics. It talks only in free-span metres (anchor/pulley -> TCP) to the
outside world, so it never knows whether those lengths came from the real
EtherCAT bridge or the Gazebo bridge — the backend is chosen by which process
publishes `state/motors` and subscribes `cmd/motors`. The raw-encoder -> free-span
conversion lives entirely in the bridge.

Fidelity: this controller reads only what a real sensor gives (cable lengths).
It never reads ground-truth Cartesian pose; position is reconstructed by FK.
Cable axial stretch is modelled in the FK (EA), matching the catenary plugin.
"""
from __future__ import annotations

import numpy as np

from .kinematics import CDPRKinematics


class CDPRController:
    def __init__(self, anchors, start_pos, ws_min, ws_max, max_speed, max_accel, max_decel,
                 EA, mass, fk_gain, max_cable_speed, max_cable_accel):
        self.kinematics = CDPRKinematics(np.asarray(anchors, dtype=float))
        self.start_pos = np.asarray(start_pos, dtype=float)
        self.ws_min = np.asarray(ws_min, dtype=float)
        self.ws_max = np.asarray(ws_max, dtype=float)
        self.max_speed = float(max_speed)
        self.max_accel = float(max_accel)
        self.max_decel = float(max_decel)
        if self.max_decel <= 0.0:
            self.max_decel = self.max_accel  # unset -> symmetric
        self.EA = float(EA)
        self.mass = float(mass)
        self.fk_gain = float(fk_gain)
        self.max_cable_speed = float(max_cable_speed)   # m/s (free span)
        self.max_cable_accel = float(max_cable_accel)   # m/s^2

        self.target_pos = np.clip(self.start_pos, self.ws_min, self.ws_max)
        self.target_vel = np.zeros(3, dtype=float)
        self.filtered_vel = np.zeros(3, dtype=float)
        self.P_est = self.target_pos.copy()
        # Nominal measured length until the first feedback readback arrives.
        self.L_m = self.kinematics.compute_commanded_lengths(self.target_pos)
        self.L_integral = np.zeros(self.kinematics.num_cables, dtype=float)
        self.fk_converged = True
        self.tensions = np.zeros(self.kinematics.num_cables, dtype=float)
        self.command_lengths = self.L_m.copy()          # free-span [m]
        self.command_vel = np.zeros(self.kinematics.num_cables, dtype=float)  # m/s
        self.command_was_clamped = np.zeros(self.kinematics.num_cables, dtype=bool)
        # Resume state: latch the actual pose on startup instead of commanding
        # home. `resumed` flips once the first readback seeds the command;
        # `target_latched` flips once the FK converges and the target is latched
        # to the estimated pose.
        self.resumed = False
        self.target_latched = False

    def set_target_velocity(self, v) -> None:
        """Task-space velocity command (from /cmd_vel)."""
        v = np.asarray(v, dtype=float)
        self.target_vel = np.clip(v, -self.max_speed, self.max_speed)

    def stop(self) -> None:
        self.target_vel = np.zeros(3, dtype=float)

    def step(self, dt: float, measured_lengths) -> np.ndarray:
        """Advance one control step; returns the commanded free-span lengths [m].

        measured_lengths: float array of per-winch free spans (anchor->TCP), or
        None if no feedback has arrived yet (then the last known length is reused).
        """
        # 1. Asymmetric slew-rate limiter: snappy acceleration (max_accel),
        #    gentle braking/reversal (max_decel). Overshoot happens only on
        #    deceleration (speed-loop integral windup on reversal), so brake
        #    gently but accelerate hard.
        vel_diff = self.target_vel - self.filtered_vel
        diff_mag = float(np.linalg.norm(vel_diff))
        speeding_up = float(np.dot(vel_diff, self.filtered_vel)) >= 0.0
        a = self.max_accel if speeding_up else self.max_decel
        max_dv = a * dt
        if diff_mag > max_dv and diff_mag > 0.0:
            self.filtered_vel += (vel_diff / diff_mag) * max_dv
        else:
            self.filtered_vel = self.target_vel.copy()

        # 2. Integrate velocity -> target position with a soft workspace
        #    boundary: brake each axis only as it approaches an edge, using the
        #    constant-acceleration stopping distance v <= sqrt(2*a*d). The clamp
        #    is ONE-SIDED per axis: moving away from a boundary is never limited.
        d_upper = np.maximum(self.ws_max - self.target_pos, 0.0)
        d_lower = np.maximum(self.target_pos - self.ws_min, 0.0)
        vmax_upper = np.sqrt(2.0 * self.max_decel * d_upper)
        vmax_lower = np.sqrt(2.0 * self.max_decel * d_lower)
        vel = np.minimum(self.filtered_vel, vmax_upper)
        vel = np.maximum(vel, -vmax_lower)
        self.target_pos += vel * dt
        self.target_pos = np.clip(self.target_pos, self.ws_min, self.ws_max)

        # 3. Measured cable lengths (free span, metres) — the only feedback.
        if measured_lengths is not None:
            self.L_m = np.asarray(measured_lengths, dtype=float)
            if not self.resumed:
                # Resume: seed the command to the actual lengths so the
                # controller holds wherever the cables are (no home slew).
                self.command_lengths = self.L_m.copy()
                self.resumed = True

        # 4. FK from MEASURED lengths (not the commanded ones). Startup uses the
        #    robust subset trilateration (no initial guess, slack-tolerant); once
        #    latched, the static-equilibrium (elastic) FK refines the pose.
        if self.resumed and not self.target_latched:
            self.P_est, self.fk_converged = self.kinematics.taut_subset_fk(self.L_m)
        else:
            self.P_est, self.fk_converged = self.kinematics.static_equilibrium_fk(
                self.L_m, self.EA, self.mass, self.P_est)
        # Reject an unphysical FK pose (outside the workspace — e.g. below the
        # floor from miscalibrated lengths) instead of clipping it into the
        # workspace and latching to the clipped phantom pose. Holding is the
        # safe fallback: the TCP stays put until the FK reads a plausible pose.
        if np.any(self.P_est < self.ws_min) or np.any(self.P_est > self.ws_max):
            self.fk_converged = False
        self.P_est = np.clip(self.P_est, self.ws_min, self.ws_max)

        # 4b. Cable tensions at the FK estimate — the slack indicator for
        # workspace-boundary tuning (a far cable's tension -> 0 means the payload
        # hangs from fewer cables and the FK loses observability).
        self.tensions = self.kinematics.cable_tensions(self.P_est, self.L_m, self.EA)

        # Hold the winches whenever the FK is invalid (unphysical pose) — at
        # startup OR after the lengths drift post-latch. Without this, the
        # feedforward IK(target_pos) keeps driving the cables toward a phantom
        # pose (e.g. releasing rope toward a clipped above-ceiling pose). Re-seed
        # the command to the measured lengths so the winches stay put.
        if not self.fk_converged:
            self.command_lengths = self.L_m.copy()
            self.command_vel = np.zeros(self.kinematics.num_cables, dtype=float)
            return self.command_lengths

        # Resume startup: the FK has converged (guaranteed here), so latch the
        # target to the estimated pose and resume normal control from there.
        if self.resumed and not self.target_latched:
            self.target_pos = self.P_est.copy()
            self.target_latched = True

        # 5. Position error (target vs FK estimate).
        e = self.target_pos - self.P_est

        # 6. Integral length feedback (compensates stretch / steady-state error).
        u = self.kinematics.anchor_unit_vectors(self.P_est)
        proj = u @ e
        # Anti-windup: if a motor's command was rate-limited last step, its
        # position error is a transient the drive is still catching up on — hold
        # the integral. The ±0.5 m bound is a safety clamp. Convergence gate:
        # under fast motion the elastic FK finds no static equilibrium and falls
        # back to the geometric estimate, so the position error is a
        # fallback-jump artifact — freeze the integral then.
        freeze = not self.fk_converged
        self.L_integral -= np.where(self.command_was_clamped | freeze, 0.0,
                                    self.fk_gain * proj * dt)
        # Null-space projection: 4 cables drive 3 DOF, so L_integral has a
        # redundant-cable component that drifts uncorrected. Keep only the
        # position-relevant part (least-squares projection onto u).
        coeffs = np.linalg.lstsq(u, self.L_integral, rcond=None)[0]
        self.L_integral = u @ coeffs
        self.L_integral = np.clip(self.L_integral, -0.5, 0.5)

        # 7. Commanded lengths = geometric IK + accumulated correction.
        L_ff = self.kinematics.compute_commanded_lengths(self.target_pos)
        L_d = np.clip(L_ff + self.L_integral, 0.05, 60.0)

        # 8. Commanded free spans, with a per-cable speed clamp so the commanded
        #    drum rate never exceeds the task cable-speed limit. The real bridge
        #    enforces the drive's own limit (max_speed), so sim and real agree.
        desired_vel = (L_d - self.command_lengths) / dt
        max_dv = self.max_cable_accel * dt
        accel_clamped = np.abs(desired_vel - self.command_vel) > max_dv
        self.command_vel += np.clip(desired_vel - self.command_vel, -max_dv, max_dv)
        # Uniform speed clamp: if ANY cable would exceed max_cable_speed, scale
        # ALL cable velocities by the same factor so the fastest just hits the
        # limit (preserves task-space direction).
        max_v = float(np.max(np.abs(self.command_vel)))
        speed_clamped = max_v > self.max_cable_speed
        if speed_clamped:
            self.command_vel *= self.max_cable_speed / max_v
        self.command_was_clamped = accel_clamped | speed_clamped
        self.command_lengths = self.command_lengths + self.command_vel * dt
        return self.command_lengths

    def snapshot(self) -> dict:
        """Observable state for telemetry/debug (no ground truth leaks back in)."""
        return {
            'position': self.P_est.copy(),          # FK estimate (from lengths)
            'target': self.target_pos.copy(),
            'measured_lengths': self.L_m.copy(),
            'tensions': self.tensions.copy(),
            'command_lengths': self.command_lengths.copy(),
        }
