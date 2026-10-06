"""Backend-agnostic CDPR controller core (pure numpy, no ROS).

Converts a task-space velocity target into per-winch encoder-count commands and
reconstructs the payload pose from *measured* encoder counts via elastic forward
kinematics. It talks only in encoder counts to the outside world, so it never
knows whether those counts came from the real EtherCAT bridge or the Gazebo
bridge — the backend is chosen by which process publishes `state/motors` and
subscribes `cmd/motors`.

Fidelity: this controller reads only what a real sensor gives (encoder counts).
It never reads ground-truth Cartesian pose; position is reconstructed by FK.
Cable axial stretch is modelled in the FK (EA), matching the catenary plugin.
"""
from __future__ import annotations

import numpy as np

from wirehawk_spool.spool_model import WinchSpec, counts_to_lengths, lengths_to_counts
from .kinematics import CDPRKinematics


class CDPRController:
    def __init__(self, anchors, start_pos, ws_min, ws_max, max_speed, max_accel,
                 EA, mass, fk_gain, max_cable_speed, spec: WinchSpec):
        self.kinematics = CDPRKinematics(np.asarray(anchors, dtype=float))
        self.start_pos = np.asarray(start_pos, dtype=float)
        self.ws_min = np.asarray(ws_min, dtype=float)
        self.ws_max = np.asarray(ws_max, dtype=float)
        self.max_speed = float(max_speed)
        self.max_accel = float(max_accel)
        self.EA = float(EA)
        self.mass = float(mass)
        self.fk_gain = float(fk_gain)
        self.max_cable_speed = float(max_cable_speed)
        self.spec = spec

        self.target_pos = np.clip(self.start_pos, self.ws_min, self.ws_max)
        self.target_vel = np.zeros(3, dtype=float)
        self.filtered_vel = np.zeros(3, dtype=float)
        self.P_est = self.target_pos.copy()
        # Nominal measured length until the first encoder readback arrives.
        self.L_m = self.kinematics.compute_commanded_lengths(self.target_pos)
        self.L_integral = np.zeros(self.kinematics.num_cables, dtype=float)
        self.command_counts = lengths_to_counts(self.L_m, self.spec)

    def set_target_velocity(self, v) -> None:
        """Task-space velocity command (from /cmd_vel)."""
        v = np.asarray(v, dtype=float)
        self.target_vel = np.clip(v, -self.max_speed, self.max_speed)

    def stop(self) -> None:
        self.target_vel = np.zeros(3, dtype=float)

    def step(self, dt: float, measured_counts) -> np.ndarray:
        """Advance one control step; returns the commanded encoder counts.

        measured_counts: int array of per-winch encoder positions, or None if
        no feedback has arrived yet (then the last known length is reused).
        """
        # 1. Acceleration slew-rate limiter.
        vel_diff = self.target_vel - self.filtered_vel
        diff_mag = float(np.linalg.norm(vel_diff))
        max_dv = self.max_accel * dt
        if diff_mag > max_dv and diff_mag > 0.0:
            self.filtered_vel += (vel_diff / diff_mag) * max_dv
        else:
            self.filtered_vel = self.target_vel.copy()

        # 2. Integrate velocity -> target position with a soft workspace
        #    boundary: brake each axis only as it approaches an edge, using the
        #    constant-acceleration stopping distance v <= sqrt(2*a*d). The clamp
        #    is ONE-SIDED per axis: moving away from a boundary is never limited
        #    (a symmetric clip would pin the TCP at the edge and trap it).
        d_upper = np.maximum(self.ws_max - self.target_pos, 0.0)
        d_lower = np.maximum(self.target_pos - self.ws_min, 0.0)
        vmax_upper = np.sqrt(2.0 * self.max_accel * d_upper)   # max speed toward ws_max
        vmax_lower = np.sqrt(2.0 * self.max_accel * d_lower)   # max speed toward ws_min
        vel = np.minimum(self.filtered_vel, vmax_upper)        # brake the +axis approach
        vel = np.maximum(vel, -vmax_lower)                     # brake the -axis approach
        self.target_pos += vel * dt
        self.target_pos = np.clip(self.target_pos, self.ws_min, self.ws_max)

        # 3. Measured cable lengths from encoder counts (the only feedback).
        if measured_counts is not None:
            self.L_m = counts_to_lengths(measured_counts, self.spec)

        # 4. Elastic FK from MEASURED lengths (not the commanded ones).
        self.P_est, _converged = self.kinematics.elastic_forward_kinematics(
            self.L_m, self.EA, self.mass, self.P_est)
        self.P_est = np.clip(self.P_est, self.ws_min, self.ws_max)

        # 5. Position error (target vs FK estimate).
        e = self.target_pos - self.P_est

        # 6. Integral length feedback (compensates stretch / steady-state error).
        u = self.kinematics.anchor_unit_vectors(self.P_est)
        proj = u @ e
        self.L_integral -= self.fk_gain * proj * dt
        self.L_integral = np.clip(self.L_integral, -5.0, 5.0)

        # 7. Commanded lengths = geometric IK + accumulated correction.
        L_ff = self.kinematics.compute_commanded_lengths(self.target_pos)
        L_d = np.clip(L_ff + self.L_integral, 0.1, self.spec.total_m)

        # 8. Commanded encoder counts, with a per-motor cable-speed clamp so the
        #    commanded drum rate never exceeds the drive's rated speed. The real
        #    EtherCAT bridge enforces the same limit (max_speed), so sim and
        #    real stay identical.
        new_counts = lengths_to_counts(L_d, self.spec)
        max_delta = int(round(self.max_cable_speed * dt))
        delta = np.clip(new_counts - self.command_counts, -max_delta, max_delta)
        self.command_counts = self.command_counts + delta
        return self.command_counts

    def snapshot(self) -> dict:
        """Observable state for telemetry/debug (no ground truth leaks back in)."""
        return {
            'position': self.P_est.copy(),          # FK estimate (from counts)
            'target': self.target_pos.copy(),
            'measured_lengths': self.L_m.copy(),
            'command_counts': self.command_counts.copy(),
        }
