"""Ideal square-packed cable-on-drum model: cable length <-> encoder counts.

Single source of truth for the winch geometry. Both the controller
(length -> counts to command; counts -> length for the FK feedback) and the
sim bridge (counts -> length to feed the Gazebo physics) import THIS module, so
the two sides cannot disagree on the mapping.

The model is IDEAL: perfect controlled traverse, no slip, no stretch, no
inertia. Cable axial stretch is deliberately not here — it is handled by the
elastic forward kinematics (wirehawk_control) and by the catenary plugin
(wirehawk_gazebo). Encoder counts reflect drum rotation only.

Conventions:
  * "outside" = cable paid off the drum [m]; "wound" = total_m - outside.
  * counts == 0  <=>  fully unwound (outside == total_m, empty drum).
  * counts increase as cable is wound IN (outside decreases, the drum fills).
  * counts = motor encoder counts = drum_turns * counts_per_rev * gear_ratio.

Calibration note (real system only): the drum rotation measures TOTAL paid-out
cable (free span + the fixed mast run from winch to top pulley). The elastic-FK
needs the free span, so the real controller must subtract a per-mast constant.
That is a calibration offset applied outside this module; here "length" is the
drum's paid-out length, which in the current sim (no modelled winch/mast run)
equals the free span directly.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml


@dataclass(frozen=True)
class WinchSpec:
    total_m: float = 65.0          # cable inventory per winch [m]
    core_d_m: float = 0.020        # drum core diameter [m]
    rope_d_m: float = 0.002        # cable diameter [m]
    width_m: float = 0.150         # winding width [m]
    pitch_m: float = 0.0021        # axial pitch per turn [m]
    flange_d_m: float = 0.064      # flange diameter [m]
    turns_per_layer: int = 70      # turns per layer (ideal square packing)
    max_layers: int = 9            # maximum winding layers
    counts_per_rev: int = 131072   # encoder counts per motor revolution
    gear_ratio: float = 1.0        # motor revolutions per drum revolution

    def __post_init__(self):
        dims = (self.total_m, self.core_d_m, self.rope_d_m, self.width_m,
                self.pitch_m, self.flange_d_m, self.gear_ratio)
        if not all(math.isfinite(v) and v > 0 for v in dims):
            raise ValueError('dimensions must be finite and positive')
        ints = (self.turns_per_layer, self.max_layers, self.counts_per_rev)
        if any(type(v) is not int or v < 1 for v in ints):
            raise ValueError('turn/layer/counts values must be positive ints')
        if self.pitch_m < self.rope_d_m:
            raise ValueError('pitch cannot be smaller than rope diameter')
        occupied = (self.turns_per_layer - 1) * self.pitch_m + self.rope_d_m
        if occupied > self.width_m + 1e-12:
            raise ValueError('turns do not fit drum width')
        if self.core_d_m + 2 * self.max_layers * self.rope_d_m >= self.flange_d_m:
            raise ValueError('flange must project beyond cable layers')


def _layer_circumference(spec: WinchSpec, layer: int) -> float:
    """Pitch-circle circumference of `layer` (1-indexed)."""
    d = spec.core_d_m + (2 * layer - 1) * spec.rope_d_m
    return math.pi * d


def _wound_to_turns(wound: float, spec: WinchSpec) -> float:
    """Total drum turns needed to wind `wound` metres (integrate through layers)."""
    remaining = wound
    turns = 0.0
    for layer in range(1, spec.max_layers + 1):
        if remaining <= 1e-12:
            break
        circ = _layer_circumference(spec, layer)
        length = min(remaining, spec.turns_per_layer * circ)
        turns += length / circ
        remaining -= length
    if remaining > 1e-9:
        raise ValueError(f'cable ({wound:.3f} m wound) exceeds spool capacity')
    return turns


def _turns_to_wound(turns: float, spec: WinchSpec) -> float:
    """Wound length [m] corresponding to `turns` drum revolutions."""
    remaining_turns = turns
    wound = 0.0
    for layer in range(1, spec.max_layers + 1):
        circ = _layer_circumference(spec, layer)
        if remaining_turns <= spec.turns_per_layer + 1e-12:
            wound += max(0.0, remaining_turns) * circ
            remaining_turns = 0.0
            break
        wound += spec.turns_per_layer * circ
        remaining_turns -= spec.turns_per_layer
    else:
        raise ValueError(f'{turns:.3f} turns exceeds spool capacity')
    return wound


def length_to_counts(length_m: float, spec: WinchSpec) -> int:
    """Outside (paid-out) length -> absolute encoder counts (int)."""
    if not math.isfinite(length_m) or not 0.0 <= length_m <= spec.total_m:
        raise ValueError(f'outside length {length_m} out of range [0, {spec.total_m}]')
    wound = spec.total_m - length_m
    turns = _wound_to_turns(wound, spec)
    return int(round(turns * spec.counts_per_rev * spec.gear_ratio))


def counts_to_length(counts: int, spec: WinchSpec) -> float:
    """Absolute encoder counts -> outside (paid-out) length [m]."""
    if type(counts) is not int:
        raise ValueError('counts must be int')
    if counts < 0:
        raise ValueError('negative counts unsupported (counts >= 0 convention)')
    turns = counts / (spec.counts_per_rev * spec.gear_ratio)
    wound = _turns_to_wound(turns, spec)
    return spec.total_m - wound


def lengths_to_counts(lengths, spec: WinchSpec) -> np.ndarray:
    """Elementwise length -> counts (int64 array)."""
    return np.array([length_to_counts(float(L), spec)
                     for L in np.atleast_1d(lengths)], dtype=np.int64)


def counts_to_lengths(counts, spec: WinchSpec) -> np.ndarray:
    """Elementwise counts -> length (float array)."""
    return np.array([counts_to_length(int(c), spec)
                     for c in np.atleast_1d(counts)], dtype=float)


def load_spec(path) -> WinchSpec:
    """Load a WinchSpec from a spool.yaml file (reads the `spool:` mapping)."""
    data = yaml.safe_load(Path(path).read_text())
    return WinchSpec(**data['spool'])
