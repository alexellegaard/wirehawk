# EtherCAT bench notes (A6-EC, SOEM)

Hard-won numbers and constraints from the 2-motor bench, so the next session
doesn't re-derive them. Newest findings at top.

## Position register rollover (±160 rev)

The A6 position object wraps at `±0x01400000` = ±20,971,520 counts = ±160 rev
(17-bit encoder × 160). On the bench (mirror mode, no cable) the target stays
well inside one wrap, but the **real cable spool** moves hundreds of revs per
axis, so a raw position readback *will* cross the boundary.

- The wrap shows up in a log as ONE sample with a ~20.94M-count delta between
  two otherwise-smooth segments — a data artifact, not real motion. Strip that
  single jump before computing velocity/noise, or the std is ~100× too large.
- The drive's own position loop handles the wrap internally (it tracks the
  multi-turn absolute value); the hazard is only in **our** int32 readback and
  any dead-reckoning/velocity math in the bridge or control layer.
- When the spool is brought online, the bridge must either unwrap the counts in
  int64 (track rev count from the 0x6063 multi-turn value) or the control layer
  must treat position as modulo 2^25. Do not accumulate velocity from raw
  deltas across the boundary.

## Sync rate

`cycle_ns: 4000000` (250 Hz / 4 ms) — see DECISIONS.md 2026-10-05. Must be an
integer multiple of 250 µs and ≥ ~1.25× the command period (~100 Hz control).

## Known parameters (factory, as shipped)

- Gains: Kp=400 (0x2001:01), Kv=250 (0x2001:02).
- Speed-feedback filter: sel=0 internal, cutoff 8000 Hz, avg=0 (0x2001:17/18/19).
- Load moment of inertia ratio: 0x2000:07 = 100 (verify semantics before trusting).
- Sync mode: 0x2013:06 — set to 2 (Sync 2) for a host with >1 µs jitter.

## Tools (src/ethercat/)

| Tool | What it does |
|------|--------------|
| `a6_gains`   | Full vendor OD dump (0x2000–0x2042), flags DIFFs between the two slaves |
| `a6_sync`    | DC clock diagnostic: 64-bit 0x0910 cycle jitter (both slaves), optional 0x0934 sweep |
| `a6_filter`  | Read/set speed-feedback filter (0x2001:17/18/19) |
| `a6_gain`    | Read/set position/speed loop gains (0x2001:01/02) |
| `a6_set_profile` | Read/set profile accel/decel (0x6083/0x6084 — PP mode only, red herring in CSP) |
| `a6_step_test` / `a6_two_motors` | Gentle ramped bench-motion scripts |

All SDO tools run in Safe-OP (motors disabled); they never enable the drives.
