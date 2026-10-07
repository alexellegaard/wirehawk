# EtherCAT bench scripts (A6-EC + SOEM)

Standalone C tools for bringing up and debugging the STEPPERONLINE A6-EC servo
drives over EtherCAT (SOEM master). All scripts are 4-motor aware: they drive
every slave found on the bus (up to 4), daisy-chained.

## Build

```bash
cd ~/wirehawk/src/ethercat
make              # build all scripts (needs SOEM at ~/SOEM; built automatically if missing)
make a6_step_test # build just one
make clean        # remove all binaries
```

## Safety

- Run with `sudo` (raw EtherCAT socket needs root): `sudo -E bash -c 'source /opt/ros/jazzy/setup.bash; sudo ./a6_xxx eth0'` if sudo resets your PATH.
- **Free shafts / no load assumed** for the motion tests — remove anything on the shaft first.
- Two tools move the motors: `a6_step_test`, `a6_profile_test`, and `a6_inertia tune`. Everything else is Safe-OP (servos stay off).
- Drives are daisy-chained: slave 1 is the DC reference; slaves 2–4 are in the relayed clock position (more jitter — `a6_sync` measures it).

## Tools

| script | what it does |
|---|---|
| `a6_step_test` | ramp-and-hold (±1 rev), reports peak following-error + settle time per motor |
| `a6_profile_test` | trapezoid speed/accel sweep: coast velocity chatter + dwell following error |
| `a6_demo` | identify each motor (1 rev each, motor 1→4), then a synchronized sine wave |
| `a6_gains` | full vendor OD dump (0x2000–0x2042 + CiA/identity), flags DIFFs across slaves |
| `a6_sync` | DC clock diagnostic: cycle jitter per slave + drift vs slave 1 |
| `a6_fault` | read 0x203F (drive fault) + 0x603F (CiA error) + 0x6041 (status) |
| `a6_reset` | clear a latched fault (control word 0x80) on all slaves |
| `a6_inertia` | read/set inertia ratio + stiffness; `tune` = offline inertia auto-tune |
| `a6_sdo` | generic SDO read/write on all slaves (any object) + EEPROM save |

## Usage

**No extra args** (just the interface):
```bash
sudo ./a6_step_test eth0     # ramp-and-hold, settle per motor
sudo ./a6_demo eth0          # identify each motor (1 rev), then synchronized wave (Ctrl-C to stop)
sudo ./a6_gains eth0         # dump + diff every vendor object
sudo ./a6_fault eth0         # read fault/error/status
sudo ./a6_reset eth0         # clear a fault on all slaves
```

**`a6_profile_test <speed_rev_s> <accel_rev_s2> [coast_ms] [dwell_ms]`**
```bash
sudo ./a6_profile_test eth0 1 10             # gentle: 1 rev/s, 10 rev/s²
sudo ./a6_profile_test eth0 45 200           # aggressive: 45 rev/s, 200 rev/s²
sudo ./a6_profile_test eth0 45 20 2000 3000  # explicit 2 s coast, 3 s dwell
```

**`a6_sync [timefilter_ns]`** — optional; writes the DC time filter (0x0934) first if given (A6 default 3072 ns):
```bash
sudo ./a6_sync eth0        # measure jitter, per slave vs slave 1
sudo ./a6_sync eth0 2000   # set time filter to 2000 ns, then measure
```

**`a6_inertia`** — no arg = read; sub-commands:
```bash
sudo ./a6_inertia eth0               # read C00.04/05/06 + C07 group
sudo ./a6_inertia eth0 set 0         # load-inertia ratio C00.06 = 0 (free shaft)
sudo ./a6_inertia eth0 stiff 10      # stiffness level C00.05 = 10
sudo ./a6_inertia eth0 tune          # offline auto-tune  ← MOVES THE MOTORS
```

**`a6_sdo <idx> <sub> [value] [bytes]`** — generic read/write. `idx`/`sub` accept hex (`0x…`) or decimal; `bytes` is 1/2/4 (default 2 = U16). Omit `value` to read:
```bash
sudo ./a6_sdo eth0 0x2001 1            # read  Kp (0x2001:01) from all slaves
sudo ./a6_sdo eth0 0x2001 1 200        # write Kp = 200 (prints readback)
sudo ./a6_sdo eth0 0x2001 2 250        # write Kv = 250
sudo ./a6_sdo eth0 0x2001 17 0         # speed-feedback filter selection = 0
sudo ./a6_sdo eth0 0x2000 7 0          # load-inertia ratio C00.06 = 0
sudo ./a6_sdo eth0 0x6083 0 1310720 4  # U32 write (bytes=4): profile accel
sudo ./a6_sdo eth0 save                # store params to EEPROM
```

## Suggested bring-up order

1. `a6_gains eth0` — confirm all 4 drives are present and parameter-identical (or find the diff).
2. `a6_sync eth0` — confirm the DC clock locks clean on all 4 (watch slaves 2–4 for relay jitter).
3. `a6_step_test eth0` — confirm all 4 move and settle.
4. `a6_profile_test eth0 <speed> <accel>` — sweep speed/accel to split command-path vs drive.
5. `a6_fault` / `a6_reset` — when something trips.

## Common A6 objects (for `a6_sdo`)

Parameter numbering maps `Cxx.yy` → SDO `0x20xx:(yy+1)`. All U16 unless noted.

| object | meaning | factory |
|---|---|---|
| `0x2001:01` | Kp (position loop gain) | 400 |
| `0x2001:02` | Kv (speed loop gain) | 250 |
| `0x2001:17` | speed-feedback filter sel (0=internal, 1=low-pass, 2=avg, 3=observer, 4=none) | 0 |
| `0x2001:18` | low-pass cutoff Hz (8000 = no filtering) | 8000 |
| `0x2001:19` | overlapping-average times (0=none, 1=2× … 6=64×) | 0 |
| `0x2000:05` | auto-tuning mode C00.04 (0=manual, 1=standard, 2=positioning) | 1 |
| `0x2000:06` | stiffness level C00.05 (1–31) | 12 |
| `0x2000:07` | load-inertia ratio C00.06 (0–12000 %, 0=no load, 100=1:1) | 100 |
| `0x6083` / `0x6084` | profile accel/decel (U32, counts/s²) — PP mode only, red herring in CSP | — |
| `0x203F` | drive fault code (read) | — |
| `0x603F` | CiA 402 error code (read) | — |
| `0x6041` | status word (read; bit 3 = fault) | — |
| `0x6040` | control word (0x80 = fault reset) | — |
| `0x1010:01` | store params to EEPROM (write `0x65766173`) | — |

Parameter writes are **retentive** — they survive a power cycle, so undo a bad
value explicitly rather than assuming power-off resets it.
