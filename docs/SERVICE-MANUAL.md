# Wirehawk — Service Manual

> **Date:** 2026-10-29 · **Language:** English — Danish version: `SERVICE-MANUAL-DA.md`. Keep the two in sync. Mirrors `Wirehawk Service Manual.docx` (current build, 40×40×3 m).

## 1. System overview

Wirehawk is a 4-cable suspended CDPR with four 3 m masts in the roof corners, each carrying a winch (motor + servo drive + spool + anchor pulley). Each mast's winch runs a cable out to a shared 3 kg mass that can be steered around the robot's workspace. A vision system detects birds, and the robot moves the mass quickly toward them to scare them away.

### 1.1 Per-mast components (×4)

- Servo motor
- Servo drive
- Power cabinet
- Electrical cabinet
- Multi-turn encoder cable
- Winch

### 1.2 Control (mounted on mast 1)

- Raspberry Pi 5 (EtherCAT master + vision)
- Ethernet switch
- Power distribution board
- E-stop — TODO
- Fuses — TODO

## 2. Safety

- Always disconnect power before cleaning or servicing inside electrical cabinets or any other electrical part. Each mast runs on 230 V single-phase, and the whole system is supplied at 400 V three-phase.
- Ground (PE) is mandatory — never run the system ungrounded.
- The servo motors have a spring-applied holding brake: when 24 V is removed the shaft is locked. An unpowered motor therefore cannot free-wheel and cannot be turned by hand.
- The servo drive's own maintenance chapter warns: disconnect power before cleaning. Use only a blower gun or dry cloth — never gasoline, thinner, alcohol or acidic/alkaline detergents (A6 manual §14.1, p.270).
- The drive is DC-sync-only: when the EtherCAT master stops, the drive latches a sync-loss fault (ErC1.1). This is expected behaviour, not a defect, and it can be reset with a fault reset or restart (A6 manual §8.2.3 p.162).
- Work at height is regulated by Arbejdstilsynet (fall protection, risk assessment, trained personnel, etc.). This is outside the scope of this manual.

## 3. Component overview

| # | Component | Name | Qty |
|---|---|---|---|
| 1 | Servo motor | A6 1000 W EtherCAT, 3000 rpm, 3.18 Nm rated, brake (24 V / 3.2 Nm), 17-bit absolute encoder, IP67 | 4 |
| 2 | Servo drive | A6-EC1000H2B1-M17 | 4 |
| 3 | 24 V power supply | Mean Well LRS-100-24 (24 V / 4.5 A) | 4 |
| 4 | Multi-turn encoder cable | AS7-C-ENC076-BAT-3.0 (3 m) | 4 |
| 5 | Suspension line | Dynamica DM20, 2 mm, spliced, 65 m | 4 |
| 6 | Electrical cabinet | IP66 | 4 |
| 7 | Network cable | CAT6 U/UTP outdoor, 50 m | 4 |
| 8 | Anchor pulley | Biltema swivel wheel, double, 50 mm, with own pulley inserted | 4 |
| 9 | Spool | 3D-printed spool and spool base | 4 |
| 10 | E-stop | TODO | 1 |
| 11 | Fuses | TODO | — |

## 4. Service intervals

The system's components inherit their service intervals from the manufacturer. For now, the A6 motor and servo drive are the only components with a full maintenance table published by the manufacturer. Intervals marked "provisional" have no manufacturer-specified service interval yet and are, for now, inspection recommendations until the supplier's data is available.

### 4.1 Servo motor

Inherited from the A6-EC servo manual: https://files.omc-stepperonline.com/A6-EC_series_servo_drive_manual.pdf

| Item | Service interval | Service description |
|---|---|---|
| Bearing | 3–5 years (20,000–30,000 h) | Replace |
| Oil seal | 5,000 h | Replace |
| Single-turn encoder | 3–5 years (20,000–30,000 h) | Replace |
| Multi-turn encoder | Depends on the specific operating conditions | TODO |

### 4.2 Servo drive

Inherited from the A6-EC servo manual: https://files.omc-stepperonline.com/A6-EC_series_servo_drive_manual.pdf

| Item | Service interval | Service description |
|---|---|---|
| Bus filter capacitor | ~5 years | Replace |
| Cooling fan | 2–3 years (10,000–30,000 h) | Replace |
| Electrolytic capacitor (circuit board) | ~5 years | Replace |
| Pre-charge relay | 100,000 operations | Replace |
| Pre-charge resistor | 20,000 operations | Replace |

### 4.3 Provisional

The remaining components. TODO

## 5. Daily maintenance (drive) — A6 manual §14.1, p.270

Standard operating conditions the drive is rated for: **30 °C** annual-average ambient, **< 80 %** average load, **< 20 h** daily operation.

**Check daily:**
- Ambient temperature/humidity normal; no dust or foreign objects in/on the drive.
- No abnormal vibration or noise from the device.
- Supply voltage normal.
- No strange smell.
- No fibres on the air inlet.
- No foreign-object intrusion at the load end.

**Cleaning:**
- Remove dust (especially *metallic* dust) from the drive surface so it cannot enter the drive.
- Keep the drive well ventilated; keep the front connectors clean.

> Disconnect power before cleaning. Blower gun or dry cloth only — never gasoline, thinner, alcohol, or acidic/alkaline detergents (§14.1, p.270).

## 6. Periodic maintenance — procedure notes

- **Drive/motor wear items** (§14.2, p.271): intervals in §4; confirm with the vendor before replacement (§14.2 instruction: "contact us or the agent before replacement").
- **Key replacement** (§14.3.1, p.271–272): B-type plain keys on the ASMK2 motor; key-extraction screws per frame size; internal hex wrench.
- **Oil-seal replacement** (§14.3.2 / §15.3.2, p.272–273): needle-nose pliers, non-slip gloves, cotton cloth; pry the seal out slowly to protect the end bracket.

**Drive-parameter caution for service:** drive parameters (gains, inertia ratio, etc.) are **retentive** — they survive power-cycles and are stored in the drive's EEPROM, not the master. After maintenance that could disturb a drive, verify the full object dictionary is intact before re-enabling (the repo's `a6_gains` tool diffs drives). A drive swapped in as a spare ships with factory defaults and will not behave like its sibling until tuned to the same load.

## 7. Spare parts & replacement list

| Spare | Part | Interval | Notes |
|---|---|---|---|
| Cooling fan | A6 drive fan | 2–3 yr / 10–30k h | §14.2 |
| Electrolytic capacitors | drive board + bus | ~5 yr | §14.2 |
| Motor bearing | ASMK2 motor | 3–5 yr / 20–30k h | §14.2 |
| Motor oil seal | ASMK2 motor | 5,000 h | §14.2 |
| Encoder | 17-bit multi-turn | 3–5 yr / 20–30k h | §14.2 |
| Encoder battery | 3.6 V / 2500 mAh (battery box) | ~2 yr | §13.5 — swap cable/battery box |
| Suspension line | Dynamica DM20 2 mm, spliced, 65 m | on damage | pending supplier spec |
| Pre-charge relay/resistor | drive | 100k / 20k ops | §14.2 |

## 8. Maintenance log (template)

| Date | Component | Serial/ID | Action | Interval / reason | Performed by | Next due |
|---|---|---|---|---|---|---|
| | | | | | | |

## 9. Open items

1. **Suspension-line service data** — obtain Dynamica Ropes' inspection/retirement criteria for the DM20 2 mm line (Salgsordre 28529).
2. **Drive protection** — the A6 manual §11 requires a circuit breaker + EMI filter + contactor per drive; currently not installed.
3. **E-stop** — planned (TODO in §3); add its functional test interval.
4. **Vision hardware** — Hailo/camera (colleague's responsibility); add if it is to be serviced.
5. **Mast/anchoring structure** — structural inspection of masts/anchoring to be added.
6. **Service intervals for remaining components** — see §4.3 (under investigation).
