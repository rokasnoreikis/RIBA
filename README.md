# RIBA

Robot control project using an STM32 NUCLEO-F767ZI microcontroller and a Python GUI communicating over Ethernet/UDP.

## Project Versions and Git Branches

### `main` — Original RobotArm

Contains the original working three-motor robot-control implementation.

### `feature/antenna-tracker-2axis` — Satellite Antenna Tracker

Development branch for converting the original robot into a two-axis satellite antenna tracker.

- M1: Azimuth (pan)
- M2: Elevation (tilt)
- M3: Removed

The original three-motor implementation remains available on `main`.

Both branches belong to the same Git repository. Git branches are separate lines of development, not separate project directories.

## Two-Axis Antenna Tracker

### Hardware

- Controller: STM32 NUCLEO-F767ZI
- PC: Ubuntu Linux
- GUI: Python / PyQt6
- Communication: Ethernet / UDP
- NUCLEO IP: `192.168.10.2`
- PC IP: `192.168.10.1`
- NUCLEO UDP port: `5001`
- PC UDP port: `5000`

### Source Files

- STM32 firmware: `Nucleo_F767/Nucleo_F767ZI_LAN_V1/Core/Src/main.c`
- Ubuntu GUI: `PC_GUI/antenna_tracker_gui.py`

### Motor Calibration

Before operating the antenna tracker, manually establish the zero reference:

- M1 = 0: Antenna pointing toward magnetic north.
- M2 = 0: Antenna horizontal, elevation 0°.

Press **ZERO SET** to reset the motor step counters.

Calibration measured on 2026-11-10:

| Axis | Positive angular direction | Position at +90° | Steps per degree |
|---|---|---:|---:|
| M1 — Azimuth | Clockwise, toward east | −2024 | 22.4889 |
| M2 — Elevation | Upward, toward vertical | −4669 | 51.8778 |

The motor counters decrease as these angles increase.

### GO TO START

GO TO START moves the motors to these absolute positions:

- M1 = −2024 steps (east)
- M2 = −4669 steps (vertical)

GO TO START is not the same as ZERO SET.

### Motor Control

The GUI supports:

- Ethernet connection
- Manual motor jogging
- ZERO SET
- GO TO START
- STOP
- M1/M2 step-position display

The M2 mechanical `break` is controlled by the STM32 firmware. Its release delay is 100 ms.

The positions are calculated from commanded motor steps, not measured by encoders. Lost steps can cause positioning errors.

### Verified Functionality

Hardware tested on 2026-11-10:

- STM32 firmware compiled successfully.
- Firmware programmed using ST-LINK.
- UDP communication with the Ubuntu GUI works.
- GO TO START reaches the calibrated azimuth and elevation positions.
- M2 mechanical `break` engages after movement.

Automatic satellite tracking and full mechanical travel limits have not yet been validated.

## Git Workflow

### Check the current branch

```bash
git status
git branch --show-current
```

### Switch to the antenna tracker

```bash
git switch feature/antenna-tracker-2axis
```

### Switch back to the original RobotArm

```bash
git switch main
```

Switch branches only after committing or safely setting aside local changes.

### Save changes

```bash
git add <file>
git commit -m "Describe changes"
git push
```

### Synchronize between Windows and Ubuntu

Both computers use the same GitHub repository:

`https://github.com/rokasnoreikis/RIBA.git`

After pushing changes from one computer, run the following on the other:

```bash
git pull --ff-only
```

This updates the current branch without creating an automatic merge commit. It will stop if a fast-forward is not possible.

### Create a new development branch

```bash
git switch -c feature/new-feature-name
git push -u origin feature/new-feature-name
```

A new branch starts from the currently checked-out commit. Commits made on the new branch do not automatically change `main`.

## Future Development

- Automatic satellite azimuth/elevation calculation using orbital elements
- Magnetic north to true north conversion
- Continuous two-axis tracking
- Mechanical travel limits and safety improvements
- Radio CAT control for Doppler compensation