# FTC 31800 Robot Controller

Java-based robot control code for FTC Team 31800, for the **2026-2027 BIOBUZZ** season. This repository contains the robot application logic for autonomous and teleoperated matches, plus the FTC SDK project (**SDK 12.0**) used to build and deploy the code to the Robot Controller.

[![Build robot APK](https://github.com/developer1-vd/ftc31800-robot-controller/actions/workflows/build-apk.yml/badge.svg)](https://github.com/developer1-vd/ftc31800-robot-controller/actions/workflows/build-apk.yml)

## Overview

This project is built for the FIRST Tech Challenge (FTC) control system and includes:

- Teleop control modes for driver operation
- BIOBUZZ autonomous routines for both alliances
- Camera vision: POLLEN / NECTAR ball detection and HIVE CELL AprilTag clusters
- Shooter, intake, and feeder subsystem logic
- Robot odometry (goBILDA Pinpoint) and path planning support (Pedro Pathing)
- Mechanism tuning and telemetry utilities
- One-command APK builds, locally and on GitHub

## BIOBUZZ at a glance

- **Scoring elements:** yellow **POLLEN** (2.8 in) and red/blue **NECTAR** (3.6 in) wiffle-style balls.
- **HIVE:** in the field center; LAUNCH into your alliance's upward-facing **CELL** to TIP the HIVE (20 pts). Each CELL has an AprilTag cluster underneath (red 30-37, blue 38-45).
- **AUTO (30 s):** LEAVE (3 pts, not touching the perimeter wall), PARK in your LOADING ZONE (5 pts), HIVE TIPS (20 pts). Every robot starts with 4 pre-loaded POLLEN.

## Repository structure

```text
ftc31800-robot-controller/
├── README.md
├── TEAMCODE_DOCUMENTATION.md
├── build-apk.sh / build-apk.bat      # build the robot APK with one command
├── .github/workflows/build-apk.yml   # GitHub builds the APK on every push
├── TeamBotTeleop.java                # older copies kept for reference (not built)
├── TeamBotTeleop_latest.java
├── BlueTeamBotAutoSimple.java
├── RedTeamBotAutoSimple.java
├── FlywheelArmStepControl.java
├── FtcRobotController-master/        # the Android Studio project (SDK 12.0)
│   ├── TeamCode/                     # our robot code
│   ├── FtcRobotController/
│   └── gradle/
├── .gitignore
└── ...
```

## Key files

All robot code that gets built is under `FtcRobotController-master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`.

| OpMode | Type | What it does |
|---|---|---|
| **BIOBUZZ Auto RED** / **BIOBUZZ Auto BLUE** | Auto | Competition autos: leave the wall, LAUNCH the 4 preloads into our HIVE CELL, collect and LAUNCH more POLLEN, finish off the wall (or PARK) |
| `TeamBotTeleop_latest` | TeleOp | Main driver control; HIVE CELL shoot-ready light; ball auto-aim (gamepad1 LB / X / B / Y) |
| `BallSpeakerAuto` | Auto | Practice version of the BIOBUZZ auto (no preloads); works with just a camera too |
| `BallDetectionTest` | TeleOp | Ball detection telemetry and tuning |
| `TeamBotTeleop_Phone (practice)` | TeleOp | Same as `TeamBotTeleop_latest`, plus phone touchscreen controls. **Practice only** |
| `LimelightTest` | TeleOp | Limelight 3A connection and pipeline check |

- `mechanisms/BallDetector.java` — POLLEN / NECTAR detection, distance and bearing; spots robots by their alliance-color signs
- `mechanisms/AprilTagWebcam.java` — HIVE CELL AprilTag clusters (SDK 12)
- `mechanisms/LimelightVision.java` — **Limelight 3A** support: used instead of the webcam when a Limelight named `limelight` is configured (pipeline setup in `TEAMCODE_DOCUMENTATION.md`)
- `mechanisms/BallCollector.java`, `FeederLauncher.java`, `PinpointOdometry.java`, `RGBIndicatorLight.java` — subsystems
- `pedroPathing/` — Pedro Pathing setup; the Near/Far Short autos in it are **last season's (DECODE)** and are disabled
- `TEAMCODE_DOCUMENTATION.md` — detailed codebase, subsystem, and tuning documentation
- `FtcRobotController-master/README.md` — FIRST's SDK release notes

## Features

### Teleop

The robot code supports driver control for:

- drivetrain movement via mecanum drive logic
- intake control
- shooter/flywheel control
- feeder and servo-based launch sequencing
- RGB indicator feedback for robot state (green = in the HIVE CELL shoot window)
- camera ball auto-aim and spoken ball announcements (gamepad1 BACK mutes)

### Autonomous

**BIOBUZZ Auto RED / BLUE** pick the alliance from the Driver Station list. They use the camera to find balls and our HIVE CELL, the Pinpoint to know where the robot is, and avoid other robots and the field walls. Settings (speeds, shoot zone, park spot, start position) are constants at the top of `BallSpeakerAuto.java`.

**Before competing:** re-tune the shoot zone and flywheel RPM for the HIVE (they are still last season's values), set `START_FIELD_*`, and optionally set the LOADING ZONE park spot (`PARK_IN_LOADING_ZONE`, `PARK_FIELD_*`).

### Vision and sensing

The codebase uses:

- color-blob ball detection (POLLEN / NECTAR) with the team's Logitech C270 calibration, or a Limelight 3A's color pipelines
- AprilTag cluster detection for HIVE CELL aiming
- Pinpoint odometry for returning, parking, and virtual field walls
- telemetry for debugging and tuning
- hardware status checks for missing or failed components

## Build and run

### Prerequisites

- Android Studio (or just JDK 17+ and the Android SDK for the build scripts)
- REV Control Hub with the FTC Robot Controller app (SDK 12.0)
- Driver Station on SDK 12.0

### Build the APK

**One command (local):**

```bash
./build-apk.sh          # macOS / Linux
build-apk.bat           # Windows
```

The APK lands in `apk/TeamCode-debug.apk`. Add `clean` for a fresh build. Install it on the Control Hub with a laptop on the robot's WiFi:

```bash
adb connect 192.168.43.1:5555
adb install -r apk/TeamCode-debug.apk
```

**On GitHub (no setup needed):** every push builds the APK automatically. Open the **Actions** tab → **Build robot APK** → the latest run → download **TeamCode-debug-apk** under *Artifacts*. You can also start a build by hand with **Run workflow**.

**Android Studio:** open `FtcRobotController-master`, let Gradle sync, and press **Run** with the Control Hub connected (USB or `adb connect 192.168.43.1:5555`).

### Getting started

1. Clone the repository.
2. Build the APK (above) or open the project in Android Studio.
3. Deploy the `TeamCode` app to the Control Hub.
4. Make sure the robot configuration names match the code (e.g. `Webcam 1` or `limelight`, `left_front_drive`, `intake`, `launcher`, `pinpoint`).
5. Select the OpMode on the Driver Station and run it. For AUTO, enable the 30-second timer.

### Driving from a phone

There are two ways, and you can use both:

**1. Phone as the Driver Station (allowed in matches).** FIRST's Driver Station app runs on Android phones as well as the REV Driver Hub (the official one). Only one Driver Station may be connected and powered on at a time, so the phone is a replacement or a spare for the Driver Hub, not a second one running together.

1. Install the **FTC Driver Station** app, version **12.0** to match the robot (from FIRST's [FtcDriverStation releases](https://github.com/FIRST-Tech-Challenge/FtcDriverStation/releases)).
2. Phone settings: **Airplane Mode on, Wi-Fi on, Bluetooth off**; forget other Wi-Fi networks (rule R711). Name it **`31800-DS`** (R705).
3. Plug the gamepad(s) into the phone with a **USB OTG adapter** (a powered USB hub for two gamepads).
4. In the app: *Settings → Pair with Robot Controller* → choose the Control Hub's network. Drive exactly as with the Driver Hub.

**2. Phone touchscreen controls (practice / demos only).** On-screen joysticks and buttons that work at the same time as the real gamepads. Not allowed in matches: only the Driver Station may control the robot, and no other device may be on the robot's Wi-Fi during a match.

1. On the Driver Station, start **`TeamBotTeleop_Phone (practice)`**.
2. Connect the phone to the **Control Hub's Wi-Fi** (its name and password are set in the REV Hardware Client).
3. Open **http://192.168.43.1:8090** in the phone's browser and hold it sideways. Pick **Driver 1** or **Driver 2** to control gamepad 1 or gamepad 2.
4. If the page closes or Wi-Fi drops, the phone's input lets go within 0.3 s; the real gamepads keep working.

The Limelight stays plugged into the Control Hub, which stays the Robot Controller in both cases.

### Enabling an OpMode

Some OpModes are disabled with `@Disabled` (including last season's DECODE autos). To test one:

1. Open the target OpMode class
2. Remove or comment out the `@Disabled` annotation
3. Rebuild and redeploy
4. Select the OpMode from the FTC Driver Station

## Documentation

- `TEAMCODE_DOCUMENTATION.md` contains a more detailed overview of robot subsystems, hardware configuration, autonomous variants, BIOBUZZ notes, and development history.

## Notes

This codebase is intended for team development and ongoing robot tuning. Hardware names, motor assignments, and control logic may change as the robot evolves.

## Team

FTC Team 31800

## License

This project is provided for educational and competition use within FTC Team 31800. Please check repository permissions before reusing it outside the team context.
