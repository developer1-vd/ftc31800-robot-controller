# FTC 31800 Robot Controller

Java-based robot control code for FTC Team 31800. This repository contains the robot application logic for autonomous and teleoperated matches, plus a bundled FTC SDK reference project used to build and deploy the code to the Robot Controller.

## Overview

This project is built for the FIRST Tech Challenge (FTC) control system and includes:

- Teleop control modes for driver operation
- Autonomous routines for alliance-side match play
- Vision integration using AprilTag detection
- Shooter, intake, and feeder subsystem logic
- Robot odometry and path planning support
- Mechanism tuning and telemetry utilities

## Repository structure

```text
ftc31800-robot-controller/
├── README.md
├── TEAMCODE_DOCUMENTATION.md
├── TeamBotTeleop.java
├── TeamBotTeleop_latest.java
├── BlueTeamBotAutoSimple.java
├── RedTeamBotAutoSimple.java
├── FlywheelArmStepControl.java
├── FtcRobotController-master/
│   ├── TeamCode/
│   ├── FtcRobotController/
│   └── gradle/
├── .gitignore
└── ...
```

## Key files

- `TeamBotTeleop.java` — main teleop control class for driver operation
- `TeamBotTeleop_latest.java` — newer development teleop version
- `BlueTeamBotAutoSimple.java` — simplified blue alliance autonomous routine
- `RedTeamBotAutoSimple.java` — simplified red alliance autonomous routine
- `TEAMCODE_DOCUMENTATION.md` — detailed codebase and subsystem documentation
- `FtcRobotController-master/` — FTC SDK project used as the deployment target for the robot code

## Features

### Teleop

The robot code supports driver control for:

- drivetrain movement via mecanum drive logic
- intake control
- shooter/flywheel control
- feeder and servo-based launch sequencing
- RGB indicator feedback for robot state
- AprilTag-based aiming and range feedback

### Autonomous

This repo includes autonomous work for both alliance colors and path variants, including simplified and path-planning-based routines.

### Vision and sensing

The codebase uses:

- AprilTag detection for localization and goal alignment
- telemetry for debugging and tuning
- hardware status checks for missing or failed components

## Build and run

### Prerequisites

- Android Studio
- FTC SDK / Robot Controller project compatible with the included `FtcRobotController-master` directory
- Java SDK configured for Android project builds
- FTC Robot Controller app installed on the control hub

### Getting started

1. Clone the repository.
2. Open the project in Android Studio.
3. Make sure the FTC SDK modules resolve correctly.
4. Sync Gradle and build the project.
5. Deploy the `TeamCode` module to the Robot Controller.
6. Enable the desired OpMode and run it from the Driver Station.

### Enabling an OpMode

Some OpModes are currently disabled with `@Disabled`. To test a mode:

1. Open the target OpMode class
2. Remove or comment out the `@Disabled` annotation
3. Rebuild and redeploy
4. Select the OpMode from the FTC Driver Station

## Documentation

- `TEAMCODE_DOCUMENTATION.md` contains a more detailed overview of robot subsystems, hardware configuration, autonomous variants, and development history.

## Notes

This codebase is intended for team development and ongoing robot tuning. Hardware names, motor assignments, and control logic may change as the robot evolves.

## Team

FTC Team 31800

## License

This project is provided for educational and competition use within FTC Team 31800. Please check repository permissions before reusing it outside the team context.
