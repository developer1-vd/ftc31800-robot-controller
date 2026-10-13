# TeamCode Documentation

## Overview

The TeamCode module contains the FTC Team 31800 robot control application for FTC 2025. The codebase implements both autonomous and teleop operation modes with support for multiple robot configurations and drive strategies.

**Last Updated:** January 17, 2026  
**Current Branch:** `meet3_auto` / `meet3_teleop`

---

## Project Structure

```
TeamCode/
├── src/main/
│   ├── AndroidManifest.xml
│   ├── java/org/firstinspires/ftc/teamcode/
│   │   ├── mechanisms/              # Robot subsystem implementations
│   │   │   ├── AprilTagWebcam.java
│   │   │   ├── BallDetector.java
│   │   │   ├── FeederLauncher.java
│   │   │   ├── FlyWheelTuner.java
│   │   │   ├── PinpointOdometry.java
│   │   │   ├── RGBIndicatorLight.java
│   │   │   └── RPM_per_dist.java
│   │   ├── pedroPathing/            # Pedro autonomous pathing implementations
│   │   │   ├── BlueTeamBotAuto.java
│   │   │   ├── BlueTeamBotAutoFarShort.java
│   │   │   ├── BlueTeamBotAutoNearShort.java
│   │   │   ├── CallbackExampleAuto.java
│   │   │   ├── Constants.java
│   │   │   ├── ExampleAuto.java
│   │   │   ├── ExampleSquarePathAuto.java
│   │   │   ├── RedTeamBotAuto.java
│   │   │   ├── RedTeamBotAutoFarLong.java
│   │   │   ├── RedTeamBotAutoFarShort.java
│   │   │   └── RedTeamBotAutoNearShort.java
│   │   ├── BallDetectionTest.java
│   │   ├── DriveTrain_Code.java
│   │   ├── TeamBotAuto.java
│   │   ├── TeamBotTeleop.java
│   │   ├── TeamBotTeleop_analytics.java
│   │   ├── TeamBotTeleop_latest.java
│   │   ├── BlueTeamBotAutoSimple.java
│   │   ├── RedTeamBotAutoSimple.java
│   │   └── readme.md
└── build/                          # Build artifacts (generated)
```

---

## Core OpModes

### 1. **Teleop Modes**

#### `TeamBotTeleop.java` (Primary)
- **Status:** Disabled (@Disabled annotation)
- **Purpose:** Main teleoperated control mode for the robot
- **Key Features:**
  - AprilTag detection and vision-based pose estimation
  - RGB indicator light for shoot-ready status
  - Dual feeder servo system (left and right)
  - Drivetrain control (4x 312 rpm goBILDA motors)
  - Intake motor control
  - High-speed flywheel shooter system
  - Motor initialization with failure tolerance
  - Shoot debounce system (3-frame debounce)

**Hardware Configuration:**
- **Drivetrain:** 4 DC motors (312 rpm goBILDA)
  - `frontLeft`, `frontRight`, `backLeft`, `backRight`
- **Intake:** 1 DC motor (312 rpm goBILDA)
- **Shooter:** 1 DcMotorEx (high-speed)
- **Feeder:** 1 DcMotorEx + 2 CRServos (continuous rotation)
- **Vision:** AprilTag Webcam + RGB Light

#### `TeamBotTeleop_latest.java`
- **Status:** Latest development version
- **Last Updated:** January 11, 2026
- **Changes:** Added ramp mechanics and flywheel improvements
- **Notable Updates:** 146 insertions, 51 deletions from previous version

#### `TeamBotTeleop_analytics.java`
- Specialized teleoperated mode with analytics tracking

---

### 2. **Autonomous Modes**

#### Basic Autonomous
- **`TeamBotAuto.java`:** General autonomous framework with encoder-based movement
- **`DriveTrain_Code.java`:** Basic drivetrain control (Disabled)

#### Simple Autonomous Variants
- **`BlueTeamBotAutoSimple.java`** (Disabled)
  - Blue alliance simplified autonomous
  - Flywheel target: 1800 RPM
  - Minimum speed: 1620 RPM (90% threshold)
  
- **`RedTeamBotAutoSimple.java`**
  - Red alliance simplified autonomous

#### Pedro Pathing Autonomous Suite
Advanced autonomous implementations using Pedro Pathing library for precise path planning and execution.

**Blue Alliance:**
- `BlueTeamBotAuto.java` - Base blue autonomous
- `BlueTeamBotAutoFarShort.java` - Far side, short path (276 lines)
- `BlueTeamBotAutoNearShort.java` - Near side, short path (379 lines)

**Red Alliance:**
- `RedTeamBotAuto.java` - Base red autonomous
- `RedTeamBotAutoFarLong.java` - Far side, long path
- `RedTeamBotAutoFarShort.java` - Far side, short path
- `RedTeamBotAutoNearShort.java` - Near side, short path

---

## Mechanisms (Subsystems)

### 1. **AprilTagWebcam** (`mechanisms/AprilTagWebcam.java`)
- **Purpose:** Vision-based pose estimation using AprilTag markers
- **Features:**
  - Custom AprilTag library configuration (Tags 20, 21, 22)
  - Pose estimation for range and bearing calculation
  - Telemetry reporting of detected tags
  - Range and bearing tolerance settings (default: 5 units)
  - Target range and bearing tracking
- **Hardware:** Webcam with AprilTag processor + VisionPortal
- **Typical Usage:** Robot positioning and alignment during teleop/auto

### 2. **FeederLauncher** (`mechanisms/FeederLauncher.java`)
- **Purpose:** Manages flywheel shooter and feeder system
- **Features:**
  - Dual feeder servo control (CRServo)
  - Flywheel velocity targeting with RPM control
  - State machine for shot sequencing: IDLE → SPIN_UP → LAUNCHING → LAUNCHED
  - Encoder-based RPM feedback (28 ticks/revolution)
  - Configurable feed time (1000 ms default)
  - Overshoot tracking for tuning
- **Hardware:**
  - 1 DcMotorEx (flywheel)
  - 1 CRServo (feeder)
- **Default Parameters:**
  - Flywheel target velocity: 2200 RPM
  - Minimum velocity threshold: 90% of target
  - Feed window: 1000 milliseconds

### 3. **FlyWheelTuner** (`mechanisms/FlyWheelTuner.java`)
- **Purpose:** Real-time tuning and testing of flywheel velocity
- Allows live RPM adjustment and performance measurement

### 4. **RGBIndicatorLight** (`mechanisms/RGBIndicatorLight.java`)
- **Purpose:** RGB LED strip for visual feedback
- **Usage:** Indicates robot state (e.g., ready to shoot when green)
- Provides visual feedback during teleoperated control

### 5. **PinpointOdometry** (`mechanisms/PinpointOdometry.java`)
- **Purpose:** Precise position tracking using Pinpoint odometry
- Enables dead reckoning and autonomous path planning

### 6. **RPM_per_dist** (`mechanisms/RPM_per_dist.java`)
- **Purpose:** Velocity/distance mapping utility
- Converts RPM to distance metrics for motion control

### 7. **BallDetector** (`mechanisms/BallDetector.java`)
- **Purpose:** Camera-based detection of yellow, blue, and red plastic (wiffle-style) balls
- **Features:**
  - Uses the SDK `ColorBlobLocatorProcessor` with the built-in `YELLOW` / `BLUE` / `RED` color ranges
  - Fills in the ball holes (blur + closing) so each ball is one round blob
  - Filters blobs by area and circularity, fits a circle to each ball
  - Estimates bearing (deg, + = left, same sign as AprilTag bearing) and distance (in)
  - `getClosestBall()` / `getClosestBall(color)` helpers for aiming and intake logic
- **Hardware:** "Webcam 1" at 640x480
- **Usage:**
  - Standalone: `ballDetector.init(hardwareMap, telemetry)`
  - Sharing the camera with AprilTags: `ballDetector.buildProcessors(telemetry)` then
    `aprilTagWebcam.init(hardwareMap, telemetry, ballDetector.getProcessors())`
  - Call `ballDetector.update()` once per loop
- **Tuning:** Distance depends on the camera focal length (default 822 px, typical for a C270).
  Put a ball at a known distance `D`, read its radius `r` in `BallDetectionTest`, then call
  `setFocalLengthPx(D * 2 * r / 2.9)` (2.9 in is the default ball diameter; change it with
  `setBallDiameter()` if your balls measure differently). Adjust `setContourAreaRange` / `setMinCircularity` if
  you see false detections.

---

## Recent Development History

### Latest Changes (January 11, 2026)
- **Commit:** f900e85 - "meet 3 teleop latest"
- **Changes:** Added ramp mechanism and flywheel improvements to `TeamBotTeleop_latest.java`
- **Impact:** 146 insertions, 51 deletions

### Autonomous Development (January 10, 2026)
- **Commit:** 8f93ab3 - "Pushing BlueTeamBot Autonomous Code"
- **New Files:**
  - `BlueTeamBotAutoFarShort.java` (276 lines)
  - `BlueTeamBotAutoNearShort.java` (379 lines)
- **Updates:**
  - `RedTeamBotAutoFarLong.java` minor updates

### Key Historical Milestones
- **Jan 11:** Meet 3 teleop latest (ramp & flywheel)
- **Jan 10:** Blue autonomous code addition
- **Jan 10:** Red near short path working
- **Jan 9:** Teleop & auto changes for meet 3
- **Dec 28:** Motor connection changes
- **Dec 23:** Meet 2 final fixes branch
- **Dec 19:** AprilTag RGB feeder and Pinpoint mechanism updates
- **Dec 16:** FeederLauncher addition
- **Dec 14:** AprilTag and camera calibration

---

## Hardware Specification

### Motor Configuration
- **Drivetrain:** 4× 312 RPM goBILDA DC Motors
  - Front Left, Front Right, Back Left, Back Right
  - Standard H-drive setup
- **Intake:** 312 RPM goBILDA DC Motor
- **Shooter Flywheel:** High-speed DcMotorEx
  - Target RPM: 1500-2200 (configurable)
  - Live tuning support
- **Feeder Motor:** DcMotorEx (312 rpm goBILDA)
- **Feeder Servos:** 2× Continuous Rotation Servos (dual push mechanism)

### Vision & Sensing
- **Webcam:** AprilTag detection capable
- **AprilTag Markers:** Tags 20, 21, 22 (0.166m size)
- **Odometry:** Pinpoint odometry system
- **Indicator Light:** RGB LED strip

### Power & Control
- Rev Robotics control hub
- Motor controllers: Rev Expansion Hub

---

## OpMode Status Reference

| OpMode | Type | Status | Branch | Purpose |
|--------|------|--------|--------|---------|
| TeamBotTeleop | TeleOp | Disabled | meet3_teleop | Primary teleop control |
| TeamBotTeleop_latest | TeleOp | Active | meet3_teleop | Latest telop with ramp |
| TeamBotAuto | Auto | Enabled | meet3_auto | Basic autonomous |
| BlueTeamBotAuto | Pedro | Enabled | meet3_auto | Blue alliance Pedro pathfinding |
| BlueTeamBotAutoFarShort | Pedro | Enabled | meet3_auto | Blue far, short path |
| BlueTeamBotAutoNearShort | Pedro | Enabled | meet3_auto | Blue near, short path |
| RedTeamBotAuto | Pedro | Enabled | meet3_auto | Red alliance Pedro pathfinding |
| RedTeamBotAutoFarShort | Pedro | Enabled | meet3_auto | Red far, short path |
| BlueTeamBotAutoSimple | Auto | Disabled | - | Simplified blue autonomous |
| RedTeamBotAutoSimple | Auto | Enabled | - | Simplified red autonomous |
| BallDetectionTest | TeleOp | Enabled | - | Ball detection telemetry + auto-aim (LB any, X blue, B red, Y yellow) |

---

## Key Features & Capabilities

### Vision System
- AprilTag-based pose estimation
- Range and bearing calculation for alignment
- RGB indicator feedback based on robot state
- Tolerance-based shoot-ready detection
- Color-based ball detection (yellow / blue / red) with bearing and distance estimates

### Shooter System
- Dual-servo feeder with synchronized pushing
- Flywheel velocity control with encoder feedback
- State machine-driven shot sequencing
- Real-time RPM tuning capabilities
- Overshoot monitoring for performance optimization

### Autonomous Navigation
- Pedro Pathing library integration for complex path planning
- Multiple trajectory variants per alliance (far/near, short/long)
- Encoder-based movement validation
- AprilTag-based pose correction

### Intake System
- Single motor driven intake
- Coordination with feeder system

---

## Code Quality Notes

### Known Patterns
- Heavy use of motor initialization with `null` checks
- Debounce systems for sensor noise rejection
- State machine pattern in FeederLauncher
- Telemetry-first debug approach
- Configurable parameters for field tuning

### Build Configuration
- Uses Gradle-based Android build system
- Rev dependencies for robotics libraries
- Vision library integration (AprilTag processor)
- Pedro Pathing library for autonomous planning

---

## Future Development Considerations

1. **Performance Optimization:** Monitor flywheel overshoot patterns for tuning
2. **Vision Integration:** Expand AprilTag usage for autonomous alignment
3. **Mechanism Refinement:** Further feeder servo synchronization tuning
4. **Autonomous Expansion:** Additional path variants based on field changes
5. **Testing:** Comprehensive range and bearing tolerance validation

---

## Build & Run

See root [README.md](README.md) for build instructions using Gradle.

To enable an OpMode:
1. Locate the desired OpMode class
2. Remove or comment out the `@Disabled` annotation
3. Rebuild and deploy to Driver Station

---

## Support & Maintenance

- **Team:** FTC Team 31800
- **Repository:** https://github.com/teamftc31800/ftc31800-robot-controller
- **Build Tool:** Gradle with Android SDK
- **Primary Branches:** `meet3_auto`, `meet3_teleop`
