# TeamCode Documentation

## Overview

The TeamCode module contains the FTC Team 31800 robot control application for FTC 2025. The codebase implements both autonomous and teleop operation modes with support for multiple robot configurations and drive strategies.

**Last Updated:** January 17, 2026  
**Current Branch:** `meet3_auto` / `meet3_teleop`

> **2026-27 season (BIOBUZZ) / FTC SDK 12.0 — October 2026.** The project now builds on SDK 12.0.
> Updated for BIOBUZZ: `AprilTagWebcam` (HIVE CELL tag clusters), `BallDetector` (POLLEN /
> NECTAR sizes), `BallSpeakerAuto`, and the shoot-ready light in `TeamBotTeleop_latest`.
> **Last season (DECODE), now `@Disabled`:** the Pedro autos in `pedroPathing/` (field poses,
> paths, 2-volley shooting), `TeamBotAuto`, and the `*AutoSimple` autos. Use **BIOBUZZ Auto RED /
> BLUE** instead. Shoot RPM / range values are still the DECODE ones and need re-tuning for the
> HIVE. See "BIOBUZZ notes" below.
>
> **Building the APK:** `./build-apk.sh` (or `build-apk.bat`) at the repo root, or download it
> from GitHub Actions (**Build robot APK** workflow, artifact `TeamCode-debug-apk`).

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
- **Ball auto-aim (gamepad1, hold):** LB = closest ball, X = blue, B = red, Y = yellow.
  Overrides rotation only; the driver keeps forward/strafe. Uses `BallDetector` on the same
  camera as AprilTags. Closest ball is shown under `== BALLS ==` in telemetry.
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
  - SDK 12 BIOBUZZ game library: one 4-tag cluster under each HIVE CELL
    (RED SCORING 30-33, RED AUDIENCE 34-37, BLUE AUDIENCE 38-41, BLUE SCORING 42-45).
    `getCellCluster(red)` returns the closest visible CELL of that alliance; the cluster origin
    is the center of the CELL opening
  - SDK 12 split detections into `AprilTagSingleDetection` and `AprilTagClusterDetection`;
    `ftcPose` works on both
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
- **Purpose:** Camera-based detection of BIOBUZZ scoring elements: yellow POLLEN (2.8 in) and
  red/blue NECTAR (3.6 in) Gopher wiffle-style balls; distance uses each color's diameter and
  the team's C270 calibration (focal length 968.4 px)
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
- **Sound:** the Driver Station speaks the color ("red ball") once when a ball comes into
  view (2 s cooldown), and "got ball" when autonomous pickup finishes. Uses
  `telemetry.speak()` because the Control Hub has no speaker. On by default:
  set `SOUND_ENABLED = false` in an OpMode to start silent, or press gamepad1 **BACK** in
  `TeamBotTeleop_latest` / `BallDetectionTest` to toggle it during a run.
- **Camera-only robot:** with just a Control Hub, battery, and "Webcam 1" (no motors/servos),
  `BallDetectionTest` and `TeamBotTeleop_latest` still run: they detect balls and speak the color,
  including during INIT. For an Autonomous option, use `BallSpeakerAuto`. The Pedro autos need
  the drivetrain and won't start without it.
- **Ball color calibration (if the webcam detects no balls):** the SDK's built-in color ranges
  need very saturated colors and often match nothing on a real webcam under field lighting. Run
  **BallDetectionTest**, open the Camera Stream, hold a ball so it fills the white square in the
  center, and press gamepad1 **D-pad UP (yellow) / RIGHT (red) / LEFT (blue)**; **DOWN** resets.
  Telemetry shows the center color (Y/Cr/Cb) and which balls it matches. Saved to
  `/sdcard/FIRST/biobuzz_ball_colors.txt` on the Control Hub and used by every OpMode (restart the
  OpMode to apply). Calibrate under the lighting you'll compete in.
- **Filters:** area 400-250000 px (ignores specks; keeps a NECTAR right in front of the camera),
  circularity ≥ 0.5 (the balls have holes).
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
- **AprilTag Markers:** BIOBUZZ HIVE CELL clusters, tags 30-45 (3.25 in, 36h11)
- **Odometry:** Pinpoint odometry system
- **Indicator Light:** RGB LED strip

### Power & Control
- Rev Robotics control hub
- Motor controllers: Rev Expansion Hub

### 8. **BallCollector** (`mechanisms/BallCollector.java`)
- **Purpose:** Autonomous ball pickup after the Pedro path finishes
- **Steps:** SEARCH (turn slowly until a ball is seen, 4 s limit) → APPROACH (drive at the ball,
  steering to keep it centered, intake on) → CAPTURE (drive straight 0.7 s once the ball is
  within 10 in) → DONE. Always stops with 1 s of auto left.
- **Used in:** `RedTeamBotAutoNearShort`, `BlueTeamBotAutoNearShort`, `RedTeamBotAutoFarShort`,
  `BlueTeamBotAutoFarShort` (new `COLLECT_BALL` path state after `END`). Turn off per auto with
  `COLLECT_BALL_AFTER_PATH = false`.
- **Return and shoot:** if a ball was captured, the auto drives straight back to `scorePose`,
  runs the same INTAKE → SHOOT_2 steps used for the preload (feed, shoot left, 2 s delay, shoot
  right), then drives back to `endPose` so LEAVE still counts. States: `COLLECT_BALL` →
  `RETURN_TO_SCORE` → `SHOOT_COLLECTED` → `LEAVE_AGAIN`. Pickup gives up with 10 s left
  (`RETURN_AND_SHOOT_MS`) to save time for this; if under 3 s remain (`LEAVE_TIME_MS`) the robot
  skips straight to leaving. Turn off with `SHOOT_COLLECTED_BALL = false`.
- **Safety:** if "Webcam 1" is missing, pickup is skipped instead of crashing the auto.
- **Tuning:** speeds, gains, and timeouts are constants at the top of the class.
  `INTAKE_IN_POWER` is 1.0 (= in, counterclockwise); if balls go out, flip `Intake.IN_IS_COUNTERCLOCKWISE`.
  Assumes camera and intake face the front of the robot.

### 14. **Intake** (`mechanisms/Intake.java`, test: `IntakeTest`)
- **Motor:** goBILDA Yellow Jacket **312 RPM** (19.2:1, **537.7 ticks/rev**, ~9.2 A stall) named
  `intake`. Used by `TeamBotTeleop_latest` (**autorun**: runs IN from Play; gamepad2 Y = off/on,
  A = on, hold B = out; `INTAKE_AUTORUN` turns autorun off). **Competition mode** (on at Play,
  gamepad2 D-pad UP toggles it vs always-on; `INTAKE_COMPETITION_MODE`): the intake only spins
  while the camera sees a POLLEN or our NECTAR within 30 in and 25° of straight ahead, plus 1.5 s
  after it leaves the view; hold A to run it anyway. No ball camera → always on. With ONLY a
  Limelight it watches for POLLEN, so the shoot-ready light needs Webcam 1 (or competition mode
  off). Used by all BallSpeakerAuto-based autos,
  the path auto and `ShotSequencer` (feeding the launchers).
- **Direction:** `run(+1)` = in, `run(-1)` = out. "In" is **counterclockwise** looking at the end
  of the motor shaft, set ONCE by `Intake.IN_IS_COUNTERCLOCKWISE` (true → motor REVERSE; a goBILDA
  motor on FORWARD turns clockwise with positive power). If IntakeTest's A spins it the wrong
  way, flip that one setting.
- **Drive:** plain motor power by default (`run(1)` = `setPower(1)`, the same as the old teleop), so
  it always turns. `SPEED_CONTROL = true` instead holds `IN_RPM`/`OUT_RPM` (300) with the hub's
  velocity control; without a working encoder it drops back to power by itself.
- **Name:** looks for `intake` (then `intakeMotor`, `intake_motor`, `Intake`). If none is found,
  telemetry says so and lists the motor names that ARE in the configuration.
- **Jam clearing (needs the encoder cable):** only after the encoder has been seen moving. Pulling
  in at < 25% speed while drawing > 3 A for 0.3 s = jammed → reverse 0.25 s → pull in again.
- **IntakeTest** (also in TEST MODE): **autoruns IN from Play**; Y = off/on, A = on, hold B = out,
  D-pad speed (gamepad1 or gamepad2), **X = RAW**
  full power straight to the motor (skips the Intake class: if X doesn't turn it, it's wiring or
  configuration). Shows RPM, current, jams, encoder status and any problem.

### 13. **The auto's "brain"** (`BallSpeakerAuto` SMART_ flags, `BallChooser`, `ShotPlanner`)
Used by every BallSpeakerAuto-based mode (BIOBUZZ autos, simple autos, practice autos, the teleop
autopilot). Telemetry shows the current decision as **Brain: ...**. Each part can be turned off:
- **SMART_SEARCH:** turns toward where it last saw a ball / the HIVE (using the Pinpoint heading)
  instead of always turning left; after looking all the way around (360°, or 12 s without a
  Pinpoint) it drives 24 in to a new spot (toward the field middle if `START_FIELD_*` is set) and
  looks again; after 3 robot avoids within 15 s it moves somewhere less crowded.
- **SMART_BALL_CHOICE (`BallChooser`):** cost = distance + 0.25 in per degree of turning − 8 in per
  neighboring ball (within 14 in). Skips the opponent's NECTAR, balls against a wall (too close to
  reach with the wall margin), balls straight behind a robot sign, balls it gave up on (10 s), and
  balls there isn't time for.
- **SMART_SHOOTING (`ShotPlanner`):** shoots from anywhere 24-60 in from the HIVE CELL with the
  flywheel RPM interpolated from `ShotPlanner.RPM_TABLE` (**re-tune the table on the robot**),
  and only once aimed and holding still for 0.3 s. The path auto also sets RPM from distance.
- **SMART_TIME:** only chases a ball if there's time to collect it, then find/line up/shoot and
  get home; goes to shoot what it holds when time gets short; shoots only as many balls as fit;
  gives up on a ball it can't get within 7 s, and on shooting after 2 failed HIVE searches; heads
  home based on the real distance home (minimum 2.5 s) instead of a fixed 6 s.

### 12. **More autonomous modes** and the TeleOp autopilot
- **TeleOp AUTOPILOT** (`TeamBotTeleop_latest`, also in the phone TeleOp): gamepad1 **START** hands
  the robot to the camera auto (BallSpeakerAuto with no time limit: find balls, collect, aim at the
  HIVE, shoot, avoid robots/walls). **START again or moving any stick** gives control back; the
  cameras are handed back and every motor is stopped first. `AUTOPILOT_RED_ALLIANCE` picks which
  HIVE it shoots into.
- **BIOBUZZ Simple Auto RED/BLUE:** BallSpeakerAuto with 4 preloads, leave-the-wall first and
  `collectBalls() = false`: leave → find our HIVE CELL → shoot → home/park → stop. If the HIVE isn't
  found, or there's no launcher, it just parks.
- **BIOBUZZ Path Auto RED/BLUE** (`pedroPathing/BiobuzzPathAuto*.java`): Pedro paths start → SHOOT
  pose → camera fine-aim (turn in place to the CELL tag) → shoot 4 → COLLECT start → slow sweep to
  COLLECT end with the intake on → SHOOT → shoot 2 → PARK; goes to PARK with 5 s left. The poses in
  the Red/Blue files are placeholders: set them and `POSES_SET = true`; until then it only drives
  14 in forward (LEAVE).
- **Practice Auto (endless):** BallSpeakerAuto with no time limit.
- **ShotSequencer** (`mechanisms/ShotSequencer.java`): the shared feed → left → right (→ feed → left
  → right) shooting sequence used by BallSpeakerAuto (and so every BIOBUZZ auto) and the path auto.

### 11. **TEST MODE (everything)** (`TestMode.java`) and turn-to-look (`mechanisms/LookAroundScanner.java`)
- **TEST MODE:** one TeleOp that hosts every other (iterative) OpMode: all BIOBUZZ TeleOps, autos
  and tests, the older TeleOps (TeamBotTeleop, TeamBotTeleop_analytics), last season's DECODE autos
  (marked OLD) and the Pedro examples. gamepad1 D-pad UP/DOWN + A to pick (Play starts the first
  pick). **Y = RUN ALL**: every mode in a row (autos 30 s, TeleOps/tests 30 s, practice autos 45 s),
  then a per-mode summary (ran / skipped / crashed / couldn't start). **X = RUN ALL BIOBUZZ only**
  (skips the DECODE autos, which drive last season's paths). Click the **left stick** = skip to the
  next mode; **both sticks** = stop everything (sets every motor and CR servo to 0) and back to the
  menu. Crashes are caught. Not included: the two LinearOpModes (DriveTrain_Code,
  RobotAutoDriveByEncoder_Linear_Modified), which can't be hosted. Practice only.
- **Turn to look:** with no ball in view, the auto turns in place in steps (turn 0.35 s, hold still
  0.45 s so the camera image is sharp) to search, and does the same to find the HIVE. In
  `TeamBotTeleop_latest` and `BallDetectionTest`, holding an aim button (LB/X/B/Y) with no matching
  ball in view does the same, unless the driver is turning with the right stick.
  (`LOOK_AROUND_FOR_BALLS`, `LOOK_AROUND_WHEN_AIMING` turn it off.) The auto now waits 6 s without
  seeing a ball (was 3 s) before going to shoot what it holds, so it has time to look around.

### 10. **PhoneGamepadServer** (`mechanisms/PhoneGamepadServer.java`, `assets/phone_gamepad.html`)
- **Purpose:** practice/demo phone touchscreen controls. A tiny web server on the Control Hub
  (port **8090**) serves a control page (two joysticks, D-pad, A/B/X/Y, bumpers, triggers,
  BACK/START, Driver 1 / Driver 2 switch). The phone sends its state ~20 times a second.
- **Used by:** `TeamBotTeleop_Phone (practice)` = `TeamBotTeleop_latest` + phone input. The SDK
  refreshes `gamepad1/2` from the Driver Station before every loop; the phone input is blended in
  each loop (sticks/triggers: whichever is pushed further; buttons: either pressed).
- **Safety:** no update for 300 ms (page closed, Wi-Fi lost, screen off) = phone input released.
- **Not for matches** (only the Driver Station may control the robot; no extra devices on the
  robot Wi-Fi during a match). For a phone that IS allowed in matches, run the FTC Driver Station
  app on it (see the README, "Driving from a phone").

### 9. **LimelightVision** (`mechanisms/LimelightVision.java`)
- **Purpose:** Limelight 3A smart camera support (USB name "Limelight / Raspberry Pi Compute
  Module 4"). If a Limelight named **`limelight`** is in the robot configuration,
  `BallSpeakerAuto`, the BIOBUZZ autos, `TeamBotTeleop_latest`, and `BallDetectionTest` use it
  instead of "Webcam 1"; otherwise they use the webcam as before.
- **Pipelines** (set up in the Limelight web interface, `http://limelight.local:5801`):
  | # | Type | Used for |
  |---|---|---|
  | 0 | AprilTag / Fiducial, 36h11, 82.55 mm (3.25 in) tags | HIVE CELL aiming (tags 30-37 red, 38-45 blue) |
  | 1 | Color, yellow | POLLEN |
  | 2 | Color, red | red NECTAR |
  | 3 | Color, blue | blue NECTAR |
  Enable multiple targets in the color pipelines and set a minimum area.
- **Both cameras (Limelight + `Webcam 1`):** each camera does one job, all the time. The Limelight
  stays on pipeline 0 and only watches the HIVE AprilTags; the Logitech C270 finds all the balls
  (yellow POLLEN, red and blue NECTAR) with `BallDetector`, including camera robot avoidance. Only
  pipeline 0 needs setting up on the Limelight in this mode. Used by `BallSpeakerAuto`, the
  BIOBUZZ autos and `TeamBotTeleop_latest`; `BallDetectionTest` uses the webcam for balls.
- **Limelight only — one pipeline at a time:** the auto uses the HIVE pipeline while finding / lining up on the
  goal and the POLLEN pipeline otherwise (`LIMELIGHT_BALL_COLOR`). The teleop uses the HIVE
  pipeline for the shoot-ready light, and the ball pipeline matching the aim button while
  gamepad1 X (blue) / B (red) / Y or LB (yellow) is held.
- **Ball distance** comes from the target area (assumes a 640x480 image, ~54.5° wide).
  Camera-based robot avoidance is webcam-only; the distance sensor and bump detection still work.
- `CellTarget` (range / bearing to our HIVE CELL) is what the OpModes use, from either camera.
- **"Detects nothing":** a new Limelight's pipelines aren't set up for this code. In the web page,
  for each pipeline: choose it, set the Pipeline Type (0 = Fiducial Markers / 36h11 / 82.55 mm;
  1-3 = Color), for color use the Thresholding color picker on a real ball so only the ball
  lights up, then **Save**. Only HIVE tags **30-45** count; other tag IDs are ignored. In
  `TeamBotTeleop_latest` balls are only looked for while gamepad1 X/B/Y/LB is held (the rest of
  the time the Limelight watches the HIVE). Distance to the HIVE uses 3D if "Full 3D" is on,
  otherwise the tag's size.
- **Troubleshooting:** run the **LimelightTest** TeleOp. It shows whether a Limelight 3A is in the
  active configuration (under any name), whether it is connected and sending data (FPS,
  temperature, pipeline), warns if a pipeline is the wrong **type** (e.g. pipeline 0 not AprilTag),
  and lists what it sees (every tag ID, labeled HIVE or ignored; color targets with size); the
  D-pad switches pipelines 0-3. The Limelight must
  be plugged into the **Control Hub** (not a laptop) and added with Configure Robot → Scan.

---

## OpMode Status Reference

| OpMode | Type | Status | Branch | Purpose |
|--------|------|--------|--------|---------|
| TeamBotTeleop | TeleOp | Disabled | meet3_teleop | Primary teleop control |
| TeamBotTeleop_latest | TeleOp | Active | meet3_teleop | Latest telop with ramp |
| TeamBotAuto | Auto | Disabled (DECODE) | meet3_auto | Basic autonomous |
| BlueTeamBotAuto | Pedro | Enabled | meet3_auto | Blue alliance Pedro pathfinding |
| BlueTeamBotAutoFarShort | Pedro | Disabled (DECODE) | meet3_auto | Blue far, short path |
| BlueTeamBotAutoNearShort | Pedro | Disabled (DECODE) | meet3_auto | Blue near, short path |
| RedTeamBotAuto | Pedro | Enabled | meet3_auto | Red alliance Pedro pathfinding |
| RedTeamBotAutoFarShort | Pedro | Disabled (DECODE) | meet3_auto | Red far, short path |
| BlueTeamBotAutoSimple | Auto | Disabled | - | Simplified blue autonomous |
| RedTeamBotAutoSimple | Auto | Enabled | - | Simplified red autonomous |
| BIOBUZZ Auto RED / BLUE (`BiobuzzAutoRed`, `BiobuzzAutoBlue`) | Auto | Enabled | - | BIOBUZZ competition autos: `BallSpeakerAuto` with the alliance set, 4 preloaded POLLEN shot first, and driving off the wall at the start (LEAVE) |
| BallSpeakerAuto | Auto | Enabled | - | Speaks detected ball colors; with drive motors, drives to the closest ball; with an `intake` motor, picks it up (up to 3); with the launcher, finds our HIVE CELL's tag cluster (set `RED_ALLIANCE`), lines up and LAUNCHES; only collects POLLEN + our NECTAR; with `pinpoint` odometry, drives back to 12 in in front of the start (off the wall, so LEAVE counts) or, at the end, to a LOADING ZONE park spot (`PARK_IN_LOADING_ZONE`, `PARK_FIELD_*`). Repeats collect → shoot → return until 30 s runs out, heading home for the last time with 6 s left. Avoids robots: optional `frontDistance` sensor (< 14 in), camera (large red/blue alliance-color blob ahead, ignored when a HIVE CELL is visible or in the top 30% of the image), or Pinpoint bump detection → wait, back up, turn away, resume. Avoids field walls with Pinpoint virtual walls (set `START_FIELD_X/Y/HEADING_DEG` in Pedro 0-144 in coordinates): motion toward a wall within 12 in is removed; stuck 1.5 s → back off and turn. Camera-only OK |
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


---

## BIOBUZZ notes (2026-27, from Competition Manual TU04)

- **AUTO** is 30 s. Points in AUTO: LEAVE 3 (not touching the perimeter wall at the end of AUTO),
  PARK 5 (at least partially in your LOADING ZONE), HIVE TIP 20 (LAUNCH enough into the
  upward-facing CELL to tip the HIVE).
- **Scoring elements:** 40 yellow POLLEN (2.8 in), 8 red + 8 blue NECTAR (3.6 in). Each robot
  starts with 4 pre-loaded POLLEN; 4 POLLEN start in each FLOWER and in each GARDEN.
- **HIVE:** in the field center, frame pivots ~44 in above the tiles. Each alliance's HIVE has two
  CELLS (opening ~20 x 14 in, 12 in deep); the one facing up is the target. AprilTag clusters are
  on the CELL undersides. FIRST notes the HIVE tags move, so they are for aiming, not field
  localization.
- **LOADING ZONE:** ~23 x 11 in against the perimeter by your ALLIANCE AREA.
- **ROBOT SIGNS** must show alliance color, which is what the camera robot detection relies on.
