package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.mechanisms.RPM_per_dist;
import org.firstinspires.ftc.teamcode.mechanisms.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.mechanisms.BallDetector;
import org.firstinspires.ftc.teamcode.mechanisms.RGBIndicatorLight;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name="TeamBotTeleop_latest", group="Drive")
public class TeamBotTeleop_latest extends OpMode {

    // -----------------------------
    // AprilTag + RGB (single instance)
    // -----------------------------
    private final AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();
    private final RGBIndicatorLight light = new RGBIndicatorLight();

    private final RPM_per_dist distToRPM = new RPM_per_dist();

    // Ball detection (shares "Webcam 1" with AprilTags)
    private final BallDetector ballDetector = new BallDetector();

    // Ball auto-aim: proportional turn gain (power per degree), limits
    private static final double BALL_TURN_GAIN     = 0.02;
    private static final double BALL_MAX_AUTO_TURN = 0.4;
    private static final double BALL_AIM_TOLERANCE = 2.0;  // degrees

    // Debounce for shoot-ready indicator
    private int shootInRangeCount = 0;
    private static final int SHOOT_DEBOUNCE_FRAMES = 3;

    // Drivetrain motors (312 rpm goBILDA)
    private DcMotor frontLeft  = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft   = null;
    private DcMotor backRight  = null;

    // Intake (312 rpm goBILDA)
    private DcMotor intake = null;

    // Flywheel (high-speed shooter)
    private DcMotorEx flywheel = null;

    // Feeder motor (312 rpm goBILDA)
    private DcMotorEx feederMotor;

    // TWO feeder servos (dual feeder)
    private CRServo feederServoLeft  = null;
    private CRServo feederServoRight = null;

    private boolean hasFlywheel = false;
    private boolean hasFeederMotor = false;
    private boolean hasFeederServoLeft = false;
    private boolean hasFeederServoRight = false;

    private boolean flywheelActive = false;
    private boolean useManualRPM = false;   // true when X-preset (3150 RPM) is active
    private String manualPresetLabel = "";     // which preset is active (RT/LT)

    private boolean lastButtonState = false;

    // Convenience: did each motor initialize?
    private boolean hasFrontLeft  = false;
    private boolean hasFrontRight = false;
    private boolean hasBackLeft   = false;
    private boolean hasBackRight  = false;
    private boolean hasIntake     = false;

    private double largest_overshoot = 0;
    private static final double F                = 400.0;  // suggested feed speed
    private static final double FEEDER_HOLD_RPM  = 0.0;    // stop when not ready

    // Ticks per revolution (output shaft)
    private static final double FLYWHEEL_TPR = 28.0;
    private static final double FEEDER_TPR   = 537.7;

    private static final double FEEDER_RPM = 400;

    // Targets
    private static double FLYWHEEL_TARGET_RPM = 2400.;
    private static final double FLYWHEEL_TOLERANCE   = 200.0;

    // Debounce: require N consecutive "in-tolerance" reads before feeding
    private int inToleranceCount = 0;
    private int dpad_pressed = 0;
    private boolean dpadPressed = false;

    private static final int IN_TOLERANCE_REQUIRED = 1;

    private static final double targetDist = 36;
    private static final double targetDistTol = 6;

    private static final double targetBearing = -3;
    private static final double targetBearingTol = 7;

    public static double remainingDistIn = targetDist;

    // Standard servo (positional)
    private Servo armServo = null;
    private boolean hasArmServo = false;

    // Servo positions
    private static final double SERVO_HOME = 0.0;
    private static final double SERVO_ACTIVE = 0.1;
    private static final double SERVO_MIN = 0.0;
    private static final double SERVO_MAX = 1.0;
    private static final double SERVO_STEP = 0.1; // speed per loop

    // --- Arm servo angle mapping (EDIT to match your mechanism) ---
    private static final double ARM_MIN_DEG = 0.0;    // angle at SERVO_MIN
    private static final double ARM_MAX_DEG = 300.0;  // angle at SERVO_MAX

    // --- Arm servo bumper edge-detect (one step per press) ---
    private boolean lastRightBumper = false;
    private boolean lastLeftBumper  = false;

    // -----------------------------
    // ARM AUTO-AIM (distance -> angle)
    // -----------------------------
    private static final double ARM_DIST_NEAR_IN = 24.0;  // tune
    private static final double ARM_DIST_FAR_IN  = 60.0;  // tune

    private static final double ARM_DEG_NEAR = 60.0;      // tune
    private static final double ARM_DEG_FAR  = 140.0;     // tune

    private boolean armAutoAim = true;        // auto mode ON by default
    private boolean lastArmToggle = false;    // edge detect for toggle

//    private final FlyWheelTuner flywheeltuner = new FlyWheelTuner();

    // Launchers
//    private final FeederLauncher leftFeederLauncher  = new FeederLauncher();
//    private final FeederLauncher rightFeederLauncher = new FeederLauncher();

    @Override
    public void init() {
        // Motors
        frontLeft  = getMotor("left_front_drive");
        frontRight = getMotor("right_front_drive");
        backLeft   = getMotor("left_back_drive");
        backRight  = getMotor("right_back_drive");
        intake     = getMotor("intake");
        flywheel   = getMotorEx("launcher"); // needs DcMotorEx for velocity

//        flywheel   = flywheeltuner.init(telemetry, hardwareMap, gamepad1);

        hasFrontLeft  = (frontLeft  != null);
        hasFrontRight = (frontRight != null);
        hasBackLeft   = (backLeft   != null);
        hasBackRight  = (backRight  != null);
        hasIntake     = (intake     != null);
        hasFlywheel   = (flywheel   != null);

        // Drivetrain directions
        if (hasFrontLeft)  frontLeft.setDirection(DcMotor.Direction.REVERSE);
        if (hasBackLeft)   backLeft.setDirection(DcMotor.Direction.REVERSE);
        if (hasFrontRight) frontRight.setDirection(DcMotor.Direction.FORWARD);
        if (hasBackRight)  backRight.setDirection(DcMotor.Direction.FORWARD);

        if (hasFrontLeft)  frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (hasBackLeft)  backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (hasFrontRight)  frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (hasBackRight)  backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Brake
        if (hasFrontLeft)  frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (hasFrontRight) frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (hasBackLeft)   backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (hasBackRight)  backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (hasIntake)     intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (hasFlywheel) {
            // Shooter usually FLOATS on zero, so wheel can spin down naturally.
            flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            // We’ll switch to RUN_USING_ENCODER so .setVelocity() works in loop()
            flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            flywheel.setDirection(DcMotor.Direction.FORWARD);
        }


        // Feeder motor
        try {
            feederMotor = hardwareMap.get(DcMotorEx.class, "feeder");
            feederMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            feederMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            feederMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            feederMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            hasFeederMotor = true;
        } catch (Exception e) {
            telemetry.addLine("⚠️ Feeder motor not found (name: 'feeder').");
        }

        // LEFT feeder servo
        try {
            feederServoLeft = hardwareMap.get(CRServo.class, "feederServoLeft");
            feederServoLeft.setPower(0.0);
            hasFeederServoLeft = true;
        } catch (Exception e) {
            telemetry.addLine("⚠️ Left feeder servo not found (name: 'feederServoLeft').");
        }

        // RIGHT feeder servo
        try {
            feederServoRight = hardwareMap.get(CRServo.class, "feederServoRight");
            feederServoRight.setPower(0.0);
            hasFeederServoRight = true;
        } catch (Exception e) {
//            telemetry.addLine("⚠️ Right feeder servo not found (name: 'feederServoRight').");
        }
        // ARM SERVO
        try {
            armServo = hardwareMap.get(Servo.class, "armServo");
            armServo.scaleRange(0.2,0.8);
            armServo.setPosition(SERVO_HOME);   // start position
            hasArmServo = true;
        } catch (Exception e) {
            telemetry.addLine("⚠️ Arm servo not found (name: 'armServo').");
        }


        // AprilTag + ball detection on the same camera
        ballDetector.buildProcessors(telemetry);
        aprilTagWebcam.init(hardwareMap, telemetry, ballDetector.getProcessors());

        // ~1 foot (12 inches) shoot window
        aprilTagWebcam.setTargetRange(targetDist);
        aprilTagWebcam.setRangeTolerance(targetDistTol);      // 10–14 inches
        aprilTagWebcam.setTargetBearing(targetBearing);
        aprilTagWebcam.setBearingTolerance(targetBearingTol);

        // Status
        aprilTagWebcam.reportHardwareStatus();

//        leftFeederLauncher.reportHardwareStatus();
//        rightFeederLauncher.reportHardwareStatus();

        // RGB
        light.init(hardwareMap, telemetry, "indicator");
        light.blue();


    }

    @Override
    public void loop() {

        // -----------------------------
        // AprilTag Update + Shoot Ready
        // -----------------------------
        aprilTagWebcam.update();

        AprilTagDetection tag24 = aprilTagWebcam.getTagBySpecificId(24); // RED goal
        AprilTagDetection tag20 = aprilTagWebcam.getTagBySpecificId(20); // BLUE goal

        // Telemetry (optional but useful)
        if (tag24 != null) {
            telemetry.addLine("Seeing Tag 24 (RED GOAL)");
            telemetry.addData("Tag24 Range (in target 36 tol 6)", tag24.ftcPose.range);
            telemetry.addData("Tag24 Bearing (deg target -3 tol 7)", tag24.ftcPose.bearing);
        }
        if (tag20 != null) {
            telemetry.addLine("Seeing Tag 20 (BLUE GOAL)");
            telemetry.addData("Tag20 Range (in target 36 tol 6)", tag20.ftcPose.range);
            telemetry.addData("Tag20 Bearing (deg target 0 tol 3)", tag20.ftcPose.bearing);
        }

        // In-range if either goal tag is in the ~1ft zone
        boolean inShootRange =
                aprilTagWebcam.IsRobotinZone(24) ||
                        aprilTagWebcam.IsRobotinZone(20);

        // Debounce so light doesn't flicker
        if (inShootRange) shootInRangeCount++;
        else shootInRangeCount = 0;

        if (shootInRangeCount >= SHOOT_DEBOUNCE_FRAMES) {
            telemetry.addData("✅ SHOOT RANGE (Tag 24 or 20)", targetDist+'\t'+targetDistTol);
            light.green();
        } else {
            light.blue();
        }

        //----------------------------------
        // 1. DRIVE: mecanum with gamepad1
        //----------------------------------
        double y  = -gamepad1.left_stick_y;   // forward = +1
        double x  =  gamepad1.left_stick_x;   // strafe right = +1
        double rx =  gamepad1.right_stick_x;  // rotate right = +1

        //----------------------------------
        // 1B. BALL AUTO-AIM (gamepad1, hold to turn toward a ball)
        // LB = closest ball, X = blue, B = red, Y = yellow
        // Driver keeps forward/strafe control; only rotation is overridden.
        //----------------------------------
        ballDetector.update();

        BallDetector.Ball ballTarget = null;
        if (gamepad1.x) {
            ballTarget = ballDetector.getClosestBall(BallDetector.BallColor.BLUE);
        } else if (gamepad1.b) {
            ballTarget = ballDetector.getClosestBall(BallDetector.BallColor.RED);
        } else if (gamepad1.y) {
            ballTarget = ballDetector.getClosestBall(BallDetector.BallColor.YELLOW);
        } else if (gamepad1.left_bumper) {
            ballTarget = ballDetector.getClosestBall();
        }

        boolean ballAiming = gamepad1.left_bumper || gamepad1.x || gamepad1.b || gamepad1.y;
        if (ballAiming && ballTarget != null) {
            // bearing > 0 means ball is to the LEFT, so turn left (negative rx)
            if (Math.abs(ballTarget.bearing) > BALL_AIM_TOLERANCE) {
                rx = Range.clip(-ballTarget.bearing * BALL_TURN_GAIN,
                        -BALL_MAX_AUTO_TURN, BALL_MAX_AUTO_TURN);
            } else {
                rx = 0;
            }
        }

        double flPower = y + x + rx;
        double blPower = y - x + rx;
        double frPower = y - x - rx;
        double brPower = y + x - rx;

        double max = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        flPower /= max;
        blPower /= max;
        frPower /= max;
        brPower /= max;

        if (hasFrontLeft)  frontLeft.setPower(flPower);
        if (hasBackLeft)   backLeft.setPower(blPower);
        if (hasFrontRight) frontRight.setPower(frPower);
        if (hasBackRight)  backRight.setPower(brPower);

        //----------------------------------
        // 2. INTAKE CONTROL (gamepad2 A/B/X)
        //----------------------------------
        double intakePower = 0.0;
        if (gamepad2.a) {
            intakePower = 1.0;
        } else if (gamepad2.b) {
            intakePower = -1.0;
        } else if (gamepad2.x) {
            intakePower = 0.0;
        }
        if (hasIntake) {
            intake.setPower(intakePower);
        }
        //----------------------------------
        // 3. ARM SERVO CONTROL (ONE STEP PER PRESS)
        //----------------------------------
        if (hasArmServo) {
            double servoPos = armServo.getPosition();

            boolean rbNow = gamepad2.right_bumper;
            boolean lbNow = gamepad2.left_bumper;

            // Rising-edge detect: only step once when bumper is first pressed
            if (rbNow && !lastRightBumper) {
                servoPos += SERVO_STEP;
            } else if (lbNow && !lastLeftBumper) {
                servoPos -= SERVO_STEP;
            }

            // Save states for next loop
            lastRightBumper = rbNow;
            lastLeftBumper  = lbNow;

            // Clamp and apply
            servoPos = Math.max(SERVO_MIN, Math.min(SERVO_MAX, servoPos));
            armServo.setPosition(servoPos);
        }
        // ----------------------------------
        // 3B. PRESET: gamepad2 X -> 120° + 3150 RPM
        // ----------------------------------
        if (gamepad2.x && hasArmServo && hasFlywheel) {
            // Set arm to 120 degrees
            double presetPos = degToServoPos(120.0);
            armServo.setPosition(presetPos);

            // Lock flywheel target to 3150 RPM
            FLYWHEEL_TARGET_RPM = 3150.0;
            useManualRPM = true;
        }
        // ----------------------------------
// ----------------------------------
// 3B. PRESETS using RT & LT triggers
// RT  -> 120° and 2400 RPM
// LT  -> 180° and 2200 RPM
// ----------------------------------
        if (hasArmServo && hasFlywheel) {

            // RT preset
            if (gamepad2.right_trigger > 0.5) {
                double pos120 = degToServoPos(150.0);
                armServo.setPosition(pos120);
                //armServo.setPosition();

                FLYWHEEL_TARGET_RPM = 2400.0;
                useManualRPM = true;
                manualPresetLabel = "RT: 120° / 2400 RPM";
            }

            // LT preset
            if (gamepad2.left_trigger > 0.5) {
                double pos180 = degToServoPos(180.0);
                armServo.setPosition(pos180);

                FLYWHEEL_TARGET_RPM = 2200.0;
                useManualRPM = true;
                manualPresetLabel = "LT: 180° / 2200 RPM";
            }
        }





        //----------------------------------
        // 4. DUAL FEEDER SERVO CONTROL (D-PAD)
        //----------------------------------
        // Left servo: D-pad LEFT (forward)
        // Right servo: D-pad RIGHT (forward)
        double leftServoPower  = 0.0;
        double rightServoPower = 0.0;

        if (gamepad2.dpad_left) {
            leftServoPower = -1.0;    // forward
        }
        if (gamepad2.dpad_right) {
            rightServoPower = -1.0;   // forward
        }

        if (hasFeederServoLeft) {
            feederServoLeft.setPower(leftServoPower);
        }
        if (hasFeederServoRight) {
            feederServoRight.setPower(rightServoPower);
        }

        //----------------------------------
        // 5. FLYWHEEL + FEEDER MOTOR SEQUENCE
        //----------------------------------

//        flywheeltuner.update();

//        boolean toggleButton = gamepad2.triangle;    // use triangle to toggle sequence
//        boolean currentButtonState = toggleButton;
//
//        // Rising edge detect
//        if (currentButtonState && !lastButtonState) {
//            flywheelActive = !flywheelActive; // toggle state
//        }
//
//        // Save for next loop
//        lastButtonState = currentButtonState;

        if (hasFlywheel) {
//            if (flywheelActive) {
//                // edge-detect: run once when D-pad is first pressed
//                boolean dpadUpNow   = gamepad2.dpad_up;
//                boolean dpadDownNow = gamepad2.dpad_down;
//
////
//                if (!dpadPressed && (dpadUpNow || dpadDownNow)) {
//                    if (dpadUpNow) {
//                        FLYWHEEL_TARGET_RPM += 25;
//                    } else if (dpadDownNow) {
//                        FLYWHEEL_TARGET_RPM -= 25;
//                    }
//                    dpadPressed = true;   // lock until D-pad released
//                }
//
//                if (!dpadUpNow && !dpadDownNow) {
//                    dpadPressed = false;  // ready for next press
//                }

                // ... keep the rest of your flywheel code unchanged ...


//                if (tag24 != null) {
//                    remainingDistIn = tag24.ftcPose.range;
//                } else if (tag20 != null) {
//                    remainingDistIn = tag20.ftcPose.range;
//                } else {
//                    remainingDistIn = targetDist;
//                }

                // If not using manual preset, compute RPM from distance
//                if (!useManualRPM) {
//                    FLYWHEEL_TARGET_RPM = distToRPM.getFlywheelRPMForDistance(remainingDistIn);
//                }


                // Spin up flywheel to target velocity
                double targetTPS = rpmToTicksPerSec(FLYWHEEL_TARGET_RPM, FLYWHEEL_TPR);
                flywheel.setVelocity(targetTPS);

                // Read actual RPM
                double currentTPS = flywheel.getVelocity(); // ticks/sec
                double currentRPM = ticksPerSecToRPM(currentTPS, FLYWHEEL_TPR);
                boolean atSpeed = Math.abs(currentRPM - FLYWHEEL_TARGET_RPM) <= FLYWHEEL_TOLERANCE;

                double overshoot = currentRPM-FLYWHEEL_TARGET_RPM;
                if (overshoot>largest_overshoot) {
                    largest_overshoot = overshoot;
                }
                if (atSpeed) {
                    inToleranceCount = Math.min(inToleranceCount + 1, IN_TOLERANCE_REQUIRED);
                } else {
                    inToleranceCount = 0;
                }

                // Run feeder motor only when flywheel is stably at speed
                if (hasFeederMotor) {
                    if (inToleranceCount >= IN_TOLERANCE_REQUIRED) {
                        feederMotor.setVelocity(rpmToTicksPerSec(FEEDER_RPM, FEEDER_TPR));
                    }
                }

                telemetry.addData("Flywheel", "Target %.0f RPM | Now %.0f RPM %s",
                        FLYWHEEL_TARGET_RPM, currentRPM, atSpeed ? "(near target)" : "");
//                telemetry.addData("FeederMotor", (hasFeederMotor
//                        ? (inToleranceCount >= IN_TOLERANCE_REQUIRED ? "FEEDING" : "WAITING")
//                        : "N/A"));
                telemetry.addData("Largest Overshoot", largest_overshoot);
//            } else {
//                // Toggle off: stop both
//                flywheel.setPower(0.0);
//                inToleranceCount = 0;
//
//                if (hasFeederMotor) {
//                    feederMotor.setPower(0.0);
//                }
////                telemetry.addData("Flywheel State", "Stopped");
////                telemetry.addData("FeederMotor", hasFeederMotor ? "Stopped" : "N/A");
//            }
        }

        if (hasFlywheel) {
//            telemetry.addData("triangle", "toggle flywheel+feeder sequence");
            double currentVel = flywheel.getVelocity(); // ticks/sec
//            telemetry.addData("flywheelVel(ticks/s)", "%.1f", currentVel);

            double ticksPerRev = 28.0;
            double currentRPM = ticksPerSecToRPM(currentVel, ticksPerRev);
            telemetry.addData("flywheelVel(RPM est)", "%.0f", currentRPM);
            if (useManualRPM) {
                telemetry.addData("Flywheel Target", "3150 RPM (X-Preset)");
            } else {
                telemetry.addData("Flywheel Target", "Current: %.0f RPM", FLYWHEEL_TARGET_RPM);
            }

            if (useManualRPM && Math.abs(FLYWHEEL_TARGET_RPM - 3150.0) < 5.0) {
                telemetry.addData("Flywheel Target", "3150 RPM  (X-Preset)");
            } else {
                telemetry.addData("Flywheel Target", "%.0f RPM", FLYWHEEL_TARGET_RPM);
            }

        }

        //----------------------------------
        // 5. TELEMETRY / DRIVER FEEDBACK
        //----------------------------------
        telemetry.addLine("== DRIVE ==");
//        telemetry.addData("y/x/rx", "%.2f %.2f %.2f", y, x, rx);
//        telemetry.addData("fl/br/bl/fr", "%.2f %.2f %.2f %.2f",
//                flPower, brPower, blPower, frPower);

        telemetry.addLine("== BALLS ==");
        BallDetector.Ball closestBall = ballDetector.getClosestBall();
        if (closestBall != null) {
            telemetry.addData("Closest Ball", "%s  %.1f in  %.1f deg",
                    closestBall.color, closestBall.distance, closestBall.bearing);
        } else {
            telemetry.addData("Closest Ball", "none");
        }
        if (ballAiming) {
            telemetry.addData("Ball Aim", ballTarget != null
                    ? "AIMING at " + ballTarget.color
                    : "no matching ball in view");
        }

        telemetry.addLine("== INTAKE ==");
//        telemetry.addData("intakePower", "%.2f", intakePower);
//        telemetry.addData("intakePresent", hasIntake);

//        telemetry.addData("Flywheel Active", flywheelActive);

        if (hasFlywheel) {
            double currentVel = flywheel.getVelocity(); // ticks/sec
            double ticksPerRev = 28.0;
            double currentRPM = ticksPerSecToRPM(currentVel, ticksPerRev);

            // Always show actual RPM
            telemetry.addData("Flywheel RPM (actual)", "%.0f", currentRPM);

            // Show target + preset info
            if (useManualRPM && !manualPresetLabel.isEmpty()) {
                telemetry.addData("Flywheel Target", "%.0f RPM (Preset)", FLYWHEEL_TARGET_RPM);
                telemetry.addData("Preset", manualPresetLabel);
            } else {
                telemetry.addData("Flywheel Target", "%.0f RPM (Auto)", FLYWHEEL_TARGET_RPM);
            }
        }

        reportHardwareStatus();

// ----------------------------------
// ARM SERVO TELEMETRY
// ----------------------------------
        if (hasArmServo) {
            double pos = armServo.getPosition();      // 0.0 .. 1.0
            double deg = servoPosToDeg(pos);          // mapped degrees

            if (useManualRPM && !manualPresetLabel.isEmpty()) {
                telemetry.addData("Arm Angle", "%.1f° (Preset)", deg);
            } else {
                telemetry.addData("Arm Angle", "Current: %.1f°", deg);
            }
        } else {
            telemetry.addData("Arm Servo", "MISSING");
        }



        telemetry.update();
    }

    // -------------------------------------------------
    // Helper: safe motor fetch (DcMotor)
    // -------------------------------------------------
    private DcMotor getMotor(String name) {
        try {
            return hardwareMap.get(DcMotor.class, name);
        } catch (Exception e) {
            return null;
        }
    }

    // -------------------------------------------------
    // Helper: safe motor fetch as DcMotorEx
    // -------------------------------------------------
    private DcMotorEx getMotorEx(String name) {
        try {
            return hardwareMap.get(DcMotorEx.class, name);
        } catch (Exception e) {
            return null;
        }
    }

    // -------------------------------------------------
    // Helper: report which motors are missing
    // -------------------------------------------------
    private void reportHardwareStatus() {
        telemetry.addLine("== HARDWARE STATUS ==");
        telemetry.addData("frontLeft",   hasFrontLeft  ? "OK" : "MISSING");
        telemetry.addData("frontRight",  hasFrontRight ? "OK" : "MISSING");
        telemetry.addData("backLeft",    hasBackLeft   ? "OK" : "MISSING");
        telemetry.addData("backRight",   hasBackRight  ? "OK" : "MISSING");
        telemetry.addData("intake",      hasIntake     ? "OK" : "MISSING");
        telemetry.addData("feederMotor", hasFeederMotor ? "OK" : "MISSING");
    }

    // -------------------------------------------------
    // Math helpers for flywheel RPM <-> ticks/sec
    // -------------------------------------------------
    private double rpmToTicksPerSec(double rpm, double ticksPerRev) {
        double ticksPerMinute = rpm * ticksPerRev;
        return ticksPerMinute / 60.0;
    }

    private double ticksPerSecToRPM(double ticksPerSec, double ticksPerRev) {
        double ticksPerMin = ticksPerSec * 60.0;
        return ticksPerMin / ticksPerRev;
    }
    // -------------------------------------------------
// Helper: convert servo position (0–1) to degrees
// -------------------------------------------------
    private double servoPosToDeg(double pos) {
        pos = Math.max(SERVO_MIN, Math.min(SERVO_MAX, pos));
        double t = (pos - SERVO_MIN) / (SERVO_MAX - SERVO_MIN);
        return ARM_MIN_DEG + t * (ARM_MAX_DEG - ARM_MIN_DEG);
    }

    // -------------------------------------------------
// Helper: convert degrees to servo position (0–1)
// -------------------------------------------------
    private double degToServoPos(double degrees) {
        // normalize 0..1 in angle space
        double t = (degrees - ARM_MIN_DEG) / (ARM_MAX_DEG - ARM_MIN_DEG);
        t = Math.max(0.0, Math.min(1.0, t));   // clamp
        // map into [SERVO_MIN, SERVO_MAX]
        return SERVO_MIN + t * (SERVO_MAX - SERVO_MIN);
    }



    @Override
    public void start() {
        // Start flywheels on launchers (not ready to shoot)
        double targetTPS = rpmToTicksPerSec(FLYWHEEL_TARGET_RPM, FLYWHEEL_TPR);
        flywheel.setVelocity(targetTPS);
    }

    @Override
    public void stop() {
        // Closes the shared camera (AprilTags + ball detection)
        aprilTagWebcam.stop();
    }
}
