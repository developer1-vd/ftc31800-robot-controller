package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.mechanisms.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.mechanisms.BallDetector;
import org.firstinspires.ftc.teamcode.mechanisms.FeederLauncher;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

/**
 * Autonomous that watches for yellow / blue / red balls, says the color out loud
 * on the Driver Station ("red ball"), turns to face the closest ball, drives to it,
 * and picks it up with the intake.
 *
 * Works with whatever hardware is configured:
 * - Camera only (Control Hub + battery + "Webcam 1"): detects and speaks.
 * - + drive motors: also turns and drives to the ball, stopping STOP_DISTANCE_IN away.
 * - + "intake" motor: drives onto the ball with the intake running and picks it up,
 *   says "got ball", then goes after the next one (up to MAX_BALLS).
 * - + "launcher" motor and feeder servos: when full (or holding balls with none left in
 *   view), turns until it sees a goal AprilTag (20 blue / 24 red), lines up in the same
 *   shoot zone as TeamBotTeleop_latest (36 in, -3 deg), shoots, then collects again.
 *
 * The robot never moves during INIT, and stops when no ball is in view.
 * Assumes the camera and intake both face the FRONT of the robot.
 */
@Autonomous(name = "BallSpeakerAuto", group = "Vision")
public class BallSpeakerAuto extends OpMode {

    // Spoken announcements on the Driver Station. Set to false for silence.
    private static final boolean SOUND_ENABLED = true;

    // Turn toward the closest ball. Set to false to only detect and speak.
    private static final boolean TURN_TO_BALL = true;

    // Drive up to the ball after facing it. Set to false to only turn.
    private static final boolean DRIVE_TO_BALL = true;

    // Pick the ball up with the intake (needs an "intake" motor). Set to false to just drive up to it.
    private static final boolean PICK_UP_BALL = true;

    // Proportional turn gain (power per degree of bearing) and limits
    private static final double TURN_GAIN     = 0.02;
    private static final double MAX_TURN      = 0.4;
    private static final double AIM_TOLERANCE = 2.0;  // degrees

    // Driving: only drive forward when roughly facing the ball, slow down as it gets close
    private static final double DRIVE_GAIN         = 0.02;  // power per inch of remaining distance
    private static final double MAX_DRIVE          = 0.3;   // start slow; raise once it works
    private static final double MIN_DRIVE          = 0.1;   // enough power to actually move
    private static final double DRIVE_MAX_BEARING  = 15.0;  // degrees; turn first if more off than this
    private static final double STOP_DISTANCE_IN   = 8.0;   // no intake: stop this far from the ball

    // Pickup (with intake)
    // Intake power that pulls balls IN (matches gamepad2 A in TeamBotTeleop_latest).
    // Flip the sign if the intake spits balls out.
    private static final double INTAKE_IN_POWER     = 1.0;
    private static final double CAPTURE_DISTANCE_IN = 10.0;  // this close -> drive straight onto it
    private static final double CAPTURE_DRIVE       = 0.25;  // forward power while swallowing the ball
    private static final double CAPTURE_MS          = 800;   // how long to drive onto the ball
    private static final double INTAKE_SETTLE_MS    = 500;   // keep intake running after, to pull it in
    private static final int    MAX_BALLS           = 3;     // stop after this many (robot capacity)

    // Shooting (needs "launcher" motor + "feederServoLeft"/"feederServoRight"). Set false to skip.
    private static final boolean SHOOT_BALLS = true;
    // Which goal to shoot at: 0 = whichever goal tag it sees, 24 = RED goal, 20 = BLUE goal.
    // Set this to your alliance's goal for a real match.
    private static final int    GOAL_TAG_ID           = 0;
    // Shoot zone, same values as TeamBotTeleop_latest (AprilTag range/bearing)
    private static final double SHOOT_RANGE_IN        = 36.0;
    private static final double SHOOT_RANGE_TOL_IN    = 6.0;
    private static final double SHOOT_BEARING_DEG     = -3.0;
    private static final double SHOOT_BEARING_TOL_DEG = 3.0;
    private static final double SHOOT_RPM             = 2400;   // teleop default / RT preset
    private static final double SHOOT_ARM_POSITION    = 0.5;    // same arm position the autos use
    private static final double GOAL_SEARCH_TURN      = 0.2;    // turn power while looking for the goal
    private static final double GOAL_SEARCH_MS        = 6000;   // give up finding the goal after this
    private static final double GOAL_LOST_MS          = 500;    // tag unseen this long -> search again
    private static final double SHOOT_WHEN_IDLE_MS    = 3000;   // holding balls, none in view -> go shoot
    private static final double FEED_INTAKE_MS        = 1000;   // intake run that moves balls to feeders
    private static final double SHOT_DELAY_MS         = 2000;   // between shots (same as the autos)
    private static final double SHOOT_TIMEOUT_MS      = 15000;  // safety: give up a stuck shot sequence

    private enum State { SEEK, CAPTURE, SETTLE, FIND_GOAL, LINE_UP, SHOOT, FULL }
    private enum ShotStep { FEED, FIRE, WAIT_LAUNCHED, DELAY }
    private State state = State.SEEK;
    private final ElapsedTime stateTimer = new ElapsedTime();
    private int ballsCollected = 0;
    private final ElapsedTime lastBallSeenTimer = new ElapsedTime();
    private final ElapsedTime goalLastSeenTimer = new ElapsedTime();

    // Shooting progress
    private ShotStep shotStep = ShotStep.FEED;
    private final ElapsedTime shotTimer = new ElapsedTime();
    private int shotsFired = 0;
    private int shotsToFire = 0;
    private boolean fireLeftNext = true;

    private boolean announcedArrival = false;

    private final BallDetector ballDetector = new BallDetector();
    private final AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();   // shares "Webcam 1"
    private final FeederLauncher leftLauncher  = new FeederLauncher();
    private final FeederLauncher rightLauncher = new FeederLauncher();

    private DcMotor frontLeft  = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft   = null;
    private DcMotor backRight  = null;
    private DcMotor intake     = null;
    private Servo   arm        = null;
    private boolean hasDrive   = false;
    private boolean hasLauncher = false;

    @Override
    public void init() {
        frontLeft  = getMotor("left_front_drive");
        frontRight = getMotor("right_front_drive");
        backLeft   = getMotor("left_back_drive");
        backRight  = getMotor("right_back_drive");
        intake     = getMotor("intake");
        hasLauncher = getMotor("launcher") != null;
        hasDrive = frontLeft != null || frontRight != null || backLeft != null || backRight != null;

        if (frontLeft  != null) frontLeft.setDirection(DcMotor.Direction.REVERSE);
        if (backLeft   != null) backLeft.setDirection(DcMotor.Direction.REVERSE);
        if (frontRight != null) frontRight.setDirection(DcMotor.Direction.FORWARD);
        if (backRight  != null) backRight.setDirection(DcMotor.Direction.FORWARD);
        for (DcMotor m : new DcMotor[] { frontLeft, frontRight, backLeft, backRight, intake }) {
            if (m != null) m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // Ball detection + AprilTags (for finding the goal) on the same camera
        ballDetector.buildProcessors(telemetry);
        aprilTagWebcam.init(hardwareMap, telemetry, ballDetector.getProcessors());
        ballDetector.setSoundEnabled(SOUND_ENABLED);

        if (hasLauncher) {
            leftLauncher.init(hardwareMap, telemetry, "launcher", "feederServoLeft");
            rightLauncher.init(hardwareMap, telemetry, "launcher", "feederServoRight");
        }
        try {
            arm = hardwareMap.get(Servo.class, "armServo");
            arm.scaleRange(0.2, 0.8);   // same range as the autos
        } catch (Exception e) {
            arm = null;
        }

        if (hasDrive && canShoot()) {
            telemetry.addLine("Ready. After Play, picks up balls and shoots them into the goal.");
        } else if (hasDrive && canPickUp()) {
            telemetry.addLine("Ready. After Play, drives to balls and picks them up.");
        } else if (hasDrive && TURN_TO_BALL) {
            telemetry.addLine("Ready. After Play, turns and drives to the closest ball.");
        } else {
            telemetry.addLine("Camera-only ball speaker ready. Show it a ball.");
        }
    }

    @Override
    public void init_loop() {
        // Detect and speak only; the robot must not move during INIT.
        ballDetector.update();
        report();
    }

    @Override
    public void start() {
        if (arm != null && canShoot()) arm.setPosition(SHOOT_ARM_POSITION);
        lastBallSeenTimer.reset();
    }

    @Override
    public void loop() {
        ballDetector.update();
        aprilTagWebcam.update();
        if (ballDetector.hasBall()) lastBallSeenTimer.reset();
        String status;

        switch (state) {
            case CAPTURE:
                // The ball is now too close for the camera to see; drive straight onto it.
                setIntake(INTAKE_IN_POWER);
                setDrivePower(CAPTURE_DRIVE, 0);
                status = "picking up ball";
                if (stateTimer.milliseconds() > CAPTURE_MS) {
                    setDrivePower(0, 0);
                    ballsCollected++;
                    ballDetector.announce("got ball");
                    setState(State.SETTLE);
                }
                break;

            case SETTLE:
                // Stopped; keep the intake running a moment to pull the ball fully in.
                setDrivePower(0, 0);
                setIntake(INTAKE_IN_POWER);
                status = "pulling ball in";
                if (stateTimer.milliseconds() > INTAKE_SETTLE_MS) {
                    setIntake(0);
                    if (ballsCollected < MAX_BALLS) {
                        setState(State.SEEK);
                    } else {
                        setState(canShoot() ? State.FIND_GOAL : State.FULL);
                    }
                }
                break;

            case FIND_GOAL:
                status = findGoal();
                break;

            case LINE_UP:
                status = lineUp();
                break;

            case SHOOT:
                status = shoot();
                break;

            case FULL:
                setDrivePower(0, 0);
                setIntake(0);
                status = "full (" + ballsCollected + " balls) - stopped";
                break;

            case SEEK:
            default:
                // Holding balls but none left in view: go shoot what we have
                if (canShoot() && ballsCollected > 0
                        && lastBallSeenTimer.milliseconds() > SHOOT_WHEN_IDLE_MS) {
                    setState(State.FIND_GOAL);
                    status = "no more balls - going to shoot";
                } else {
                    status = seek();
                }
                break;
        }

        // Launchers run their own spin-up/feed timing; keep them updated every loop
        if (hasLauncher) {
            leftLauncher.update();
            rightLauncher.update();
        }

        if (hasDrive && TURN_TO_BALL) {
            telemetry.addData("Robot", status);
        }
        report();
    }

    /** Turn toward and drive to the closest ball. Returns a status line. */
    private String seek() {
        BallDetector.Ball ball = ballDetector.getClosestBall();
        if (!TURN_TO_BALL || ball == null) {
            setDrivePower(0, 0);
            setIntake(0);
            return "no ball - holding still";
        }

        double turn = 0;     // + = turn right (same as right stick in teleop)
        double forward = 0;  // + = drive forward
        boolean facing = Math.abs(ball.bearing) <= DRIVE_MAX_BEARING;

        if (Math.abs(ball.bearing) > AIM_TOLERANCE) {
            // bearing > 0 means ball is to the LEFT, so turn left (negative turn)
            turn = Range.clip(-ball.bearing * TURN_GAIN, -MAX_TURN, MAX_TURN);
        }

        if (canPickUp()) {
            // Drive all the way onto the ball with the intake running
            if (ball.distance <= CAPTURE_DISTANCE_IN && facing) {
                setState(State.CAPTURE);
                return "picking up " + ball.color + " ball";
            }
            if (facing) {
                double remaining = ball.distance - CAPTURE_DISTANCE_IN;
                forward = Range.clip(remaining * DRIVE_GAIN, MIN_DRIVE, MAX_DRIVE);
                setIntake(INTAKE_IN_POWER);
            } else {
                setIntake(0);
            }
            setDrivePower(forward, turn);
            return (forward > 0 ? "driving to " : "turning to ") + ball.color + " ball";
        }

        // No intake: drive up and stop STOP_DISTANCE_IN away
        double remaining = ball.distance - STOP_DISTANCE_IN;
        if (DRIVE_TO_BALL && remaining > 0 && facing) {
            forward = Range.clip(remaining * DRIVE_GAIN, MIN_DRIVE, MAX_DRIVE);
        }
        setDrivePower(forward, turn);

        if (DRIVE_TO_BALL && remaining <= 0 && turn == 0) {
            if (!announcedArrival) {
                ballDetector.announce("at ball");
                announcedArrival = true;
            }
            return "at " + ball.color + " ball";
        }
        if (remaining > 4) announcedArrival = false;   // moved away -> can announce again
        return (forward > 0 ? "driving to " : "turning to ") + ball.color + " ball";
    }

    // -------------------------------------------------
    // Shooting
    // -------------------------------------------------

    /** Turn in place until a goal AprilTag is visible. */
    private String findGoal() {
        setIntake(0);
        if (getGoalTag() != null) {
            goalLastSeenTimer.reset();
            setState(State.LINE_UP);
            return "found goal";
        }
        if (stateTimer.milliseconds() > GOAL_SEARCH_MS) {
            setDrivePower(0, 0);
            ballDetector.announce("can't find goal");
            // Full: stop here. Otherwise keep collecting and try again later.
            setState(ballsCollected >= MAX_BALLS ? State.FULL : State.SEEK);
            lastBallSeenTimer.reset();
            return "can't find goal";
        }
        setDrivePower(0, -GOAL_SEARCH_TURN);   // turn left slowly
        return "looking for goal";
    }

    /** Turn to face the goal and drive into the shoot zone. */
    private String lineUp() {
        AprilTagDetection tag = getGoalTag();
        if (tag == null) {
            setDrivePower(0, 0);
            if (goalLastSeenTimer.milliseconds() > GOAL_LOST_MS) {
                setState(State.FIND_GOAL);
            }
            return "lost goal";
        }
        goalLastSeenTimer.reset();

        double bearingError = tag.ftcPose.bearing - SHOOT_BEARING_DEG;   // + = goal is LEFT
        double rangeError   = tag.ftcPose.range - SHOOT_RANGE_IN;        // + = too far

        boolean aimed   = Math.abs(bearingError) <= SHOOT_BEARING_TOL_DEG;
        boolean inRange = Math.abs(rangeError) <= SHOOT_RANGE_TOL_IN;
        if (aimed && inRange) {
            setDrivePower(0, 0);
            startShooting();
            return "in shoot zone";
        }

        double turn = aimed ? 0 : Range.clip(-bearingError * TURN_GAIN, -MAX_TURN, MAX_TURN);
        double forward = 0;
        if (!inRange && Math.abs(bearingError) <= DRIVE_MAX_BEARING) {
            forward = Math.signum(rangeError)
                    * Range.clip(Math.abs(rangeError) * DRIVE_GAIN, MIN_DRIVE, MAX_DRIVE);
        }
        setDrivePower(forward, turn);
        return String.format("lining up: %.0f in, %.1f deg", tag.ftcPose.range, tag.ftcPose.bearing);
    }

    private void startShooting() {
        shotsFired = 0;
        // We can't tell which side each ball is on, so always fire left AND right
        // (1-2 balls -> 2 shots, 3-4 balls -> 4 shots, like the competition autos).
        shotsToFire = ((ballsCollected + 1) / 2) * 2;
        fireLeftNext = true;
        shotStep = ShotStep.FEED;
        shotTimer.reset();
        ballDetector.announce("shooting");
        setState(State.SHOOT);
    }

    /**
     * Same pattern as the competition autos: run the intake to move balls up to the
     * feeders, then shoot left, wait, shoot right, and feed again for any extra ball.
     */
    private String shoot() {
        setDrivePower(0, 0);

        if (shotsFired >= shotsToFire || stateTimer.milliseconds() > SHOOT_TIMEOUT_MS) {
            finishShooting();
            return "done shooting";
        }

        switch (shotStep) {
            case FEED:
                setIntake(INTAKE_IN_POWER);
                if (shotTimer.milliseconds() > FEED_INTAKE_MS) {
                    setIntake(0);
                    shotStep = ShotStep.FIRE;
                }
                return "feeding balls to launcher";

            case FIRE:
                (fireLeftNext ? leftLauncher : rightLauncher).launch(SHOOT_RPM, 0, true, true);
                shotStep = ShotStep.WAIT_LAUNCHED;
                return "spinning up";

            case WAIT_LAUNCHED:
                FeederLauncher launcher = fireLeftNext ? leftLauncher : rightLauncher;
                if (launcher.getStatus() == FeederLauncher.LaunchState.LAUNCHED) {
                    shotsFired++;
                    // After a left+right pair, feed the next ball up; otherwise switch sides
                    boolean pairDone = !fireLeftNext;
                    fireLeftNext = !fireLeftNext;
                    shotStep = pairDone ? ShotStep.FEED : ShotStep.DELAY;
                    shotTimer.reset();
                }
                return "shooting (" + (shotsFired + 1) + " of " + shotsToFire + ")";

            case DELAY:
            default:
                if (shotTimer.milliseconds() > SHOT_DELAY_MS) {
                    shotStep = ShotStep.FIRE;
                }
                return "waiting between shots";
        }
    }

    private void finishShooting() {
        setIntake(0);
        leftLauncher.stop();     // flywheel off until the next round
        rightLauncher.stop();
        ballsCollected = 0;
        lastBallSeenTimer.reset();
        setState(State.SEEK);
    }

    /** The goal tag to shoot at (GOAL_TAG_ID, or the closer of 20/24 when it is 0). */
    private AprilTagDetection getGoalTag() {
        if (GOAL_TAG_ID != 0) {
            return aprilTagWebcam.getTagBySpecificId(GOAL_TAG_ID);
        }
        AprilTagDetection red  = aprilTagWebcam.getTagBySpecificId(24);
        AprilTagDetection blue = aprilTagWebcam.getTagBySpecificId(20);
        if (red == null) return blue;
        if (blue == null) return red;
        return red.ftcPose.range <= blue.ftcPose.range ? red : blue;
    }

    private boolean canShoot() {
        return SHOOT_BALLS && canPickUp() && hasLauncher;
    }

    private boolean canPickUp() {
        return PICK_UP_BALL && DRIVE_TO_BALL && TURN_TO_BALL && hasDrive && intake != null;
    }

    private void setState(State newState) {
        state = newState;
        stateTimer.reset();
    }

    private void report() {
        telemetry.addData("Sound", ballDetector.isSoundEnabled() ? "ON" : "OFF");
        telemetry.addData("Drive", hasDrive ? "connected" : "none (camera-only)");
        telemetry.addData("Intake", intake != null ? "connected" : "none");
        telemetry.addData("Launcher", hasLauncher ? "connected" : "none");
        if (canPickUp()) {
            telemetry.addData("Balls collected", "%d / %d", ballsCollected, MAX_BALLS);
        }
        ballDetector.displayTelemetry();
        telemetry.update();
    }

    /** Mecanum forward + turn (no strafe): left side = forward + turn, right side = forward - turn. */
    private void setDrivePower(double forward, double turn) {
        double max = Math.max(Math.abs(forward) + Math.abs(turn), 1);
        double left  = (forward + turn) / max;
        double right = (forward - turn) / max;
        if (frontLeft  != null) frontLeft.setPower(left);
        if (backLeft   != null) backLeft.setPower(left);
        if (frontRight != null) frontRight.setPower(right);
        if (backRight  != null) backRight.setPower(right);
    }

    private void setIntake(double power) {
        if (intake != null) intake.setPower(power);
    }

    @Override
    public void stop() {
        setDrivePower(0, 0);
        setIntake(0);
        if (hasLauncher) {
            leftLauncher.stop();
            rightLauncher.stop();
        }
        aprilTagWebcam.stop();   // closes the shared camera
    }

    private DcMotor getMotor(String name) {
        try {
            return hardwareMap.get(DcMotor.class, name);
        } catch (Exception e) {
            return null;
        }
    }
}
