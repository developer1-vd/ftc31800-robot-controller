package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.mechanisms.BallDetector;

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

    private enum State { SEEK, CAPTURE, SETTLE, FULL }
    private State state = State.SEEK;
    private final ElapsedTime stateTimer = new ElapsedTime();
    private int ballsCollected = 0;

    private boolean announcedArrival = false;

    private final BallDetector ballDetector = new BallDetector();

    private DcMotor frontLeft  = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft   = null;
    private DcMotor backRight  = null;
    private DcMotor intake     = null;
    private boolean hasDrive   = false;

    @Override
    public void init() {
        frontLeft  = getMotor("left_front_drive");
        frontRight = getMotor("right_front_drive");
        backLeft   = getMotor("left_back_drive");
        backRight  = getMotor("right_back_drive");
        intake     = getMotor("intake");
        hasDrive = frontLeft != null || frontRight != null || backLeft != null || backRight != null;

        if (frontLeft  != null) frontLeft.setDirection(DcMotor.Direction.REVERSE);
        if (backLeft   != null) backLeft.setDirection(DcMotor.Direction.REVERSE);
        if (frontRight != null) frontRight.setDirection(DcMotor.Direction.FORWARD);
        if (backRight  != null) backRight.setDirection(DcMotor.Direction.FORWARD);
        for (DcMotor m : new DcMotor[] { frontLeft, frontRight, backLeft, backRight, intake }) {
            if (m != null) m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        ballDetector.init(hardwareMap, telemetry);
        ballDetector.setSoundEnabled(SOUND_ENABLED);

        if (hasDrive && canPickUp()) {
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
    public void loop() {
        ballDetector.update();
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
                    setState(ballsCollected >= MAX_BALLS ? State.FULL : State.SEEK);
                }
                break;

            case FULL:
                setDrivePower(0, 0);
                setIntake(0);
                status = "full (" + ballsCollected + " balls) - stopped";
                break;

            case SEEK:
            default:
                status = seek();
                break;
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
        ballDetector.stop();
    }

    private DcMotor getMotor(String name) {
        try {
            return hardwareMap.get(DcMotor.class, name);
        } catch (Exception e) {
            return null;
        }
    }
}
