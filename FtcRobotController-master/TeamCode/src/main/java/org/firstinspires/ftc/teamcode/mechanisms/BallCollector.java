package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Autonomous ball pickup using the camera (BallDetector) and the Pedro follower.
 *
 * Steps, run from an autonomous loop() after the normal path is finished:
 *   SEARCH   - turn slowly in place until a ball is seen (gives up after SEARCH_TIMEOUT_MS)
 *   APPROACH - drive forward toward the ball, steering to keep it centered, intake ON
 *   CAPTURE  - the ball drops out of view under the intake; keep driving straight briefly
 *   DONE     - stop driving and intake
 *
 * Usage:
 *   init():  ballCollector.init(hardwareMap, telemetry, follower, intake);
 *   when the path is done:  ballCollector.start();
 *   loop():  ballCollector.update(msLeftInAuto);  then check ballCollector.isDone()
 *   stop():  ballCollector.stop();
 *
 * Assumes the camera and intake both face the FRONT of the robot.
 */
public class BallCollector {

    public enum State { IDLE, SEARCH, APPROACH, CAPTURE, DONE }

    // -----------------------------
    // Tuning (start slow, speed up once it works)
    // -----------------------------
    private static final double SEARCH_TURN_POWER   = 0.2;   // in-place turn while searching
    private static final double APPROACH_DRIVE      = 0.3;   // forward power toward the ball
    private static final double TURN_GAIN           = 0.02;  // turn power per degree of bearing
    private static final double MAX_TURN            = 0.3;
    private static final double CAPTURE_DISTANCE_IN = 10.0;  // ball this close -> blind capture
    private static final double CAPTURE_MS          = 700;   // drive straight this long to swallow it
    private static final double LOST_BALL_MS        = 500;   // ball unseen this long -> search again
    private static final double SEARCH_TIMEOUT_MS   = 4000;  // give up if nothing found
    private static final double STOP_BEFORE_END_MS  = 1000;  // stop with this much auto time left

    // Intake power that pulls balls IN (matches gamepad2 A in TeamBotTeleop_latest).
    // Flip the sign if the intake spits balls out.
    private static final double INTAKE_IN_POWER = 1.0;

    private final BallDetector ballDetector = new BallDetector();
    private Follower follower;
    private DcMotor intake;
    private Telemetry telemetry;

    private State state = State.IDLE;
    private boolean cameraOk = false;
    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime lastSeenTimer = new ElapsedTime();

    // Only collect this color; null = any color
    private BallDetector.BallColor targetColor = null;

    /**
     * Opens "Webcam 1" for ball detection. Call from the OpMode's init().
     * intake may be null (the robot will still drive to the ball).
     * If the webcam is missing, ball pickup is skipped instead of crashing the auto.
     */
    public void init(HardwareMap hwMap, Telemetry telemetry, Follower follower, DcMotor intake) {
        this.telemetry = telemetry;
        this.follower = follower;
        this.intake = intake;
        try {
            ballDetector.init(hwMap, telemetry);
            cameraOk = true;
        } catch (Exception e) {
            telemetry.addLine("⚠️ Webcam 1 not found - ball pickup disabled.");
        }
    }

    public void setTargetColor(BallDetector.BallColor color) {
        this.targetColor = color;
    }

    /** Begin searching for a ball. The Pedro path must be finished first. */
    public void start() {
        if (!cameraOk) {
            setState(State.DONE);
            return;
        }
        follower.startTeleopDrive();
        setState(State.SEARCH);
    }

    /**
     * Run once per loop() while collecting.
     * @param msLeftInAuto milliseconds left in autonomous; stops safely before time runs out.
     */
    public void update(double msLeftInAuto) {
        if (state == State.IDLE || state == State.DONE) return;

        ballDetector.update();
        BallDetector.Ball ball = (targetColor == null)
                ? ballDetector.getClosestBall()
                : ballDetector.getClosestBall(targetColor);
        if (ball != null) lastSeenTimer.reset();

        if (msLeftInAuto < STOP_BEFORE_END_MS) {
            finish();
            return;
        }

        switch (state) {
            case SEARCH:
                if (ball != null) {
                    setState(State.APPROACH);
                } else if (stateTimer.milliseconds() > SEARCH_TIMEOUT_MS) {
                    finish();
                } else {
                    drive(0, SEARCH_TURN_POWER);   // turn left slowly
                }
                break;

            case APPROACH:
                setIntake(INTAKE_IN_POWER);
                if (ball == null) {
                    if (lastSeenTimer.milliseconds() > LOST_BALL_MS) {
                        setState(State.SEARCH);
                    } else {
                        drive(APPROACH_DRIVE, 0);  // brief dropout: keep going straight
                    }
                } else if (ball.distance <= CAPTURE_DISTANCE_IN) {
                    setState(State.CAPTURE);
                } else {
                    // bearing > 0 = ball is LEFT; Pedro turn > 0 = turn LEFT
                    double turn = Range.clip(ball.bearing * TURN_GAIN, -MAX_TURN, MAX_TURN);
                    drive(APPROACH_DRIVE, turn);
                }
                break;

            case CAPTURE:
                setIntake(INTAKE_IN_POWER);
                if (stateTimer.milliseconds() < CAPTURE_MS) {
                    drive(APPROACH_DRIVE, 0);
                } else {
                    finish();
                }
                break;

            default:
                break;
        }

        telemetry.addData("Ball Collector", state);
        if (ball != null) {
            telemetry.addData("Target Ball", "%s  %.1f in  %.1f deg",
                    ball.color, ball.distance, ball.bearing);
        }
    }

    public boolean isDone() {
        return state == State.DONE;
    }

    public State getState() {
        return state;
    }

    /** Stops motors and closes the camera. Call from the OpMode's stop(). */
    public void stop() {
        if (state != State.IDLE) {
            drive(0, 0);
        }
        setIntake(0);
        if (cameraOk) ballDetector.stop();
    }

    private void finish() {
        drive(0, 0);
        setIntake(0);
        setState(State.DONE);
    }

    private void setState(State newState) {
        state = newState;
        stateTimer.reset();
    }

    /** Robot-centric drive through Pedro: forward > 0 = forward, turn > 0 = turn left. */
    private void drive(double forward, double turn) {
        follower.setTeleOpDrive(forward, 0, turn, true);
    }

    private void setIntake(double power) {
        if (intake != null) intake.setPower(power);
    }
}
