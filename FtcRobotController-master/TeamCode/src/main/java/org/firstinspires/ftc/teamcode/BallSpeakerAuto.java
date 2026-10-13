package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.mechanisms.BallDetector;

/**
 * Autonomous that watches for yellow / blue / red balls, says the color out loud
 * on the Driver Station ("red ball"), turns to face the closest ball, and drives
 * up to it, stopping STOP_DISTANCE_IN away.
 *
 * - Camera-only robot (Control Hub + battery + "Webcam 1"): detects and speaks only.
 * - With drive motors connected: also turns and drives to the ball after Play.
 *   It never moves during INIT, and stops when no ball is in view.
 */
@Autonomous(name = "BallSpeakerAuto", group = "Vision")
public class BallSpeakerAuto extends OpMode {

    // Spoken announcements on the Driver Station. Set to false for silence.
    private static final boolean SOUND_ENABLED = true;

    // Turn toward the closest ball. Set to false to only detect and speak.
    private static final boolean TURN_TO_BALL = true;

    // Drive up to the ball after facing it. Set to false to only turn.
    private static final boolean DRIVE_TO_BALL = true;

    // Proportional turn gain (power per degree of bearing) and limits
    private static final double TURN_GAIN     = 0.02;
    private static final double MAX_TURN      = 0.4;
    private static final double AIM_TOLERANCE = 2.0;  // degrees

    // Driving: only drive forward when roughly facing the ball, slow down as it gets close
    private static final double DRIVE_GAIN         = 0.02;  // power per inch of remaining distance
    private static final double MAX_DRIVE          = 0.3;   // start slow; raise once it works
    private static final double MIN_DRIVE          = 0.1;   // enough power to actually move
    private static final double DRIVE_MAX_BEARING  = 15.0;  // degrees; turn first if more off than this
    private static final double STOP_DISTANCE_IN   = 8.0;   // stop this far from the ball

    private boolean announcedArrival = false;

    private final BallDetector ballDetector = new BallDetector();

    private DcMotor frontLeft  = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft   = null;
    private DcMotor backRight  = null;
    private boolean hasDrive   = false;

    @Override
    public void init() {
        frontLeft  = getMotor("left_front_drive");
        frontRight = getMotor("right_front_drive");
        backLeft   = getMotor("left_back_drive");
        backRight  = getMotor("right_back_drive");
        hasDrive = frontLeft != null || frontRight != null || backLeft != null || backRight != null;

        if (frontLeft  != null) frontLeft.setDirection(DcMotor.Direction.REVERSE);
        if (backLeft   != null) backLeft.setDirection(DcMotor.Direction.REVERSE);
        if (frontRight != null) frontRight.setDirection(DcMotor.Direction.FORWARD);
        if (backRight  != null) backRight.setDirection(DcMotor.Direction.FORWARD);
        for (DcMotor m : new DcMotor[] { frontLeft, frontRight, backLeft, backRight }) {
            if (m != null) m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        ballDetector.init(hardwareMap, telemetry);
        ballDetector.setSoundEnabled(SOUND_ENABLED);

        if (hasDrive && TURN_TO_BALL) {
            telemetry.addLine("Ball speaker ready. After Play, turns and drives to the closest ball.");
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

        double turn = 0;     // + = turn right (same as right stick in teleop)
        double forward = 0;  // + = drive forward
        String status = "no ball - holding still";

        BallDetector.Ball ball = ballDetector.getClosestBall();
        if (TURN_TO_BALL && ball != null) {
            if (Math.abs(ball.bearing) > AIM_TOLERANCE) {
                // bearing > 0 means ball is to the LEFT, so turn left (negative turn)
                turn = Range.clip(-ball.bearing * TURN_GAIN, -MAX_TURN, MAX_TURN);
            }

            double remaining = ball.distance - STOP_DISTANCE_IN;
            if (DRIVE_TO_BALL && remaining > 0 && Math.abs(ball.bearing) <= DRIVE_MAX_BEARING) {
                forward = Range.clip(remaining * DRIVE_GAIN, MIN_DRIVE, MAX_DRIVE);
            }

            if (DRIVE_TO_BALL && remaining <= 0 && turn == 0) {
                status = "at " + ball.color + " ball";
                if (!announcedArrival) {
                    ballDetector.announce("at ball");
                    announcedArrival = true;
                }
            } else {
                status = (forward > 0 ? "driving to " : "turning to ") + ball.color + " ball";
                if (remaining > 4) announcedArrival = false;   // moved away -> can announce again
            }
        }
        setDrivePower(forward, turn);

        if (hasDrive && TURN_TO_BALL) {
            telemetry.addData("Robot", status);
        }
        report();
    }

    private void report() {
        telemetry.addData("Sound", ballDetector.isSoundEnabled() ? "ON" : "OFF");
        telemetry.addData("Drive", hasDrive ? "connected" : "none (camera-only)");
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

    @Override
    public void stop() {
        setDrivePower(0, 0);
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
