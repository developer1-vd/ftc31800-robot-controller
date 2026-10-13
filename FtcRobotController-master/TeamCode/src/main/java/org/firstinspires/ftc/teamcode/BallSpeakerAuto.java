package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.mechanisms.BallDetector;

/**
 * Autonomous that watches for yellow / blue / red balls, says the color out loud
 * on the Driver Station ("red ball"), and turns the robot in place to face the
 * closest ball.
 *
 * - Camera-only robot (Control Hub + battery + "Webcam 1"): detects and speaks only.
 * - With drive motors connected: also turns toward the ball after Play.
 *   It never moves during INIT, and stops turning when no ball is in view.
 */
@Autonomous(name = "BallSpeakerAuto", group = "Vision")
public class BallSpeakerAuto extends OpMode {

    // Spoken announcements on the Driver Station. Set to false for silence.
    private static final boolean SOUND_ENABLED = true;

    // Turn toward the closest ball. Set to false to only detect and speak.
    private static final boolean TURN_TO_BALL = true;

    // Proportional turn gain (power per degree of bearing) and limits
    private static final double TURN_GAIN     = 0.02;
    private static final double MAX_TURN      = 0.4;
    private static final double AIM_TOLERANCE = 2.0;  // degrees

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
            telemetry.addLine("Ball speaker ready. After Play, turns to face the closest ball.");
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

        double turn = 0;   // + = turn right (same as right stick in teleop)
        BallDetector.Ball ball = ballDetector.getClosestBall();
        if (TURN_TO_BALL && ball != null && Math.abs(ball.bearing) > AIM_TOLERANCE) {
            // bearing > 0 means ball is to the LEFT, so turn left (negative turn)
            turn = Range.clip(-ball.bearing * TURN_GAIN, -MAX_TURN, MAX_TURN);
        }
        setTurnPower(turn);

        if (hasDrive && TURN_TO_BALL) {
            telemetry.addData("Turning", ball == null ? "no ball - holding still"
                    : (turn == 0 ? "facing " + ball.color + " ball" : "toward " + ball.color + " ball"));
        }
        report();
    }

    private void report() {
        telemetry.addData("Sound", ballDetector.isSoundEnabled() ? "ON" : "OFF");
        telemetry.addData("Drive", hasDrive ? "connected" : "none (camera-only)");
        ballDetector.displayTelemetry();
        telemetry.update();
    }

    /** Mecanum turn in place: left side +turn, right side -turn. */
    private void setTurnPower(double turn) {
        if (frontLeft  != null) frontLeft.setPower(turn);
        if (backLeft   != null) backLeft.setPower(turn);
        if (frontRight != null) frontRight.setPower(-turn);
        if (backRight  != null) backRight.setPower(-turn);
    }

    @Override
    public void stop() {
        setTurnPower(0);
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
