package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.mechanisms.BallDetector;

/**
 * Test OpMode for camera-based ball detection.
 *
 * - Shows every detected ball (color, distance, bearing) on telemetry.
 * - Mecanum drive on gamepad1.
 * - Hold gamepad1 LEFT BUMPER to auto-turn toward the closest ball
 *   (X = closest purple only, Y = closest green only).
 *
 * Also use this to tune BallDetector: check radius/distance at known distances
 * and adjust the focal length and blob filters.
 */
@TeleOp(name = "BallDetectionTest", group = "Vision")
public class BallDetectionTest extends OpMode {

    private final BallDetector ballDetector = new BallDetector();

    private DcMotor frontLeft  = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft   = null;
    private DcMotor backRight  = null;

    // Proportional turn gain (power per degree of bearing) and limits
    private static final double TURN_GAIN      = 0.02;
    private static final double MAX_AUTO_TURN  = 0.4;
    private static final double AIM_TOLERANCE  = 2.0;  // degrees

    @Override
    public void init() {
        frontLeft  = getMotor("left_front_drive");
        frontRight = getMotor("right_front_drive");
        backLeft   = getMotor("left_back_drive");
        backRight  = getMotor("right_back_drive");

        if (frontLeft  != null) frontLeft.setDirection(DcMotor.Direction.REVERSE);
        if (backLeft   != null) backLeft.setDirection(DcMotor.Direction.REVERSE);
        if (frontRight != null) frontRight.setDirection(DcMotor.Direction.FORWARD);
        if (backRight  != null) backRight.setDirection(DcMotor.Direction.FORWARD);

        ballDetector.init(hardwareMap, telemetry);

        telemetry.addLine("Ball detector ready. Hold LB to aim at closest ball.");
    }

    @Override
    public void init_loop() {
        ballDetector.update();
        ballDetector.displayTelemetry();
        telemetry.update();
    }

    @Override
    public void loop() {
        ballDetector.update();

        double y  = -gamepad1.left_stick_y;   // forward = +1
        double x  =  gamepad1.left_stick_x;   // strafe right = +1
        double rx =  gamepad1.right_stick_x;  // rotate right = +1

        // Pick a target ball
        BallDetector.Ball target = null;
        if (gamepad1.x) {
            target = ballDetector.getClosestBall(BallDetector.BallColor.PURPLE);
        } else if (gamepad1.y) {
            target = ballDetector.getClosestBall(BallDetector.BallColor.GREEN);
        } else if (gamepad1.left_bumper) {
            target = ballDetector.getClosestBall();
        }

        boolean aiming = gamepad1.left_bumper || gamepad1.x || gamepad1.y;
        if (aiming && target != null) {
            // bearing > 0 means ball is to the LEFT, so turn left (negative rx)
            if (Math.abs(target.bearing) > AIM_TOLERANCE) {
                rx = Range.clip(-target.bearing * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
            } else {
                rx = 0;
            }
            telemetry.addLine(String.format("AIMING at %s ball: %.1f in, %.1f deg",
                    target.color, target.distance, target.bearing));
        } else if (aiming) {
            telemetry.addLine("AIMING: no ball in view");
        }

        double flPower = y + x + rx;
        double blPower = y - x + rx;
        double frPower = y - x - rx;
        double brPower = y + x - rx;

        double max = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        if (frontLeft  != null) frontLeft.setPower(flPower / max);
        if (backLeft   != null) backLeft.setPower(blPower / max);
        if (frontRight != null) frontRight.setPower(frPower / max);
        if (backRight  != null) backRight.setPower(brPower / max);

        ballDetector.displayTelemetry();
        telemetry.update();
    }

    @Override
    public void stop() {
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
