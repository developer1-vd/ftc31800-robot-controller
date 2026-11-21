package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.mechanisms.BallColorCalibration;
import org.firstinspires.ftc.teamcode.mechanisms.BallDetector;
import org.firstinspires.ftc.teamcode.mechanisms.CenterColorSampler;
import org.firstinspires.ftc.teamcode.mechanisms.LimelightVision;
import org.firstinspires.ftc.teamcode.mechanisms.LookAroundScanner;

/**
 * Test OpMode for camera-based ball detection.
 *
 * - Shows every detected ball (color, distance, bearing) on telemetry.
 * - Mecanum drive on gamepad1.
 * - Hold gamepad1 LEFT BUMPER to auto-turn toward the closest ball.
 *   Hold X / B / Y for the closest BLUE / RED / YELLOW ball (matches the button colors).
 * - Driver Station says the ball color when one comes into view; gamepad1 BACK mutes/unmutes.
 *
 * Limelight 3A (named "limelight" in the configuration): used instead of the webcam. It runs one
 * color pipeline at a time: yellow POLLEN by default, blue/red/yellow while X/B/Y is held.
 * Good for tuning each Limelight color pipeline.
 *
 * BALL COLOR CALIBRATION (webcam): if balls aren't detected, the colors need calibrating for
 * your camera and lighting. Telemetry shows the color in the white square at the center of the
 * camera view (Driver Station: ... menu -> Camera Stream to see it) and which ball colors it
 * matches. Hold a ball so it fills the square, then press on gamepad1:
 *   D-pad UP = save as YELLOW POLLEN, RIGHT = save as RED NECTAR, LEFT = save as BLUE NECTAR,
 *   D-pad DOWN = forget all calibration (back to the SDK default colors).
 * The colors are saved on the Control Hub and used by every OpMode; restart the OpMode to apply.
 *
 * Also use this to tune BallDetector: check radius/distance at known distances
 * and adjust the focal length and blob filters.
 */
@TeleOp(name = "BallDetectionTest", group = "Vision")
public class BallDetectionTest extends OpMode {

    private final BallDetector ballDetector = new BallDetector();
    private LimelightVision limelight = null;
    private final CenterColorSampler sampler = new CenterColorSampler();
    private boolean lastDpad = false;
    private final LookAroundScanner ballScanner = new LookAroundScanner();
    private String calibrationMessage = "";

    private DcMotor frontLeft  = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft   = null;
    private DcMotor backRight  = null;

    // Proportional turn gain (power per degree of bearing) and limits
    private static final double TURN_GAIN      = 0.02;
    private static final double MAX_AUTO_TURN  = 0.4;
    private static final double AIM_TOLERANCE  = 2.0;  // degrees

    // Spoken ball announcements; set to false to start silent. gamepad1 BACK toggles.
    private static final boolean SOUND_ENABLED = true;
    private boolean lastSoundToggle = false;

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

        // Balls come from Webcam 1 when it's configured (even with a Limelight, which is then
        // left for AprilTags); otherwise from the Limelight's color pipelines.
        limelight = BallDetector.webcamConfigured(hardwareMap) ? null
                : LimelightVision.tryCreate(hardwareMap, telemetry);
        if (limelight != null) {
            ballDetector.useLimelight(limelight, telemetry);
            telemetry.addLine("Using Limelight 3A");
        } else {
            try {
                ballDetector.init(hardwareMap, telemetry, sampler);
            } catch (Exception e) {
                telemetry.addLine("⚠️ No camera found: no Limelight 3A and no \"Webcam 1\" in the robot"
                        + " configuration. Run LimelightTest for setup help.");
            }
        }
        ballDetector.setSoundEnabled(SOUND_ENABLED);

        if (frontLeft == null && frontRight == null && backLeft == null && backRight == null) {
            telemetry.addLine("No drive motors found - camera-only mode (detects and speaks).");
        } else {
            telemetry.addLine("Ball detector ready. Hold LB to aim at closest ball.");
        }
    }

    @Override
    public void init_loop() {
        updateVision();
        colorCalibration();
        ballDetector.displayTelemetry();
        telemetry.update();
    }

    @Override
    public void loop() {
        updateVision();

        // Sound on/off: gamepad1 BACK (one toggle per press)
        if (gamepad1.back && !lastSoundToggle) {
            ballDetector.setSoundEnabled(!ballDetector.isSoundEnabled());
            telemetry.speak(ballDetector.isSoundEnabled() ? "sound on" : "sound off");
        }
        lastSoundToggle = gamepad1.back;

        double y  = -gamepad1.left_stick_y;   // forward = +1
        double x  =  gamepad1.left_stick_x;   // strafe right = +1
        double rx =  gamepad1.right_stick_x;  // rotate right = +1

        // Pick a target ball
        BallDetector.Ball target = null;
        if (gamepad1.x) {
            target = ballDetector.getClosestBall(BallDetector.BallColor.BLUE);
        } else if (gamepad1.b) {
            target = ballDetector.getClosestBall(BallDetector.BallColor.RED);
        } else if (gamepad1.y) {
            target = ballDetector.getClosestBall(BallDetector.BallColor.YELLOW);
        } else if (gamepad1.left_bumper) {
            target = ballDetector.getClosestBall();
        }

        boolean aiming = gamepad1.left_bumper || gamepad1.x || gamepad1.b || gamepad1.y;
        if (aiming && target != null) {
            ballScanner.reset();
            // bearing > 0 means ball is to the LEFT, so turn left (negative rx)
            if (Math.abs(target.bearing) > AIM_TOLERANCE) {
                rx = Range.clip(-target.bearing * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
            } else {
                rx = 0;
            }
            telemetry.addLine(String.format("AIMING at %s ball: %.1f in, %.1f deg",
                    target.color, target.distance, target.bearing));
        } else if (aiming && Math.abs(gamepad1.right_stick_x) < 0.1) {
            rx = ballScanner.turnPower();   // turn to look
            telemetry.addLine("AIMING: no ball in view - turning to look");
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

        telemetry.addData("Sound", ballDetector.isSoundEnabled() ? "ON (BACK to mute)" : "OFF (BACK to unmute)");
        colorCalibration();
        ballDetector.displayTelemetry();
        telemetry.update();
    }

    @Override
    public void stop() {
        if (limelight != null) {
            limelight.stop();
        } else {
            ballDetector.stop();
        }
    }

    /** With a Limelight: pick the color pipeline from the held button (default yellow POLLEN). */
    private void updateVision() {
        if (limelight != null) {
            BallDetector.BallColor color = BallDetector.BallColor.YELLOW;
            if (gamepad1.x)      color = BallDetector.BallColor.BLUE;
            else if (gamepad1.b) color = BallDetector.BallColor.RED;
            limelight.setMode(LimelightVision.Mode.BALLS, color);
            limelight.update();
            limelight.displayTelemetry();
        }
        ballDetector.update();
    }

    /** Live center color readout, and D-pad ball color calibration (webcam only). */
    private void colorCalibration() {
        if (ballDetector.usingLimelight()) {
            telemetry.addLine("Color tuning: use the Limelight web page (color pipelines 1-3).");
            return;
        }
        double[] mean = sampler.getMean(), std = sampler.getStd();
        if (mean == null) {
            telemetry.addLine("Color sampler: waiting for camera frames...");
            return;
        }
        StringBuilder matches = new StringBuilder();
        for (BallDetector.BallColor c : BallDetector.BallColor.values()) {
            if (ballDetector.getColorRange(c).contains(mean[0], mean[1], mean[2])) {
                matches.append(matches.length() > 0 ? ", " : "").append(c);
            }
        }
        telemetry.addLine("== Ball color calibration (white square, center of view) ==");
        telemetry.addData("Center color", "Y %.0f  Cr %.0f  Cb %.0f   matches: %s",
                mean[0], mean[1], mean[2], matches.length() > 0 ? matches : "NONE");
        for (BallDetector.BallColor c : BallDetector.BallColor.values()) {
            telemetry.addData("  " + c, "%s%s", ballDetector.getColorRange(c),
                    BallColorCalibration.isCalibrated(c) ? " (calibrated)" : " (SDK default)");
        }
        telemetry.addLine("Fill the square with a ball, D-pad: UP yellow, RIGHT red, LEFT blue, DOWN reset");

        // One action per D-pad press
        boolean up = gamepad1.dpad_up, right = gamepad1.dpad_right, left = gamepad1.dpad_left,
                down = gamepad1.dpad_down;
        boolean any = up || right || left || down;
        if (any && !lastDpad) {
            try {
                if (down) {
                    BallColorCalibration.reset();
                    calibrationMessage = "Calibration cleared (SDK defaults). Restart the OpMode to apply.";
                } else {
                    BallDetector.BallColor color = up ? BallDetector.BallColor.YELLOW
                            : right ? BallDetector.BallColor.RED : BallDetector.BallColor.BLUE;
                    BallColorCalibration.Range range = BallColorCalibration.rangeFrom(mean, std);
                    BallColorCalibration.save(color, range);
                    calibrationMessage = "Saved " + color + ": " + range + ". Restart the OpMode to apply.";
                    telemetry.speak(color.name().toLowerCase() + " saved");
                }
            } catch (Exception e) {
                calibrationMessage = "Could not save: " + e.getMessage();
            }
        }
        lastDpad = any;
        if (!calibrationMessage.isEmpty()) telemetry.addLine(calibrationMessage);
    }

    private DcMotor getMotor(String name) {
        try {
            return hardwareMap.get(DcMotor.class, name);
        } catch (Exception e) {
            return null;
        }
    }
}
