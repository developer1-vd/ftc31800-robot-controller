package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.mechanisms.ShotPlanner;

/**
 * Flywheel test for the "launcher" motor (and the two feeder servos, if present).
 * Gamepad1 or gamepad2:
 *
 *   A = spin up to the target RPM        B = stop
 *   D-pad UP / DOWN = target +/- 50 RPM  D-pad LEFT / RIGHT = previous / next preset
 *       (presets = ShotPlanner's distance table, plus the teleop's 3150 RPM preset)
 *   LB / RB = test shot: run the LEFT / RIGHT feeder for 1 s (only once the wheel is at speed)
 *   hold Y = TOP SPEED test: full power, shows the highest RPM it reaches
 *   X = clear the measurements
 *
 * Telemetry: target and actual RPM, error, how long it took to reach speed, how steady it is
 * (lowest / highest RPM over the last second), current, and for each test shot how far the
 * speed dropped and how long it took to recover. It also warns about a missing encoder cable
 * or a flywheel spinning backwards.
 */
@TeleOp(name = "FlywheelTest", group = "Vision")
public class FlywheelTest extends OpMode {

    // Same as the teleop and FeederLauncher
    private static final double TICKS_PER_REV   = 28.0;    // encoder ticks per flywheel motor turn
    private static final double TOLERANCE_RPM   = 50;      // "at speed" = within this
    private static final int    AT_SPEED_FRAMES = 5;       // ...for this many loops in a row
    private static final double FEED_MS         = 1000;    // feeder run per test shot
    private static final double FEED_POWER      = -1.0;    // feeder direction that pushes the ball in
    private static final double STEP_RPM        = 50;
    private static final double MAX_TARGET_RPM  = 6000;

    private DcMotorEx flywheel = null;
    private CRServo feederLeft = null, feederRight = null;

    private double[] presets;
    private int presetIndex = 1;
    private double targetRpm;

    private enum Mode { STOPPED, SPINNING, TOP_SPEED }
    private Mode mode = Mode.STOPPED;

    // Measurements
    private final ElapsedTime spinUpTimer = new ElapsedTime();
    private double spinUpSec = -1;          // time to reach speed, -1 = not yet
    private int atSpeedCount = 0;
    private double topRpm = 0;
    private final double[] window = new double[256];   // RPM samples for the last second
    private final double[] windowMs = new double[256];
    private int windowCount = 0, windowNext = 0;
    private int lastPosition = 0;
    private final ElapsedTime noMoveTimer = new ElapsedTime();
    private boolean encoderWarning = false;

    // Test shot
    private CRServo shotFeeder = null;
    private String shotSide = "";
    private final ElapsedTime shotTimer = new ElapsedTime();
    private boolean shotRunning = false;
    private double shotStartRpm = 0, shotLowestRpm = 0;
    private boolean shotRecovered = true;
    private String lastShot = "none yet";

    private boolean lastA, lastB, lastX, lastUp, lastDown, lastLeft, lastRight, lastLb, lastRb;

    @Override
    public void init() {
        // Presets: the ShotPlanner table (distance -> RPM), then the teleop's long-shot preset
        presets = new double[ShotPlanner.RPM_TABLE.length + 1];
        for (int i = 0; i < ShotPlanner.RPM_TABLE.length; i++) presets[i] = ShotPlanner.RPM_TABLE[i][1];
        presets[presets.length - 1] = 3150;
        presetIndex = Math.min(1, presets.length - 1);
        targetRpm = presets[presetIndex];

        try {
            flywheel = hardwareMap.get(DcMotorEx.class, "launcher");
            flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);   // coast down
            flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            flywheel.setDirection(DcMotor.Direction.FORWARD);                 // same as the teleop
        } catch (Exception e) {
            flywheel = null;
        }
        try { feederLeft = hardwareMap.get(CRServo.class, "feederServoLeft"); } catch (Exception e) { feederLeft = null; }
        try { feederRight = hardwareMap.get(CRServo.class, "feederServoRight"); } catch (Exception e) { feederRight = null; }
        if (feederLeft != null) feederLeft.setPower(0);
        if (feederRight != null) feederRight.setPower(0);

        telemetry.addData("launcher (flywheel)", flywheel != null ? "OK" : "MISSING: no motor named \"launcher\"");
        telemetry.addData("feederServoLeft", feederLeft != null ? "OK" : "missing (no left test shots)");
        telemetry.addData("feederServoRight", feederRight != null ? "OK" : "missing (no right test shots)");
        telemetry.addLine("Keep hands and balls clear of the flywheel.");
        telemetry.addLine("A = spin, B = stop, D-pad = RPM / presets, LB/RB = test shot, hold Y = top speed, X = clear.");
    }

    @Override
    public void loop() {
        if (flywheel == null) {
            telemetry.addLine("⚠️ No motor named \"launcher\" in the robot configuration: nothing to test.");
            telemetry.update();
            return;
        }

        boolean a = gamepad1.a || gamepad2.a, b = gamepad1.b || gamepad2.b, x = gamepad1.x || gamepad2.x;
        boolean y = gamepad1.y || gamepad2.y;
        boolean up = gamepad1.dpad_up || gamepad2.dpad_up, down = gamepad1.dpad_down || gamepad2.dpad_down;
        boolean left = gamepad1.dpad_left || gamepad2.dpad_left, right = gamepad1.dpad_right || gamepad2.dpad_right;
        boolean lb = gamepad1.left_bumper || gamepad2.left_bumper, rb = gamepad1.right_bumper || gamepad2.right_bumper;

        // ---- Target RPM ----
        if (up && !lastUp)       setTarget(targetRpm + STEP_RPM);
        if (down && !lastDown)   setTarget(targetRpm - STEP_RPM);
        if (right && !lastRight) { presetIndex = Math.min(presets.length - 1, presetIndex + 1); setTarget(presets[presetIndex]); }
        if (left && !lastLeft)   { presetIndex = Math.max(0, presetIndex - 1); setTarget(presets[presetIndex]); }

        // ---- Start / stop / top speed ----
        if (a && !lastA) startSpin();
        if (b && !lastB) stopAll();
        if (x && !lastX) clearMeasurements();
        if (y && mode != Mode.TOP_SPEED) {
            mode = Mode.TOP_SPEED;
            flywheel.setPower(1.0);
        } else if (!y && mode == Mode.TOP_SPEED) {
            flywheel.setPower(0);
            mode = Mode.STOPPED;
        }
        if (mode == Mode.SPINNING) flywheel.setVelocity(targetRpm * TICKS_PER_REV / 60.0);

        // ---- Measurements ----
        double rpm = flywheel.getVelocity() * 60.0 / TICKS_PER_REV;
        double nowMs = System.nanoTime() / 1e6;
        addSample(rpm, nowMs);
        topRpm = Math.max(topRpm, rpm);
        boolean atSpeed = mode == Mode.SPINNING && Math.abs(rpm - targetRpm) <= TOLERANCE_RPM;
        atSpeedCount = atSpeed ? atSpeedCount + 1 : 0;
        boolean ready = atSpeedCount >= AT_SPEED_FRAMES;
        if (ready && spinUpSec < 0) spinUpSec = spinUpTimer.seconds();
        checkEncoder(rpm);

        // ---- Test shots ----
        if (lb && !lastLb) startShot(feederLeft, "LEFT", ready);
        if (rb && !lastRb) startShot(feederRight, "RIGHT", ready);
        updateShot(rpm);

        lastA = a; lastB = b; lastX = x; lastUp = up; lastDown = down;
        lastLeft = left; lastRight = right; lastLb = lb; lastRb = rb;

        // ---- Telemetry ----
        switch (mode) {
            case SPINNING:
                telemetry.addData("Flywheel", ready ? "AT SPEED - ready to shoot" : "spinning up...");
                break;
            case TOP_SPEED:
                telemetry.addData("Flywheel", "TOP SPEED TEST (full power, release Y to stop)");
                break;
            default:
                telemetry.addData("Flywheel", "stopped (A = spin up)");
        }
        telemetry.addData("Target", "%.0f RPM  (preset %d of %d)", targetRpm, presetIndex + 1, presets.length);
        telemetry.addData("Now", "%.0f RPM  (error %+.0f)", rpm, rpm - targetRpm);
        telemetry.addData("Spin-up time", spinUpSec < 0 ? (mode == Mode.SPINNING ? "%.1f s so far" : "-")
                : "%.2f s", spinUpSec < 0 ? spinUpTimer.seconds() : spinUpSec);
        double[] lowHigh = windowLowHigh(nowMs);
        if (lowHigh != null) {
            telemetry.addData("Steadiness (last 1 s)", "%.0f to %.0f RPM  (spread %.0f)",
                    lowHigh[0], lowHigh[1], lowHigh[1] - lowHigh[0]);
        }
        telemetry.addData("Top speed seen", "%.0f RPM", topRpm);
        telemetry.addData("Current", "%.1f A", amps());
        telemetry.addData("Last test shot", lastShot);
        if (encoderWarning) {
            telemetry.addLine("⚠️ Powered but the encoder isn't counting: plug the launcher's encoder"
                    + " cable into the SAME port number's encoder socket (speed control needs it).");
        }
        if (mode != Mode.STOPPED && rpm < -100) {
            telemetry.addLine("⚠️ The flywheel reads BACKWARDS: if it really spins the wrong way, change"
                    + " the launcher's direction (teleop, FeederLauncher and this test all use FORWARD).");
        }
        telemetry.addLine("A spin | B stop | D-pad ±50 / presets | LB/RB test shot | hold Y top speed | X clear");
        telemetry.update();
    }

    // -------------------------------------------------

    private void setTarget(double rpm) {
        targetRpm = Math.max(0, Math.min(MAX_TARGET_RPM, rpm));
        if (mode == Mode.SPINNING) {   // measure the spin-up to the new speed
            spinUpTimer.reset();
            spinUpSec = -1;
            atSpeedCount = 0;
        }
    }

    private void startSpin() {
        mode = Mode.SPINNING;
        spinUpTimer.reset();
        spinUpSec = -1;
        atSpeedCount = 0;
        noMoveTimer.reset();
        lastPosition = flywheel.getCurrentPosition();
    }

    private void stopAll() {
        mode = Mode.STOPPED;
        flywheel.setPower(0);
        stopFeeders();
        shotRunning = false;
    }

    private void clearMeasurements() {
        topRpm = 0;
        windowCount = 0;
        windowNext = 0;
        lastShot = "none yet";
        encoderWarning = false;
        if (mode == Mode.SPINNING) {
            spinUpTimer.reset();
            spinUpSec = -1;
        }
    }

    private void startShot(CRServo feeder, String side, boolean ready) {
        if (shotRunning) return;
        if (feeder == null) {
            lastShot = side + ": no feeder servo";
            return;
        }
        if (!ready) {
            lastShot = side + ": wait until the flywheel is AT SPEED (press A first)";
            return;
        }
        shotFeeder = feeder;
        shotSide = side;
        shotRunning = true;
        shotRecovered = false;
        shotStartRpm = flywheel.getVelocity() * 60.0 / TICKS_PER_REV;
        shotLowestRpm = shotStartRpm;
        shotTimer.reset();
        feeder.setPower(FEED_POWER);
    }

    /** Track the speed drop from a test shot and how long it takes to get back to speed. */
    private void updateShot(double rpm) {
        if (shotRunning && shotTimer.milliseconds() > FEED_MS) {
            shotFeeder.setPower(0);
            shotRunning = false;
        }
        if (shotRecovered) return;
        shotLowestRpm = Math.min(shotLowestRpm, rpm);
        double drop = shotStartRpm - shotLowestRpm;
        boolean dipped = drop > TOLERANCE_RPM;
        if (dipped && Math.abs(rpm - targetRpm) <= TOLERANCE_RPM) {
            shotRecovered = true;
            lastShot = String.format("%s: dropped %.0f RPM, back to speed in %.2f s",
                    shotSide, drop, shotTimer.seconds());
        } else if (!shotRunning && shotTimer.milliseconds() > FEED_MS + 3000) {
            shotRecovered = true;
            lastShot = dipped
                    ? String.format("%s: dropped %.0f RPM, NOT back to speed after 4 s", shotSide, drop)
                    : String.format("%s: no speed drop seen (was a ball fed?)", shotSide);
        } else {
            lastShot = String.format("%s: shooting... drop so far %.0f RPM", shotSide, drop);
        }
    }

    private void stopFeeders() {
        if (feederLeft != null) feederLeft.setPower(0);
        if (feederRight != null) feederRight.setPower(0);
    }

    /** Powered for 0.8 s and the encoder count never changed = encoder cable missing. */
    private void checkEncoder(double rpm) {
        if (mode == Mode.STOPPED) {
            noMoveTimer.reset();
            lastPosition = flywheel.getCurrentPosition();
            return;
        }
        int pos = flywheel.getCurrentPosition();
        if (pos != lastPosition) {
            lastPosition = pos;
            noMoveTimer.reset();
            encoderWarning = false;
        } else if (noMoveTimer.milliseconds() > 800) {
            encoderWarning = true;
        }
    }

    private void addSample(double rpm, double nowMs) {
        window[windowNext] = rpm;
        windowMs[windowNext] = nowMs;
        windowNext = (windowNext + 1) % window.length;
        windowCount = Math.min(windowCount + 1, window.length);
    }

    /** Lowest and highest RPM in the last second, or null with too few samples. */
    private double[] windowLowHigh(double nowMs) {
        double lo = Double.MAX_VALUE, hi = -Double.MAX_VALUE;
        int n = 0;
        for (int i = 0; i < windowCount; i++) {
            if (nowMs - windowMs[i] <= 1000) {
                lo = Math.min(lo, window[i]);
                hi = Math.max(hi, window[i]);
                n++;
            }
        }
        return n < 3 ? null : new double[]{lo, hi};
    }

    private double amps() {
        try {
            return flywheel.getCurrent(CurrentUnit.AMPS);
        } catch (Exception e) {
            return 0;
        }
    }

    @Override
    public void stop() {
        if (flywheel != null) flywheel.setPower(0);
        stopFeeders();
    }
}
