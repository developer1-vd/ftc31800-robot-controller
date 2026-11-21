package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.mechanisms.Intake;

/**
 * Intake test (goBILDA Yellow Jacket 312 RPM on "intake").
 *
 *   AUTORUN: the intake starts running IN (counterclockwise) as soon as Play is pressed.
 *   Y = turn it off / on, A = on, hold B = out, D-pad UP/DOWN = speed +/- 10%
 *   (gamepad1 or gamepad2, the same buttons as the teleop).
 *   X = RAW test: full power straight to the motor, skipping the Intake class. If X turns it but
 *   A doesn't, the problem is in the code; if X doesn't turn it either, it's wiring/configuration.
 * Telemetry: direction, RPM (with the encoder), current, jams cleared, encoder status.
 *
 * Check: it must pull a ball IN, counterclockwise. If not, flip Intake.IN_IS_COUNTERCLOCKWISE.
 * Jam test (needs the encoder): while it runs, stop the roller by hand (carefully): it should
 * reverse briefly and retry.
 */
@TeleOp(name = "IntakeTest", group = "Vision")
public class IntakeTest extends OpMode {

    private final Intake intake = new Intake();
    private double speed = 1.0;
    private boolean lastUp = false, lastDown = false, lastY = false;
    private boolean autoOn = false;            // set true at Play (autorun)
    private boolean found = false;
    private DcMotor raw = null;

    @Override
    public void init() {
        found = intake.init(hardwareMap, telemetry, "intake");   // explains what's wrong if not found
        if (found) {
            try { raw = hardwareMap.get(DcMotor.class, "intake"); } catch (Exception ignored) { }
        }
        telemetry.addLine("Press Play: the intake starts running IN by itself.");
        telemetry.addLine("Y = off/on, A = on, hold B = out, X = RAW full power, D-pad = speed.");
    }

    @Override
    public void start() {
        autoOn = true;   // AUTORUN from Play
    }

    @Override
    public void loop() {
        boolean up = gamepad1.dpad_up || gamepad2.dpad_up, down = gamepad1.dpad_down || gamepad2.dpad_down;
        boolean y = gamepad1.y || gamepad2.y;
        if (up && !lastUp)     speed = Math.min(1.0, speed + 0.1);
        if (down && !lastDown) speed = Math.max(0.1, speed - 0.1);
        if (y && !lastY)       autoOn = !autoOn;
        lastUp = up;
        lastDown = down;
        lastY = y;
        if (gamepad1.a || gamepad2.a) autoOn = true;

        if ((gamepad1.x || gamepad2.x) && raw != null) {
            raw.setPower(1.0);   // bypasses the Intake class completely
            telemetry.addLine("RAW: power 1.0 sent straight to \"intake\" (does it turn?)");
            telemetry.update();
            return;
        }
        if (gamepad1.b || gamepad2.b) intake.run(-speed);
        else if (autoOn)              intake.run(speed);
        else                          intake.stop();

        telemetry.addData("Autorun", autoOn ? "ON - running IN (Y = off)" : "OFF (Y or A = on)");
        telemetry.addData("Speed", "%.0f%%", speed * 100);
        if (!found) telemetry.addLine("⚠️ No intake motor found, so nothing can turn. See the line below.");
        intake.addTelemetry();
        if (intake.isEncoderMissing()) {
            telemetry.addLine("Encoder not detected: plug the motor's encoder cable into the SAME"
                    + " port number's encoder socket for jam clearing (it still runs without it).");
        }
        telemetry.update();
    }

    @Override
    public void stop() {
        intake.stop();
    }
}
