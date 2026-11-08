package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.mechanisms.Intake;

/**
 * Intake test (goBILDA Yellow Jacket 312 RPM on "intake").
 *
 *   gamepad1 A = in, B = out (hold), Y = in at full speed (for the jam test),
 *   D-pad UP/DOWN = speed +/- 10%.
 *   X = RAW test: full power straight to the motor, skipping the Intake class. If X turns it but
 *   A doesn't, the problem is in the code; if X doesn't turn it either, it's wiring/configuration.
 * Telemetry: direction, RPM (with the encoder), current, jams cleared, encoder status.
 *
 * Check: A must pull a ball IN. If it pushes it out, flip Intake.INTAKE_DIRECTION.
 * Jam test: hold Y and stop the roller by hand (carefully): it should reverse briefly and retry.
 */
@TeleOp(name = "IntakeTest", group = "Vision")
public class IntakeTest extends OpMode {

    private final Intake intake = new Intake();
    private double speed = 1.0;
    private boolean lastUp = false, lastDown = false;
    private boolean found = false;
    private DcMotor raw = null;

    @Override
    public void init() {
        found = intake.init(hardwareMap, telemetry, "intake");   // explains what's wrong if not found
        if (found) {
            try { raw = hardwareMap.get(DcMotor.class, "intake"); } catch (Exception ignored) { }
        }
        telemetry.addLine("A = in, B = out, Y = full in (jam test), X = RAW full power, D-pad = speed.");
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up && !lastUp)     speed = Math.min(1.0, speed + 0.1);
        if (gamepad1.dpad_down && !lastDown) speed = Math.max(0.1, speed - 0.1);
        lastUp = gamepad1.dpad_up;
        lastDown = gamepad1.dpad_down;

        if (gamepad1.x && raw != null) {
            raw.setPower(1.0);   // bypasses the Intake class completely
            telemetry.addLine("RAW: power 1.0 sent straight to \"intake\" (does it turn?)");
            telemetry.update();
            return;
        }
        if (gamepad1.y)      intake.run(1.0);
        else if (gamepad1.a) intake.run(speed);
        else if (gamepad1.b) intake.run(-speed);
        else                 intake.stop();

        telemetry.addData("Speed", "%.0f%% of %.0f RPM", speed * 100, Intake.IN_RPM);
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
