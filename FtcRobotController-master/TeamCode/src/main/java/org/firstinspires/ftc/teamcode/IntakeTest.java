package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.Intake;

/**
 * Intake test (goBILDA Yellow Jacket 312 RPM on "intake").
 *
 *   gamepad1 A = in, B = out (hold), Y = in at full speed (for the jam test),
 *   D-pad UP/DOWN = speed +/- 10%.
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

    @Override
    public void init() {
        if (!intake.init(hardwareMap, telemetry, "intake")) {
            telemetry.addLine("No \"intake\" motor in the robot configuration.");
        }
        telemetry.addLine("A = in, B = out, Y = full in (jam test), D-pad up/down = speed.");
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up && !lastUp)     speed = Math.min(1.0, speed + 0.1);
        if (gamepad1.dpad_down && !lastDown) speed = Math.max(0.1, speed - 0.1);
        lastUp = gamepad1.dpad_up;
        lastDown = gamepad1.dpad_down;

        if (gamepad1.y)      intake.run(1.0);
        else if (gamepad1.a) intake.run(speed);
        else if (gamepad1.b) intake.run(-speed);
        else                 intake.stop();

        telemetry.addData("Speed", "%.0f%% of %.0f RPM", speed * 100, Intake.IN_RPM);
        intake.addTelemetry();
        if (intake.isEncoderMissing()) {
            telemetry.addLine("Encoder not detected: plug the motor's encoder cable into the hub's"
                    + " encoder port for speed control and jam clearing.");
        }
        telemetry.update();
    }

    @Override
    public void stop() {
        intake.stop();
    }
}
