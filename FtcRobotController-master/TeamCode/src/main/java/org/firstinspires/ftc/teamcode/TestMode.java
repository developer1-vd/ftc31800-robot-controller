package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * TEST MODE: run every TeleOp, autonomous and test from one OpMode, and switch between them
 * without going back to the Driver Station's OpMode list.
 *
 *   Menu (INIT, or after leaving a mode): gamepad1 D-pad UP/DOWN to choose, A to start.
 *   While a mode runs: it works exactly like the real OpMode (same controls).
 *   Press BOTH JOYSTICK BUTTONS (click both sticks) on gamepad1 = stop everything and go back
 *   to the menu. The Driver Station's Stop button stops everything too.
 *
 * Autonomous modes drive on their own using the cameras, so test them with room around the
 * robot. Practice only: in matches, pick the real OpModes from the Driver Station list.
 */
@TeleOp(name = "TEST MODE (everything)", group = "AAA Test")
public class TestMode extends OpMode {

    /** One entry in the menu: a name and how to make that OpMode. */
    private static class Mode {
        final String name;
        final java.util.function.Supplier<OpMode> make;

        Mode(String name, java.util.function.Supplier<OpMode> make) {
            this.name = name;
            this.make = make;
        }
    }

    /** The camera collect-and-shoot auto with a 2-minute run instead of 30 s, for practice. */
    public static class PracticeAuto extends BallSpeakerAuto {
        @Override
        protected double autoLengthMs() {
            return 120000;
        }
    }

    private final Mode[] modes = {
            new Mode("Driver TeleOp (TeamBotTeleop_latest)",              TeamBotTeleop_latest::new),
            new Mode("Driver TeleOp + phone touch controls",              TeamBotTeleop_Phone::new),
            new Mode("Auto: BIOBUZZ RED (30 s, camera driving)",          BiobuzzAutoRed::new),
            new Mode("Auto: BIOBUZZ BLUE (30 s, camera driving)",         BiobuzzAutoBlue::new),
            new Mode("Auto: camera collect + shoot practice (2 min)",     PracticeAuto::new),
            new Mode("Test: balls + color calibration + aim",             BallDetectionTest::new),
            new Mode("Test: Limelight",                                   LimelightTest::new),
    };

    private int selected = 0;
    private OpMode running = null;
    private String runningName = "";
    private String message = "";
    private boolean lastUp = false, lastDown = false, lastA = false;

    @Override
    public void init() {
        message = "Choose a mode, then press Play (or A after Play).";
    }

    @Override
    public void init_loop() {
        menu(false);
    }

    @Override
    public void start() {
        startSelected();
    }

    @Override
    public void loop() {
        if (running == null) {
            menu(true);
            return;
        }
        // Exit: both joystick buttons on gamepad1
        if (gamepad1.left_stick_button && gamepad1.right_stick_button) {
            stopRunning("Stopped " + runningName + ".");
            menu(true);
            return;
        }
        telemetry.addData("TEST MODE", "%s   (click both sticks = menu)", runningName);
        try {
            running.loop();
        } catch (Exception e) {
            stopRunning("⚠️ " + runningName + " crashed: " + e);
        }
    }

    @Override
    public void stop() {
        stopRunning("");
    }

    // -------------------------------------------------

    /** Menu: D-pad up/down to choose; A (only after Play) to start. */
    private void menu(boolean canStart) {
        boolean up = gamepad1.dpad_up, down = gamepad1.dpad_down, a = gamepad1.a;
        if (up && !lastUp)     selected = (selected + modes.length - 1) % modes.length;
        if (down && !lastDown) selected = (selected + 1) % modes.length;
        boolean pressedA = a && !lastA;
        lastUp = up;
        lastDown = down;
        lastA = a;

        telemetry.addLine("=== TEST MODE: run everything ===");
        telemetry.addLine("D-pad UP/DOWN choose, " + (canStart ? "A start" : "Play start")
                + ". While running: click BOTH sticks = back here.");
        for (int i = 0; i < modes.length; i++) {
            telemetry.addLine((i == selected ? " >> " : "    ") + modes[i].name);
        }
        if (!message.isEmpty()) telemetry.addLine(message);
        telemetry.addLine("Autonomous modes drive by themselves: keep clear of the robot.");
        telemetry.update();

        if (canStart && pressedA) startSelected();
    }

    /** Make the chosen OpMode, hand it this OpMode's hardware, telemetry and gamepads, start it. */
    private void startSelected() {
        Mode mode = modes[selected];
        try {
            OpMode op = mode.make.get();
            op.hardwareMap = hardwareMap;
            op.telemetry = telemetry;
            op.gamepad1 = gamepad1;
            op.gamepad2 = gamepad2;
            running = op;
            runningName = mode.name;
            op.init();
            op.start();
            message = "";
        } catch (Exception e) {
            stopRunning("⚠️ Couldn't start " + mode.name + ": " + e);
        }
        // Don't let the A press that started it also count inside the mode
        lastA = true;
    }

    /** Stop the running mode, then make sure every motor and CR servo is stopped. */
    private void stopRunning(String why) {
        if (running != null) {
            try {
                running.stop();
            } catch (Exception ignored) {
                // still stop the motors below
            }
            running = null;
        }
        for (DcMotorSimple m : hardwareMap.getAll(DcMotorSimple.class)) {
            try {
                m.setPower(0);
            } catch (Exception ignored) {
            }
        }
        if (!why.isEmpty()) message = why;
    }
}
