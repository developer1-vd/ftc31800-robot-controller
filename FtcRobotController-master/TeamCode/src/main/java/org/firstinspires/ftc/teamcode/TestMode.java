package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedroPathing.BiobuzzPathAutoBlue;
import org.firstinspires.ftc.teamcode.pedroPathing.BiobuzzPathAutoRed;
import org.firstinspires.ftc.teamcode.pedroPathing.BlueTeamBotAuto;
import org.firstinspires.ftc.teamcode.pedroPathing.BlueTeamBotAutoFarShort;
import org.firstinspires.ftc.teamcode.pedroPathing.BlueTeamBotAutoNearShort;
import org.firstinspires.ftc.teamcode.pedroPathing.CallbackExampleAuto;
import org.firstinspires.ftc.teamcode.pedroPathing.ExampleAuto;
import org.firstinspires.ftc.teamcode.pedroPathing.ExampleSquarePathAuto;
import org.firstinspires.ftc.teamcode.pedroPathing.RedTeamBotAuto;
import org.firstinspires.ftc.teamcode.pedroPathing.RedTeamBotAutoFarLong;
import org.firstinspires.ftc.teamcode.pedroPathing.RedTeamBotAutoFarShort;
import org.firstinspires.ftc.teamcode.pedroPathing.RedTeamBotAutoNearShort;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

/**
 * TEST MODE: run every TeleOp, autonomous and test from one OpMode, one at a time or all in a row.
 *
 * Menu (gamepad1):
 *   D-pad UP/DOWN choose, A start the chosen mode (or Play at INIT)
 *   Y = RUN ALL: every mode, one after another
 *   X = RUN ALL BIOBUZZ only (skips last season's DECODE autos)
 * While a mode runs it works exactly like the real OpMode (same controls), plus:
 *   click the LEFT stick       = skip to the next mode (RUN ALL)
 *   click BOTH sticks          = stop everything and go back to the menu
 * Between modes every motor and CR servo is set to 0. A crash in a mode is caught and reported.
 *
 * Not included: DriveTrain_Code and RobotAutoDriveByEncoder_Linear_Modified are LinearOpModes,
 * which have to own the whole run and can't be hosted inside another OpMode.
 * Autonomous modes drive by themselves: keep clear. Practice only; use the real OpModes in matches.
 */
@TeleOp(name = "TEST MODE (everything)", group = "AAA Test")
public class TestMode extends OpMode {

    private static final double TELEOP_MS = 30000, AUTO_MS = 30000, PRACTICE_MS = 45000, TEST_MS = 30000;

    /** One entry in the menu. */
    private static class Mode {
        final String name;
        final Supplier<OpMode> make;
        final double runAllMs;     // how long RUN ALL gives it
        final boolean old;         // last season (DECODE): skipped by "RUN ALL BIOBUZZ only"

        Mode(String name, Supplier<OpMode> make, double runAllMs, boolean old) {
            this.name = name;
            this.make = make;
            this.runAllMs = runAllMs;
            this.old = old;
        }
    }

    /** The camera collect-and-shoot auto with a 2-minute run instead of 30 s, for practice. */
    public static class PracticeAuto extends BallSpeakerAuto {
        @Override
        protected double autoLengthMs() {
            return 120000;
        }
    }

    private Mode[] modes = {
            // ---- This season (BIOBUZZ) ----
            new Mode("Driver TeleOp (TeamBotTeleop_latest)",          TeamBotTeleop_latest::new,  TELEOP_MS, false),
            new Mode("Driver TeleOp + phone touch controls",          TeamBotTeleop_Phone::new,   TELEOP_MS, false),
            new Mode("Auto: BIOBUZZ RED (camera driving)",            BiobuzzAutoRed::new,        AUTO_MS, false),
            new Mode("Auto: BIOBUZZ BLUE (camera driving)",           BiobuzzAutoBlue::new,       AUTO_MS, false),
            new Mode("Auto: BIOBUZZ Simple RED (leave, shoot, park)", BiobuzzSimpleAutoRed::new,  AUTO_MS, false),
            new Mode("Auto: BIOBUZZ Simple BLUE (leave, shoot, park)",BiobuzzSimpleAutoBlue::new, AUTO_MS, false),
            new Mode("Auto: BIOBUZZ Path RED (Pedro paths)",          BiobuzzPathAutoRed::new,    AUTO_MS, false),
            new Mode("Auto: BIOBUZZ Path BLUE (Pedro paths)",         BiobuzzPathAutoBlue::new,   AUTO_MS, false),
            new Mode("Auto: BallSpeakerAuto (30 s)",                  BallSpeakerAuto::new,       AUTO_MS, false),
            new Mode("Auto: camera collect + shoot practice (2 min)", PracticeAuto::new,          PRACTICE_MS, false),
            new Mode("Auto: camera collect + shoot, endless",         PracticeAutoEndless::new,   PRACTICE_MS, false),
            new Mode("Test: balls + color calibration + aim",         BallDetectionTest::new,     TEST_MS, false),
            new Mode("Test: Limelight",                               LimelightTest::new,         TEST_MS, false),
            // ---- Older TeleOps ----
            new Mode("Old TeleOp: TeamBotTeleop",                     TeamBotTeleop::new,         TELEOP_MS, false),
            new Mode("Old TeleOp: TeamBotTeleop_analytics",           TeamBotTeleop_analytics::new, TELEOP_MS, false),
            // ---- Last season (DECODE) autos: drive DECODE paths ----
            new Mode("OLD DECODE auto: TeamBotAuto",                  TeamBotAuto::new,           AUTO_MS, true),
            new Mode("OLD DECODE auto: RedTeamBotAutoSimple",         RedTeamBotAutoSimple::new,  AUTO_MS, true),
            new Mode("OLD DECODE auto: BlueTeamBotAutoSimple",        BlueTeamBotAutoSimple::new, AUTO_MS, true),
            new Mode("OLD DECODE auto: RedTeamBotAuto",               RedTeamBotAuto::new,        AUTO_MS, true),
            new Mode("OLD DECODE auto: BlueTeamBotAuto",              BlueTeamBotAuto::new,       AUTO_MS, true),
            new Mode("OLD DECODE auto: RedTeamBotAutoNearShort",      RedTeamBotAutoNearShort::new, AUTO_MS, true),
            new Mode("OLD DECODE auto: BlueTeamBotAutoNearShort",     BlueTeamBotAutoNearShort::new, AUTO_MS, true),
            new Mode("OLD DECODE auto: RedTeamBotAutoFarShort",       RedTeamBotAutoFarShort::new, AUTO_MS, true),
            new Mode("OLD DECODE auto: BlueTeamBotAutoFarShort",      BlueTeamBotAutoFarShort::new, AUTO_MS, true),
            new Mode("OLD DECODE auto: RedTeamBotAutoFarLong",        RedTeamBotAutoFarLong::new, AUTO_MS, true),
            // ---- Pedro Pathing examples ----
            new Mode("Pedro example: Example Auto",                   ExampleAuto::new,           AUTO_MS, false),
            new Mode("Pedro example: Callback Example Auto",          CallbackExampleAuto::new,   AUTO_MS, false),
            new Mode("Pedro example: Square Path Auto",               ExampleSquarePathAuto::new, AUTO_MS, false),
    };

    private int selected = 0;
    private OpMode running = null;
    private String runningName = "";
    private String message = "";
    private boolean lastUp, lastDown, lastA, lastX, lastY;

    // RUN ALL
    private boolean runAll = false;
    private boolean runAllIncludesOld = true;
    private int runIndex = -1;
    private final com.qualcomm.robotcore.util.ElapsedTime modeTimer = new com.qualcomm.robotcore.util.ElapsedTime();
    private final List<String> results = new ArrayList<>();
    private boolean leftStickHeld = false, rightDuringHold = false;

    @Override
    public void init() {
        message = "Choose a mode, then press Play (or A after Play). Y = RUN ALL.";
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
        // Both sticks: stop everything (checked first, so it also works between RUN ALL modes)
        if (gamepad1.left_stick_button && gamepad1.right_stick_button && (running != null || runAll)) {
            if (running != null) stopRunning("Stopped " + runningName + ".");
            if (runAll) finishRunAll("RUN ALL stopped.");
            leftStickHeld = false;
            menu(true);
            return;
        }

        if (running == null) {
            if (runAll) {
                nextInRunAll("");   // the last mode couldn't start: move on
            } else {
                menu(true);
            }
            return;
        }
        // Left stick click (released without the right stick) = skip to the next mode
        if (gamepad1.left_stick_button) {
            leftStickHeld = true;
            rightDuringHold |= gamepad1.right_stick_button;
        } else if (leftStickHeld) {
            boolean skip = !rightDuringHold;
            leftStickHeld = false;
            rightDuringHold = false;
            if (skip && runAll) {
                nextInRunAll("skipped");
                return;
            }
        }
        if (runAll && modeTimer.milliseconds() > modes[runIndex].runAllMs) {
            nextInRunAll("ran " + (int) (modes[runIndex].runAllMs / 1000) + " s");
            return;
        }

        if (runAll) {
            telemetry.addData("RUN ALL", "%d/%d: %s  (%.0f s left; click L stick = skip, both = stop)",
                    position(), count(), runningName,
                    Math.max(0, modes[runIndex].runAllMs - modeTimer.milliseconds()) / 1000);
        } else {
            telemetry.addData("TEST MODE", "%s   (click both sticks = menu)", runningName);
        }
        try {
            running.loop();
        } catch (Exception e) {
            stopRunning("⚠️ " + runningName + " crashed: " + e);
            if (runAll) nextInRunAll("CRASHED: " + e.getClass().getSimpleName() + " " + e.getMessage());
        }
    }

    @Override
    public void stop() {
        stopRunning("");
    }

    // -------------------------------------------------
    // Menu
    // -------------------------------------------------

    private void menu(boolean canStart) {
        boolean up = gamepad1.dpad_up, down = gamepad1.dpad_down, a = gamepad1.a, x = gamepad1.x, y = gamepad1.y;
        if (up && !lastUp)     selected = (selected + modes.length - 1) % modes.length;
        if (down && !lastDown) selected = (selected + 1) % modes.length;
        boolean pressedA = a && !lastA, pressedX = x && !lastX, pressedY = y && !lastY;
        lastUp = up; lastDown = down; lastA = a; lastX = x; lastY = y;

        telemetry.addLine("=== TEST MODE: run everything ===");
        telemetry.addLine("D-pad UP/DOWN choose, " + (canStart ? "A start" : "Play start")
                + ".  Y = RUN ALL,  X = RUN ALL BIOBUZZ only.");
        telemetry.addLine("While running: click LEFT stick = skip, BOTH sticks = stop.");
        // Show a window of the list around the selection (the full list is long)
        int from = Math.max(0, Math.min(selected - 6, modes.length - 13));
        int to = Math.min(modes.length, from + 13);
        if (from > 0) telemetry.addLine("    ...");
        for (int i = from; i < to; i++) {
            telemetry.addLine((i == selected ? " >> " : "    ") + modes[i].name);
        }
        if (to < modes.length) telemetry.addLine("    ... (" + (modes.length - to) + " more)");
        if (!message.isEmpty()) telemetry.addLine(message);
        for (String r : results) telemetry.addLine(r);
        telemetry.addLine("Autonomous modes drive by themselves: keep clear of the robot.");
        telemetry.update();

        if (!canStart) return;
        if (pressedY) startRunAll(true);
        else if (pressedX) startRunAll(false);
        else if (pressedA) startSelected();
    }

    // -------------------------------------------------
    // RUN ALL
    // -------------------------------------------------

    private void startRunAll(boolean includeOld) {
        runAll = true;
        runAllIncludesOld = includeOld;
        runIndex = -1;
        results.clear();
        results.add(includeOld ? "RUN ALL results:" : "RUN ALL BIOBUZZ results:");
        nextInRunAll("");
    }

    /** Record how the current mode went, stop it, and start the next one in RUN ALL. */
    private void nextInRunAll(String howItWent) {
        if (runIndex >= 0 && !howItWent.isEmpty()) {
            results.add("  " + modes[runIndex].name + ": " + howItWent);
        }
        stopRunning("");
        do {
            runIndex++;
        } while (runIndex < modes.length && modes[runIndex].old && !runAllIncludesOld);
        if (runIndex >= modes.length) {
            finishRunAll("RUN ALL finished: " + (results.size() - 1) + " modes.");
            return;
        }
        selected = runIndex;
        startSelected();
        if (running == null) {
            results.add("  " + modes[runIndex].name + ": couldn't start (" + message + ")");
        }
        modeTimer.reset();
    }

    private void finishRunAll(String why) {
        runAll = false;
        stopRunning("");
        message = why;
    }

    private int position() {
        int n = 0;
        for (int i = 0; i <= runIndex; i++) if (!modes[i].old || runAllIncludesOld) n++;
        return n;
    }

    private int count() {
        int n = 0;
        for (Mode m : modes) if (!m.old || runAllIncludesOld) n++;
        return n;
    }

    // -------------------------------------------------

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
        // Don't let the button press that started it also count inside the mode
        lastA = lastX = lastY = true;
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
