package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.LimelightVision;

/**
 * Limelight 3A troubleshooting. Run it from INIT; it shows each step of the chain:
 *   1. Is a Limelight 3A in the active robot configuration (and under what name)?
 *   2. Is it connected to the Control Hub and sending data?
 *   3. Which pipeline is running, and what does it see?
 *
 * D-pad: up = pipeline 0 (HIVE AprilTags), right = 1 (POLLEN), down = 2 (red), left = 3 (blue).
 */
@TeleOp(name = "LimelightTest", group = "Vision")
public class LimelightTest extends OpMode {

    private Limelight3A limelight = null;
    private int pipeline = LimelightVision.HIVE_PIPELINE;

    @Override
    public void init() {
        limelight = LimelightVision.find(hardwareMap);
        if (limelight != null) {
            limelight.setPollRateHz(100);
            limelight.start();
            limelight.pipelineSwitch(pipeline);
        }
    }

    @Override
    public void init_loop() {
        show();
    }

    @Override
    public void loop() {
        show();
    }

    private void show() {
        telemetry.addLine("== 1. Robot configuration ==");
        if (limelight == null) {
            telemetry.addLine("NO Limelight 3A in the active configuration.");
            telemetry.addLine("Fix:");
            telemetry.addLine(" a) Plug the Limelight into a Control Hub USB port");
            telemetry.addLine("    (not a laptop). Wait ~30 s for it to boot.");
            telemetry.addLine(" b) Driver Station: Configure Robot -> edit config");
            telemetry.addLine("    -> Scan. Name the Limelight \"limelight\", Save,");
            telemetry.addLine("    and ACTIVATE that configuration.");
            telemetry.addLine(" c) Restart this OpMode.");
            telemetry.addLine("If Scan doesn't list it: update the Control Hub OS");
            telemetry.addLine("and Limelight firmware (REV Hardware Client /");
            telemetry.addLine("Limelight Hardware Manager), try another USB port/cable.");
            telemetry.update();
            return;
        }
        telemetry.addData("Found", "%s  (%s)",
                hardwareMap.getNamesOf(limelight), limelight.getConnectionInfo());

        telemetry.addLine("== 2. Connection ==");
        telemetry.addData("Connected", limelight.isConnected());
        telemetry.addData("Running", limelight.isRunning());
        telemetry.addData("ms since last data", limelight.getTimeSinceLastUpdate());
        if (!limelight.isConnected()) {
            telemetry.addLine("Configured but not talking. Check the USB cable,");
            telemetry.addLine("that the Limelight LED is on, and power-cycle the robot.");
        }
        LLStatus status = limelight.getStatus();
        if (status != null) {
            telemetry.addData("Name / FPS", "%s / %.0f", status.getName(), status.getFps());
            telemetry.addData("Temp / CPU", "%.0f C / %.0f%%", status.getTemp(), status.getCpu());
            telemetry.addData("Pipeline on device", "%d (%s)",
                    status.getPipelineIndex(), status.getPipelineType());
            // Is this pipeline the TYPE the robot code expects? (most common setup problem)
            String type = status.getPipelineType() == null ? "" : status.getPipelineType();
            boolean wantTags = status.getPipelineIndex() == LimelightVision.HIVE_PIPELINE;
            boolean ok = wantTags ? type.contains("fiducial") : type.contains("color");
            if (!ok) {
                telemetry.addLine("⚠️ WRONG PIPELINE TYPE: pipeline " + status.getPipelineIndex() + " is \""
                        + type + "\" but the robot code needs "
                        + (wantTags ? "AprilTag (Fiducial Markers, 36h11)." : "Color (tuned to the ball color)."));
                telemetry.addLine("   Fix it in the Limelight web page (see LimelightVision.java).");
            }
        }

        // D-pad switches pipelines
        int wanted = pipeline;
        if (gamepad1.dpad_up)    wanted = LimelightVision.HIVE_PIPELINE;
        if (gamepad1.dpad_right) wanted = LimelightVision.POLLEN_PIPELINE;
        if (gamepad1.dpad_down)  wanted = LimelightVision.RED_NECTAR_PIPELINE;
        if (gamepad1.dpad_left)  wanted = LimelightVision.BLUE_NECTAR_PIPELINE;
        if (wanted != pipeline) {
            pipeline = wanted;
            limelight.pipelineSwitch(pipeline);
        }

        telemetry.addLine("== 3. What it sees ==");
        telemetry.addData("Requested pipeline", "%d  (dpad: up 0 HIVE, right 1 POLLEN, down 2 red, left 3 blue)",
                pipeline);
        LLResult result = limelight.getLatestResult();
        if (result == null) {
            telemetry.addLine("No results from the Limelight yet.");
        } else if (!result.isValid()) {
            telemetry.addData("Result", "pipeline %d (%s): NO TARGET", result.getPipelineIndex(),
                    result.getPipelineType());
            telemetry.addLine(pipeline == LimelightVision.HIVE_PIPELINE
                    ? "Point it at a HIVE CELL tag (IDs 30-45, 36h11)."
                    : "Point it at a ball. If a ball in view isn't found, retune this pipeline's color"
                      + " (Thresholding: use the color picker on the ball).");
        } else {
            telemetry.addData("Result", "pipeline %d (%s)", result.getPipelineIndex(), result.getPipelineType());
            telemetry.addData("tx / ty / ta", "%.1f / %.1f / %.2f%%", result.getTx(), result.getTy(), result.getTa());
            for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                int id = tag.getFiducialId();
                boolean hive = id >= 30 && id <= 45;
                telemetry.addData("Tag", "id %d %s  tx %.1f  area %.2f%%", id,
                        hive ? (id <= 37 ? "(RED HIVE CELL)" : "(BLUE HIVE CELL)") : "(not a HIVE tag; ignored)",
                        tag.getTargetXDegrees(), tag.getTargetArea());
            }
            int colorCount = result.getColorResults().size();
            if (colorCount > 0) {
                telemetry.addData("Color targets", colorCount);
                for (LLResultTypes.ColorResult blob : result.getColorResults()) {
                    telemetry.addData("  blob", "tx %.1f  ty %.1f  area %.2f%%", blob.getTargetXDegrees(),
                            blob.getTargetYDegrees(), blob.getTargetArea());
                }
            }
        }
        telemetry.update();
    }

    @Override
    public void stop() {
        if (limelight != null) limelight.stop();
    }
}
