package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.ArrayList;
import java.util.List;

/**
 * Limelight 3A support (the Limelight's USB name is "Limelight / Raspberry Pi Compute Module 4").
 *
 * The Limelight does its own image processing and runs ONE pipeline at a time. Set these up
 * in the Limelight web interface (http://limelight.local:5801 with the Limelight plugged into
 * a laptop, or through the Control Hub), then save each one:
 *
 *   Pipeline 0  HIVE_PIPELINE         AprilTag / Fiducial, family 36h11, tag size 82.55 mm (3.25 in)
 *   Pipeline 1  POLLEN_PIPELINE       Color, tuned to the yellow POLLEN ball
 *   Pipeline 2  RED_NECTAR_PIPELINE   Color, tuned to the red NECTAR ball
 *   Pipeline 3  BLUE_NECTAR_PIPELINE  Color, tuned to the blue NECTAR ball
 *
 * For the color pipelines, enable multiple targets ("Smart Speckle"/"Max targets" > 1) so every
 * ball in view is reported, and set a minimum area so specks are ignored.
 *
 * Hardware: plug the Limelight into one of the Control Hub's USB ports (it is powered over
 * USB), wait ~30 s for it to boot (its LED turns green), then on the Driver Station:
 * Configure Robot -> edit the configuration -> Scan. The Limelight appears in the list (as an
 * Ethernet device); name it "limelight" (any name works, but that's the default), Save, and
 * make that configuration active. Use LimelightTest to check it is seen and running.
 * Plugged into a laptop instead, it is a USB network gadget for its web interface only.
 *
 * OpModes call setMode() to pick what to look for, update() once per loop, then read
 * getCellTarget() or getBallSightings(). Switching pipelines takes a moment, so results
 * from the previous pipeline are ignored until the new one is running.
 */
public class LimelightVision {

    public static final String DEVICE_NAME = "limelight";

    public static final int HIVE_PIPELINE        = 0;
    public static final int POLLEN_PIPELINE      = 1;
    public static final int RED_NECTAR_PIPELINE  = 2;
    public static final int BLUE_NECTAR_PIPELINE = 3;

    // Limelight 3A camera: 640x480, about 54.5 x 42 degree field of view
    public static final double IMAGE_WIDTH_PX  = 640;
    public static final double IMAGE_HEIGHT_PX = 480;
    public static final double HORIZONTAL_FOV_DEG = 54.5;

    // BIOBUZZ HIVE CELL AprilTag IDs (clusters under each CELL)
    private static final int RED_FIRST_ID = 30, RED_LAST_ID = 37;
    private static final int BLUE_FIRST_ID = 38, BLUE_LAST_ID = 45;

    /** What the Limelight should look for. */
    public enum Mode { HIVE, BALLS }

    /** One ball the Limelight's color pipeline saw. */
    public static class BallSighting {
        public final BallDetector.BallColor color;
        public final double txDeg;        // + = RIGHT of center (Limelight convention)
        public final double tyDeg;        // + = above center
        public final double areaPercent;  // target area, % of the image

        BallSighting(BallDetector.BallColor color, double txDeg, double tyDeg, double areaPercent) {
            this.color = color;
            this.txDeg = txDeg;
            this.tyDeg = tyDeg;
            this.areaPercent = areaPercent;
        }
    }

    private final Limelight3A limelight;
    private final Telemetry telemetry;
    private int pipeline = -1;
    private LLResult result = null;

    private LimelightVision(Limelight3A limelight, Telemetry telemetry) {
        this.limelight = limelight;
        this.telemetry = telemetry;
    }

    /**
     * The Limelight in the active robot configuration: the one named "limelight" if there is
     * one, otherwise the first Limelight 3A under any name. Null if none is configured.
     */
    public static Limelight3A find(HardwareMap hwMap) {
        Limelight3A named = hwMap.tryGet(Limelight3A.class, DEVICE_NAME);
        if (named != null) return named;
        List<Limelight3A> all = hwMap.getAll(Limelight3A.class);
        return all.isEmpty() ? null : all.get(0);
    }

    /**
     * Returns a started LimelightVision if a Limelight 3A is in the robot configuration,
     * or null if there isn't one (so OpModes can fall back to the webcam).
     */
    public static LimelightVision tryCreate(HardwareMap hwMap, Telemetry telemetry) {
        Limelight3A limelight = find(hwMap);
        if (limelight == null) {
            if (telemetry != null) {
                telemetry.addLine("No Limelight 3A in the robot configuration (using Webcam 1)."
                        + " Run LimelightTest for setup help.");
            }
            return null;
        }
        try {
            LimelightVision vision = new LimelightVision(limelight, telemetry);
            limelight.setPollRateHz(100);   // ask for results 100 times a second
            limelight.start();
            vision.setPipeline(HIVE_PIPELINE);
            if (telemetry != null) {
                telemetry.addData("Limelight 3A", "found: %s", hwMap.getNamesOf(limelight));
            }
            return vision;
        } catch (Exception e) {
            if (telemetry != null) telemetry.addData("⚠️ Limelight 3A failed to start", e.getMessage());
            return null;
        }
    }

    /** Look for the HIVE (AprilTags) or for balls of the given color. */
    public void setMode(Mode mode, BallDetector.BallColor ballColor) {
        setPipeline(mode == Mode.HIVE ? HIVE_PIPELINE : pipelineFor(ballColor));
    }

    public static int pipelineFor(BallDetector.BallColor color) {
        switch (color) {
            case RED:  return RED_NECTAR_PIPELINE;
            case BLUE: return BLUE_NECTAR_PIPELINE;
            case YELLOW:
            default:   return POLLEN_PIPELINE;
        }
    }

    private void setPipeline(int index) {
        if (index != pipeline) {
            limelight.pipelineSwitch(index);
            pipeline = index;
            result = null;
        }
    }

    /** Reads the newest result. Call once per loop. */
    public void update() {
        LLResult latest = limelight.getLatestResult();
        // Only use results from the pipeline we asked for (ignore ones from before a switch)
        result = (latest != null && latest.isValid() && latest.getPipelineIndex() == pipeline)
                ? latest : null;
    }

    /** True while the Limelight is connected and sending results. */
    public boolean isConnected() {
        return limelight.isConnected();
    }

    /**
     * Our alliance's HIVE CELL from the visible CELL tags (red 30-37, blue 38-45),
     * averaged over all of them, or null if none are in view (or not in HIVE mode).
     */
    public CellTarget getCellTarget(boolean red) {
        if (result == null || pipeline != HIVE_PIPELINE) return null;
        int first = red ? RED_FIRST_ID : BLUE_FIRST_ID;
        int last  = red ? RED_LAST_ID  : BLUE_LAST_ID;

        double sumX = 0, sumY = 0, sumZ = 0;
        int count = 0;
        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            int id = tag.getFiducialId();
            if (id < first || id > last) continue;
            // Camera space: x = right, y = down, z = forward
            Position p = tag.getTargetPoseCameraSpace().getPosition().toUnit(DistanceUnit.INCH);
            sumX += p.x;
            sumY += p.y;
            sumZ += p.z;
            count++;
        }
        if (count == 0) return null;
        double x = sumX / count, y = sumY / count, z = sumZ / count;
        double range = Math.sqrt(x * x + y * y + z * z);
        double bearing = Math.toDegrees(Math.atan2(-x, z));   // + = LEFT, like AprilTag bearing
        return new CellTarget(range, bearing);
    }

    /** Balls from the current color pipeline (empty in HIVE mode). */
    public List<BallSighting> getBallSightings() {
        List<BallSighting> out = new ArrayList<>();
        if (result == null || pipeline == HIVE_PIPELINE) return out;
        BallDetector.BallColor color =
                pipeline == RED_NECTAR_PIPELINE ? BallDetector.BallColor.RED
                : pipeline == BLUE_NECTAR_PIPELINE ? BallDetector.BallColor.BLUE
                : BallDetector.BallColor.YELLOW;
        for (LLResultTypes.ColorResult blob : result.getColorResults()) {
            out.add(new BallSighting(color, blob.getTargetXDegrees(),
                    blob.getTargetYDegrees(), blob.getTargetArea()));
        }
        return out;
    }

    public void displayTelemetry() {
        if (telemetry == null) return;
        telemetry.addData("Limelight", "%s, pipeline %d%s",
                limelight.isConnected() ? "connected" : "NOT CONNECTED", pipeline,
                result == null ? " (no target)" : "");
    }

    public void stop() {
        limelight.stop();
    }
}
