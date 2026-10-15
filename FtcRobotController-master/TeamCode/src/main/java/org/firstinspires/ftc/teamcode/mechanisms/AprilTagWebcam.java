package org.firstinspires.ftc.teamcode.mechanisms;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.ArrayList;
import java.util.List;

public class AprilTagWebcam {
    // AprilTag processor that performs detection + pose estimation
    private AprilTagProcessor aprilTagProcessor;

    // VisionPortal connects the camera stream with the processors
    private VisionPortal visionPortal;

    // List of AprilTag detections found on the current frame
    private List<AprilTagDetection> detectedTags = new ArrayList<>();

    // Telemetry reference for reporting data to the Driver Station
    private Telemetry telemetry;
    
    private double rangeTolerance = 5;
    
    private double bearingTolerance = 5;

    private double targetRange = 0;

    private double targetBearing = 0;

    /**
     * Initializes the webcam, tag library, and AprilTag detection pipeline.
     */
    public void init(HardwareMap hwMap, Telemetry telemetry) {
        init(hwMap, telemetry, new VisionProcessor[0]);
    }

    /**
     * Same as init(), but also attaches extra processors (e.g. BallDetector.getProcessors())
     * to the same camera, since only one VisionPortal can use the webcam at a time.
     */
    public void init(HardwareMap hwMap, Telemetry telemetry, VisionProcessor... extraProcessors) {
        this.telemetry = telemetry;

        // Build a custom AprilTag library (defines IDs, names, and tag sizes)
        AprilTagLibrary.Builder tagbuilder = new AprilTagLibrary.Builder();
        tagbuilder.addTag(20, "Tag20", 0.166, DistanceUnit.METER);
        tagbuilder.addTag(21, "Tag21", 0.166, DistanceUnit.METER);
        tagbuilder.addTag(22, "Tag22", 0.166, DistanceUnit.METER);
        tagbuilder.addTag(23, "Tag23", 0.166, DistanceUnit.METER);
        tagbuilder.addTag(24, "Tag24", 0.166, DistanceUnit.METER);

        // ...add other tags if desired

        // Final library object containing all registered tags
        AprilTagLibrary library = tagbuilder.build();

        // Create the AprilTag detector and configure drawing + output units
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)           // Show tag ID on screen
                .setDrawTagOutline(true)      // Show tag borders
                .setDrawAxes(true)            // Show tag orientation axes
                .setDrawCubeProjection(true)  // 3D cube overlay for visualization
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setTagLibrary(library)       // Use custom FTCLib-style tag library
                .build();

        // Open camera and attach AprilTag processor
        visionPortal = new VisionPortal.Builder()
                .setCamera(hwMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))   // Camera resolution
                .addProcessor(aprilTagProcessor)
                .addProcessors(extraProcessors)
                .build();
    }
    
    public void setRangeTolerance(double rangeTolerance) {
        this.rangeTolerance = rangeTolerance;
    }


    public void setBearingTolerance(double bearingTolerance) {
        this.bearingTolerance = bearingTolerance;
    }

    public void setTargetRange(double targetRange) {
        this.targetRange = targetRange;
    }

    public void setTargetBearing(double targetBearing) {
        this.targetBearing = targetBearing;
    }

    /**
     * Updates the detection list for the current frame.
     * Should be called repeatedly in loop().
     */
    public void update() {
        detectedTags = aprilTagProcessor.getDetections();
    }

    /**
     * Sends formatted AprilTag pose data to telemetry.
     * Useful for debugging + visualization in Driver Station.
     */
    public void displayDetectionTelemetry(AprilTagDetection detectionId) {
        if (detectionId != null) {
            // SDK 12: a detection is either a single tag or a cluster of tags
            AprilTagSingleDetection single = (detectionId instanceof AprilTagSingleDetection)
                    ? (AprilTagSingleDetection) detectionId : null;
            boolean known = (single == null) || single.metadata != null;
            if (known) {
                if (single != null) {
                    // Known tag (exists in library)
                    telemetry.addLine(String.format("\n==== (ID %d) %s", single.id, single.metadata.name));
                } else {
                    AprilTagClusterDetection cluster = (AprilTagClusterDetection) detectionId;
                    telemetry.addLine(String.format("\n==== Tag Cluster (%s) %d%% found",
                            cluster.metadata.name, cluster.percentClusterFound));
                }

                // Position of the tag relative to the camera
                telemetry.addLine(String.format(
                        "XYZ %6.1f %6.1f %6.1f  (inch)",
                        detectionId.ftcPose.x,
                        detectionId.ftcPose.y,
                        detectionId.ftcPose.z
                ));

                // Orientation angles of the tag
                telemetry.addLine(String.format(
                        "PRY %6.1f %6.1f %6.1f  (deg)",
                        detectionId.ftcPose.pitch,
                        detectionId.ftcPose.roll,
                        detectionId.ftcPose.yaw
                ));

                // Range, bearing, elevation (useful for navigation)
                telemetry.addLine(String.format(
                        "RBE %6.1f %6.1f %6.1f  (inch, deg, deg)",
                        detectionId.ftcPose.range,
                        detectionId.ftcPose.bearing,
                        detectionId.ftcPose.elevation
                ));
            } else {
                // Tag detected but not included in configured library
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", single.id));
                telemetry.addLine(String.format(
                        "Center %6.0f %6.0f   (pixels)",
                        single.center.x,
                        single.center.y
                ));
            }
        }
    }

    /**
     * Returns a list of all tags detected in the most recent frame.
     */
    public List<AprilTagDetection> getDetectedTags() {
        return detectedTags;
    }

    /**
     * Returns a single tag with the requested ID, or null if not seen.
     */
    public AprilTagDetection getTagBySpecificId(int id) {
        for (AprilTagDetection detection : detectedTags) {
            if (idOf(detection) == id) {
                return detection;
            }
        }
        return null;
    }

    /** Tag ID of a single-tag detection, or -1 for a tag cluster (SDK 12+). */
    private static int idOf(AprilTagDetection detection) {
        return (detection instanceof AprilTagSingleDetection)
                ? ((AprilTagSingleDetection) detection).id : -1;
    }

    public boolean IsRobotinZone(int id) {
        boolean inZone = false;
        for (AprilTagDetection detection : detectedTags) {
            if (idOf(detection) == id) {
                inZone = isRangeBearingInZone(detection);
                break;
            }
        }
        return inZone;
    }


    private boolean isRangeBearingInZone(AprilTagDetection detection) {
        if ((Math.abs(detection.ftcPose.range - targetRange) <= rangeTolerance) &&
                (Math.abs(detection.ftcPose.bearing - targetBearing) <= bearingTolerance)) {
            return true;
        }
        else {
            return false;
        }
    }

    /**
     * Closes the VisionPortal camera stream.
     * Always call this when the OpMode ends.
     */
    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    /**
     * Simple hardware check for diagnostics.
     */
    public void reportHardwareStatus() {
        telemetry.addLine("Webcam 1");
        telemetry.addData("Target Range",targetRange);
        telemetry.addData("Target Bearing",targetBearing);
        telemetry.addData("Tolerance Range",rangeTolerance);
        telemetry.addData("Tolerance Bearing",bearingTolerance);

    }
}
