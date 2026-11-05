package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

/**
 * Which Robot Controller the code is running on, and which camera to use.
 *
 * FTC rule R701 allows either:
 *   A. a REV Control Hub (the officially supported Robot Controller), or
 *   B. an Android phone (Android 7.0+) connected by USB to a REV Expansion Hub.
 * The same TeamCode APK runs on both. Motors, servos, sensors, the Pinpoint and the
 * Limelight are configured the same way; only the camera choice differs:
 *
 *   1. a Limelight 3A, if configured (see LimelightVision; chosen by the OpModes),
 *   2. "Webcam 1", if configured,
 *   3. on a phone only: the phone's own back camera.
 */
public final class RobotPlatform {

    public static final String WEBCAM_NAME = "Webcam 1";

    // Ball distance needs the camera's focal length in pixels at 640x480.
    // Phone back cameras are wider than the C270; ~470 px is typical (about 69 deg wide).
    // Calibrate yours with BallDetectionTest (ball at 24 in, see BallDetector) and change this.
    public static final double PHONE_CAMERA_FOCAL_LENGTH_PX = 470.0;

    private RobotPlatform() {}

    /** True on a REV Control Hub, false on an Android phone Robot Controller. */
    public static boolean isControlHub() {
        return LynxConstants.isRevControlHub();
    }

    /** True if "Webcam 1" is in the active robot configuration. */
    public static boolean hasWebcam(HardwareMap hwMap) {
        return hwMap.tryGet(WebcamName.class, WEBCAM_NAME) != null;
    }

    /** True if vision will use the phone's own camera (phone RC with no "Webcam 1"). */
    public static boolean usesPhoneCamera(HardwareMap hwMap) {
        return !isControlHub() && !hasWebcam(hwMap);
    }

    /**
     * Points a VisionPortal at "Webcam 1", or at the phone's back camera on a phone
     * Robot Controller without a webcam.
     *
     * @throws IllegalStateException on a Control Hub with no "Webcam 1" configured
     */
    public static VisionPortal.Builder setCamera(VisionPortal.Builder builder, HardwareMap hwMap) {
        WebcamName webcam = hwMap.tryGet(WebcamName.class, WEBCAM_NAME);
        if (webcam != null) {
            return builder.setCamera(webcam);
        }
        if (!isControlHub()) {
            return builder.setCamera(BuiltinCameraDirection.BACK);
        }
        throw new IllegalStateException("No \"" + WEBCAM_NAME + "\" in the robot configuration");
    }

    /** One line for telemetry, e.g. "Android phone + Expansion Hub, phone back camera". */
    public static String describe(HardwareMap hwMap) {
        String controller = isControlHub() ? "REV Control Hub" : "Android phone + Expansion Hub";
        String camera = hasWebcam(hwMap) ? WEBCAM_NAME
                : (isControlHub() ? "no webcam" : "phone back camera");
        return controller + ", " + camera;
    }
}
