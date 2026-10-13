package org.firstinspires.ftc.teamcode.mechanisms;

import android.graphics.Color;
import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.opencv.Circle;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

import java.util.ArrayList;
import java.util.Collections;
import java.util.EnumMap;
import java.util.List;

/**
 * Camera-based ball detection using the FTC SDK color blob locator.
 *
 * Finds yellow, blue, and red plastic (wiffle-style) balls in the camera image,
 * fits a circle to each one (the holes are filled in by the blob cleanup step),
 * and estimates the horizontal angle (bearing) and distance to the ball.
 *
 * Two ways to use it:
 *  1. Standalone: call init(hardwareMap, telemetry) and this class opens its own camera.
 *  2. Shared camera with AprilTags: call buildProcessors(telemetry) and pass the result to
 *     AprilTagWebcam.init(hardwareMap, telemetry, ballDetector.getProcessors()).
 *     Only one VisionPortal can use "Webcam 1" at a time, so use this when you need both.
 *
 * Call update() once per loop, then read getBalls() / getClosestBall().
 *
 * Sound: when a ball comes into view, the Driver Station says its color out loud
 * (e.g. "red ball"). On by default; turn off with setSoundEnabled(false).
 * The Control Hub has no speaker, so speech plays on the Driver Station.
 */
public class BallDetector {

    public enum BallColor {
        YELLOW(ColorRange.YELLOW, Color.rgb(255, 255, 0)),
        BLUE  (ColorRange.BLUE,   Color.rgb(0, 128, 255)),
        RED   (ColorRange.RED,    Color.rgb(255, 0, 0));

        final ColorRange range;
        final int drawColor;   // circle color on the camera stream preview

        BallColor(ColorRange range, int drawColor) {
            this.range = range;
            this.drawColor = drawColor;
        }
    }

    /**
     * A single detected ball for the current frame.
     */
    public static class Ball {
        public final BallColor color;
        public final double centerX;      // pixels
        public final double centerY;      // pixels
        public final double radius;       // pixels
        public final double circularity;  // 0..1, 1 = perfect circle
        public final double bearing;      // degrees, + = ball is LEFT of center (same sign as AprilTag bearing)
        public final double distance;     // inches, approximate (see focalLengthPx)

        Ball(BallColor color, double centerX, double centerY, double radius,
             double circularity, double bearing, double distance) {
            this.color = color;
            this.centerX = centerX;
            this.centerY = centerY;
            this.radius = radius;
            this.circularity = circularity;
            this.bearing = bearing;
            this.distance = distance;
        }
    }

    // -----------------------------
    // Camera / geometry constants
    // -----------------------------
    // Must match the VisionPortal resolution (AprilTagWebcam also uses 640x480).
    public static final int IMAGE_WIDTH  = 640;
    public static final int IMAGE_HEIGHT = 480;

    // Ball diameter in inches. These plastic practice balls are about 2.9 in;
    // measure yours and call setBallDiameter() if it differs.
    private double ballDiameterIn = 2.9;

    // Camera focal length in pixels at IMAGE_WIDTH x IMAGE_HEIGHT.
    // ~822 is typical for a Logitech C270 at 640x480. TUNE THIS:
    // place a ball at a known distance D (inches), read its radius r (pixels) from telemetry,
    // then focalLengthPx = D * 2 * r / ballDiameterIn.
    private double focalLengthPx = 822.0;

    // -----------------------------
    // Blob filters (tune on the field)
    // -----------------------------
    private double minContourArea = 150;     // pixels^2, ignore specks
    private double maxContourArea = 60000;   // pixels^2, ignore huge regions (walls, robots)
    private double minCircularity = 0.55;    // ignore non-round blobs

    private final EnumMap<BallColor, ColorBlobLocatorProcessor> processors = new EnumMap<>(BallColor.class);
    private VisionPortal visionPortal;   // only set when this class owns the camera
    private Telemetry telemetry;

    private List<Ball> balls = new ArrayList<>();

    // -----------------------------
    // Sound (spoken on the Driver Station)
    // -----------------------------
    private boolean soundEnabled = true;
    private boolean announcedCurrentBall = false;          // already spoke for the ball in view
    private final ElapsedTime noBallTimer = new ElapsedTime();
    private final ElapsedTime announceTimer = new ElapsedTime();
    private static final double NO_BALL_RESET_MS    = 1000; // ball gone this long -> can announce again
    private static final double ANNOUNCE_COOLDOWN_MS = 2000; // never speak more often than this

    /**
     * Standalone init: opens "Webcam 1" with only the ball processors attached.
     */
    public void init(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        buildProcessors();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hwMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(IMAGE_WIDTH, IMAGE_HEIGHT))
                .addProcessors(getProcessors())
                .build();
    }

    /**
     * Shared-camera init: builds the processors only. Pass getProcessors() to the
     * VisionPortal that owns the camera (e.g. AprilTagWebcam).
     */
    public void buildProcessors(Telemetry telemetry) {
        this.telemetry = telemetry;
        buildProcessors();
    }

    private void buildProcessors() {
        processors.clear();
        for (BallColor color : BallColor.values()) {
            processors.put(color, buildProcessor(color));
        }
    }

    private ColorBlobLocatorProcessor buildProcessor(BallColor color) {
        return new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(color.range)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())
                .setDrawContours(true)
                .setBoxFitColor(0)               // don't draw rectangles
                .setCircleFitColor(color.drawColor)    // draw fitted circle on the stream preview
                .setBlurSize(5)
                .setDilateSize(15)               // fill divots (balls have holes)
                .setErodeSize(15)
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
                .build();
    }

    /**
     * Processors to attach to a shared VisionPortal. Call buildProcessors(telemetry) first.
     */
    public VisionProcessor[] getProcessors() {
        return processors.values().toArray(new VisionProcessor[0]);
    }

    public void setBallDiameter(double inches) {
        this.ballDiameterIn = inches;
    }

    public void setFocalLengthPx(double focalLengthPx) {
        this.focalLengthPx = focalLengthPx;
    }

    public void setContourAreaRange(double minArea, double maxArea) {
        this.minContourArea = minArea;
        this.maxContourArea = maxArea;
    }

    public void setMinCircularity(double minCircularity) {
        this.minCircularity = minCircularity;
    }

    /**
     * Reads the latest blobs from both processors. Call once per loop().
     */
    public void update() {
        List<Ball> found = new ArrayList<>();
        for (BallColor color : BallColor.values()) {
            addBalls(processors.get(color), color, found);
        }

        // Closest (largest) ball first
        Collections.sort(found, (a, b) -> Double.compare(a.distance, b.distance));
        balls = found;

        updateSound();
    }

    /** Says "<color> ball" once when a ball comes into view (not every frame). */
    private void updateSound() {
        if (balls.isEmpty()) {
            if (noBallTimer.milliseconds() > NO_BALL_RESET_MS) {
                announcedCurrentBall = false;
            }
            return;
        }
        noBallTimer.reset();

        if (!announcedCurrentBall && announceTimer.milliseconds() > ANNOUNCE_COOLDOWN_MS) {
            announce(balls.get(0).color.name().toLowerCase() + " ball");
            announcedCurrentBall = true;
        }
    }

    /** Speaks text on the Driver Station if sound is enabled. */
    public void announce(String text) {
        if (soundEnabled && telemetry != null) {
            telemetry.speak(text);
            announceTimer.reset();
        }
    }

    public void setSoundEnabled(boolean enabled) {
        this.soundEnabled = enabled;
    }

    public boolean isSoundEnabled() {
        return soundEnabled;
    }

    private void addBalls(ColorBlobLocatorProcessor processor, BallColor color, List<Ball> out) {
        if (processor == null) return;

        List<ColorBlobLocatorProcessor.Blob> blobs = processor.getBlobs();

        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                minContourArea, maxContourArea, blobs);
        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY,
                minCircularity, 1.0, blobs);

        for (ColorBlobLocatorProcessor.Blob blob : blobs) {
            Circle circle = blob.getCircle();
            double x = circle.getX();
            double y = circle.getY();
            double r = circle.getRadius();
            if (r <= 0) continue;

            out.add(new Ball(color, x, y, r, blob.getCircularity(),
                    pixelToBearing(x), radiusToDistance(r)));
        }
    }

    /**
     * Horizontal angle to a pixel column. Positive = left of image center,
     * matching AprilTag ftcPose.bearing so the same turn logic works for both.
     */
    private double pixelToBearing(double pixelX) {
        double offset = (IMAGE_WIDTH / 2.0) - pixelX;
        return Math.toDegrees(Math.atan2(offset, focalLengthPx));
    }

    /**
     * Pinhole camera estimate: distance = realDiameter * focal / pixelDiameter.
     */
    private double radiusToDistance(double radiusPx) {
        return (ballDiameterIn * focalLengthPx) / (2.0 * radiusPx);
    }

    /** All balls in the latest frame, closest first. */
    public List<Ball> getBalls() {
        return balls;
    }

    /** Closest ball of any color, or null if none seen. */
    public Ball getClosestBall() {
        return balls.isEmpty() ? null : balls.get(0);
    }

    /** Closest ball of the given color, or null if none seen. */
    public Ball getClosestBall(BallColor color) {
        for (Ball ball : balls) {
            if (ball.color == color) return ball;
        }
        return null;
    }

    public boolean hasBall() {
        return !balls.isEmpty();
    }

    public void displayTelemetry() {
        if (telemetry == null) return;
        telemetry.addData("Balls seen", balls.size());
        for (Ball ball : balls) {
            telemetry.addLine(String.format("%-6s dist %5.1f in  brg %5.1f deg  r %3d px  circ %.2f",
                    ball.color, ball.distance, ball.bearing, (int) ball.radius, ball.circularity));
        }
    }

    /**
     * Closes the camera if this class opened it. When sharing a portal,
     * the owner (e.g. AprilTagWebcam.stop()) closes it instead.
     */
    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}
