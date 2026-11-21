package org.firstinspires.ftc.teamcode.mechanisms;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.opencv.core.Scalar;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.EnumMap;
import java.util.Map;

/**
 * Ball color ranges, calibrated from the robot's own camera.
 *
 * The SDK's built-in YELLOW / RED / BLUE ranges are tuned for other game pieces and need very
 * saturated colors, so under real field lighting a webcam often matches nothing. In
 * BallDetectionTest, hold a ball in the center of the camera view and press the D-pad button for
 * its color: the measured color is saved to /sdcard/FIRST/biobuzz_ball_colors.txt on the Control
 * Hub and used by every OpMode from then on (restart the OpMode to apply). Colors that were never
 * calibrated use the SDK defaults.
 *
 * Ranges are in YCrCb: Y = brightness, Cr = red-ness, Cb = blue-ness (0-255 each).
 */
public final class BallColorCalibration {

    public static final File FILE = new File(AppUtil.FIRST_FOLDER, "biobuzz_ball_colors.txt");

    // How wide a range to make around a measured color
    private static final double MIN_HALF_WIDTH = 15;   // Cr / Cb: at least +/- this much
    private static final double STD_MULTIPLIER = 3;    // ... or 3 standard deviations, if wider
    private static final double Y_BELOW = 90;          // allow this much darker (shading, shadow)
    private static final double Y_MIN = 20;

    /** One color range in YCrCb: {yMin, crMin, cbMin} .. {yMax, crMax, cbMax}. */
    public static class Range {
        public final double[] min, max;

        public Range(double[] min, double[] max) {
            this.min = min;
            this.max = max;
        }

        public ColorRange toColorRange() {
            return new ColorRange(ColorSpace.YCrCb,
                    new Scalar(min[0], min[1], min[2]), new Scalar(max[0], max[1], max[2]));
        }

        public boolean contains(double y, double cr, double cb) {
            return y >= min[0] && y <= max[0] && cr >= min[1] && cr <= max[1]
                    && cb >= min[2] && cb <= max[2];
        }

        @Override
        public String toString() {
            return String.format("Y %.0f-%.0f Cr %.0f-%.0f Cb %.0f-%.0f",
                    min[0], max[0], min[1], max[1], min[2], max[2]);
        }
    }

    // FTC SDK 12 ColorRange defaults (YCrCb), used until a color is calibrated
    private static final Map<BallDetector.BallColor, Range> DEFAULTS = new EnumMap<>(BallDetector.BallColor.class);
    static {
        DEFAULTS.put(BallDetector.BallColor.YELLOW, new Range(new double[]{32, 128, 0},  new double[]{255, 170, 120}));
        DEFAULTS.put(BallDetector.BallColor.RED,    new Range(new double[]{32, 176, 0},  new double[]{255, 255, 132}));
        DEFAULTS.put(BallDetector.BallColor.BLUE,   new Range(new double[]{16, 0, 155},  new double[]{255, 127, 255}));
    }

    private BallColorCalibration() {}

    /** The range to use for each color: calibrated if saved, otherwise the SDK default. */
    public static Map<BallDetector.BallColor, Range> load() {
        Map<BallDetector.BallColor, Range> ranges = new EnumMap<>(DEFAULTS);
        if (!FILE.exists()) return ranges;
        try (BufferedReader in = new BufferedReader(new FileReader(FILE))) {
            String line;
            while ((line = in.readLine()) != null) {
                String[] p = line.trim().split("\\s+");
                if (p.length != 7) continue;   // COLOR yMin crMin cbMin yMax crMax cbMax
                try {
                    BallDetector.BallColor color = BallDetector.BallColor.valueOf(p[0]);
                    double[] min = {Double.parseDouble(p[1]), Double.parseDouble(p[2]), Double.parseDouble(p[3])};
                    double[] max = {Double.parseDouble(p[4]), Double.parseDouble(p[5]), Double.parseDouble(p[6])};
                    ranges.put(color, new Range(min, max));
                } catch (IllegalArgumentException ignored) {
                    // bad line: keep the default for that color
                }
            }
        } catch (IOException ignored) {
            // unreadable file: defaults
        }
        return ranges;
    }

    /** True if this color has a saved calibration. */
    public static boolean isCalibrated(BallDetector.BallColor color) {
        Range saved = load().get(color);
        Range def = DEFAULTS.get(color);
        return saved != null && def != null && !(java.util.Arrays.equals(saved.min, def.min)
                && java.util.Arrays.equals(saved.max, def.max));
    }

    /**
     * A range around a measured color (mean and standard deviation of Y, Cr, Cb over the
     * center of the image): Cr and Cb +/- max(15, 3 std), Y from 90 darker up to 255.
     */
    public static Range rangeFrom(double[] mean, double[] std) {
        double[] min = new double[3], max = new double[3];
        min[0] = Math.max(Y_MIN, mean[0] - Y_BELOW);
        max[0] = 255;
        for (int i = 1; i < 3; i++) {
            double half = Math.max(MIN_HALF_WIDTH, STD_MULTIPLIER * std[i]);
            min[i] = clamp(mean[i] - half);
            max[i] = clamp(mean[i] + half);
        }
        return new Range(min, max);
    }

    /** Saves one color's range, keeping the others. */
    public static void save(BallDetector.BallColor color, Range range) throws IOException {
        Map<BallDetector.BallColor, Range> ranges = load();
        ranges.put(color, range);
        write(ranges);
    }

    /** Deletes the calibration file: back to the SDK defaults. */
    public static boolean reset() {
        return !FILE.exists() || FILE.delete();
    }

    private static void write(Map<BallDetector.BallColor, Range> ranges) throws IOException {
        File parent = FILE.getParentFile();
        if (parent != null && !parent.exists() && !parent.mkdirs()) {
            throw new IOException("can't create " + parent);
        }
        try (FileWriter out = new FileWriter(FILE)) {
            out.write("# BIOBUZZ ball colors (YCrCb): COLOR yMin crMin cbMin yMax crMax cbMax\n");
            for (Map.Entry<BallDetector.BallColor, Range> e : ranges.entrySet()) {
                Range r = e.getValue();
                out.write(String.format(java.util.Locale.US, "%s %.0f %.0f %.0f %.0f %.0f %.0f%n", e.getKey(),
                        r.min[0], r.min[1], r.min[2], r.max[0], r.max[1], r.max[2]));
            }
        }
    }

    private static double clamp(double v) {
        return Math.max(0, Math.min(255, v));
    }
}
