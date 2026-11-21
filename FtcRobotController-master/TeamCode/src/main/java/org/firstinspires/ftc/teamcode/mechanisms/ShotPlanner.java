package org.firstinspires.ftc.teamcode.mechanisms;

/**
 * Shooting and timing estimates for the autonomous "brain".
 *
 * - rpmForRange(): flywheel speed from the distance to the HIVE CELL, interpolated in RPM_TABLE.
 *   The table is a starting point (36 in / 2400 RPM matches the teleop shoot zone): measure what
 *   actually scores at each distance on your robot and put it here.
 * - shootTimeMs(), collectTimeMs(): how long things take, so the auto can tell whether there's
 *   still time to do them before AUTO ends.
 */
public final class ShotPlanner {

    /** {range to the CELL in inches, flywheel RPM}, sorted by range. RE-TUNE on the robot. */
    public static final double[][] RPM_TABLE = {
            {24, 2200},
            {36, 2400},
            {48, 2650},
            {60, 2900},
    };

    /** Closest / farthest distance it will shoot from (the ends of RPM_TABLE). */
    public static final double MIN_RANGE_IN = RPM_TABLE[0][0];
    public static final double MAX_RANGE_IN = RPM_TABLE[RPM_TABLE.length - 1][0];

    // Timing model (milliseconds / speeds), rough and on the safe side
    private static final double SPINUP_AND_FEED_MS = 1500;   // per shot: flywheel at speed + 1 s feeder push
    private static final double TURN_DEG_PER_S     = 120;    // turning toward a ball
    private static final double DRIVE_IN_PER_S     = 16;     // driving to a ball at the auto's slow speed
    private static final double CAPTURE_MS         = 1300;   // drive onto it + let the intake pull it in

    private ShotPlanner() {}

    /** Flywheel RPM for a distance, interpolated in RPM_TABLE (clamped at the ends). */
    public static double rpmForRange(double rangeIn) {
        if (Double.isNaN(rangeIn) || rangeIn <= RPM_TABLE[0][0]) return RPM_TABLE[0][1];
        for (int i = 1; i < RPM_TABLE.length; i++) {
            if (rangeIn <= RPM_TABLE[i][0]) {
                double[] a = RPM_TABLE[i - 1], b = RPM_TABLE[i];
                return a[1] + (b[1] - a[1]) * (rangeIn - a[0]) / (b[0] - a[0]);
            }
        }
        return RPM_TABLE[RPM_TABLE.length - 1][1];
    }

    /** True if the robot can shoot from this distance (inside RPM_TABLE). */
    public static boolean inShootingRange(double rangeIn) {
        return rangeIn >= MIN_RANGE_IN && rangeIn <= MAX_RANGE_IN;
    }

    /**
     * Time to shoot this many balls: left+right pairs, each pair = intake feed + 2 shots +
     * the delay between them.
     */
    public static double shootTimeMs(int balls, double feedMs, double delayMs) {
        int pairs = (Math.max(balls, 1) + 1) / 2;
        return pairs * (feedMs + 2 * SPINUP_AND_FEED_MS + delayMs);
    }

    /** Largest number of balls (whole left+right pairs) that can be shot in this much time. */
    public static int ballsThatFit(int balls, double msAvailable, double feedMs, double delayMs) {
        int pairsWanted = (Math.max(balls, 0) + 1) / 2;
        double perPair = feedMs + 2 * SPINUP_AND_FEED_MS + delayMs;
        int pairsFit = (int) Math.floor(msAvailable / perPair);
        return Math.min(pairsWanted, Math.max(pairsFit, 0)) * 2;
    }

    /** Time to turn toward, drive to and pick up a ball this far away at this angle. */
    public static double collectTimeMs(double distanceIn, double bearingDeg) {
        return Math.abs(bearingDeg) / TURN_DEG_PER_S * 1000
                + Math.max(0, distanceIn) / DRIVE_IN_PER_S * 1000
                + CAPTURE_MS;
    }

    /** Time to drive this far home / to park (plus turning and settling). */
    public static double driveTimeMs(double distanceIn) {
        return Math.max(0, distanceIn) / DRIVE_IN_PER_S * 1000 + 1500;
    }
}
