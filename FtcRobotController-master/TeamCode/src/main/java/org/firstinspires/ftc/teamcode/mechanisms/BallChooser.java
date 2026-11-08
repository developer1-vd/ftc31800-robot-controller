package org.firstinspires.ftc.teamcode.mechanisms;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

/**
 * Picks which ball to go for: the one that's quickest to get, not just the closest.
 *
 * Cost (in "inches of effort") = distance + TURN_COST_IN_PER_DEG x how far the robot must turn
 *                               - CLUSTER_BONUS_IN x other wanted balls next to it.
 * Skipped completely:
 *  - the other alliance's NECTAR
 *  - balls too close to a field wall to reach (needs the field position, see setWallCheck)
 *  - balls straight behind a robot the camera sees (a robot sign in the same direction)
 *  - balls it recently gave up on (giveUpOn), for GIVE_UP_MS
 *  - balls there isn't time to collect AND still do what's needed after (time budget)
 *
 * World positions use the odometry frame (start = 0,0): x forward, y left, inches.
 */
public class BallChooser {

    public static final double TURN_COST_IN_PER_DEG = 0.25;   // 40 deg of turning ~ 10 in of driving
    public static final double CLUSTER_RADIUS_IN    = 14;     // balls this close count as a cluster
    public static final double CLUSTER_BONUS_IN     = 8;      // each neighbor makes a ball "8 in closer"
    public static final double ROBOT_BLOCK_DEG      = 8;      // ball within this angle of a robot = blocked
    public static final double ROBOT_BLOCK_WIDTH    = 0.15;   // ...if the robot sign is at least this wide
    public static final double GIVE_UP_MS           = 10000;  // ignore a given-up spot this long
    public static final double GIVE_UP_RADIUS_IN    = 12;

    /** Answers "is this world position too close to a wall to reach?" */
    public interface WallCheck {
        boolean unreachable(double worldX, double worldY);
    }

    /** The decision, with the reason (for telemetry). */
    public static class Choice {
        public final BallDetector.Ball ball;   // null = none worth going for
        public final String why;
        public final double worldX, worldY;    // where the ball is (if the robot pose is known)

        Choice(BallDetector.Ball ball, String why, double worldX, double worldY) {
            this.ball = ball;
            this.why = why;
            this.worldX = worldX;
            this.worldY = worldY;
        }
    }

    private static class GaveUp {
        final double x, y;
        final long untilMs;

        GaveUp(double x, double y, long untilMs) {
            this.x = x;
            this.y = y;
            this.untilMs = untilMs;
        }
    }

    private final List<GaveUp> gaveUp = new ArrayList<>();
    private WallCheck wallCheck = null;

    public void setWallCheck(WallCheck check) {
        this.wallCheck = check;
    }

    /** Don't go for balls near this world position for GIVE_UP_MS. */
    public void giveUpOn(double worldX, double worldY, long nowMs) {
        gaveUp.add(new GaveUp(worldX, worldY, nowMs + (long) GIVE_UP_MS));
    }

    /**
     * @param balls          balls seen this frame
     * @param opponent       the other alliance's NECTAR color (skipped)
     * @param havePose       true if robotX/Y/heading are valid (odometry)
     * @param robotX         robot world position and heading (radians, CCW)
     * @param robots         robot-sign blobs the camera sees (may be empty)
     * @param msLeft         time left in the run
     * @param msNeededAfter  time still needed after picking this ball up (shoot, go home...)
     * @param nowMs          current time, for the give-up list
     */
    public Choice choose(List<BallDetector.Ball> balls, BallDetector.BallColor opponent,
                         boolean havePose, double robotX, double robotY, double robotHeadingRad,
                         List<BallDetector.RobotBlob> robots, double msLeft, double msNeededAfter,
                         long nowMs) {
        // Forget old give-ups
        for (Iterator<GaveUp> it = gaveUp.iterator(); it.hasNext(); ) {
            if (it.next().untilMs <= nowMs) it.remove();
        }

        // Wanted balls and their positions relative to the robot (x forward, y left)
        List<BallDetector.Ball> wanted = new ArrayList<>();
        for (BallDetector.Ball b : balls) if (b.color != opponent) wanted.add(b);
        if (wanted.isEmpty()) return new Choice(null, balls.isEmpty() ? "no balls in view" : "only opponent NECTAR in view", 0, 0);

        BallDetector.Ball best = null;
        double bestCost = Double.MAX_VALUE, bestX = 0, bestY = 0;
        String lastSkip = "";
        for (BallDetector.Ball b : wanted) {
            double relX = b.distance * Math.cos(Math.toRadians(b.bearing));
            double relY = b.distance * Math.sin(Math.toRadians(b.bearing));
            double wx = robotX + relX * Math.cos(robotHeadingRad) - relY * Math.sin(robotHeadingRad);
            double wy = robotY + relX * Math.sin(robotHeadingRad) + relY * Math.cos(robotHeadingRad);

            if (havePose && wallCheck != null && wallCheck.unreachable(wx, wy)) {
                lastSkip = "skipping a ball against the wall";
                continue;
            }
            if (havePose && recentlyGaveUp(wx, wy)) {
                lastSkip = "skipping a ball it gave up on";
                continue;
            }
            if (blockedByRobot(b, robots)) {
                lastSkip = "skipping a ball behind a robot";
                continue;
            }
            if (ShotPlanner.collectTimeMs(b.distance, b.bearing) + msNeededAfter > msLeft) {
                lastSkip = "no time for the balls in view";
                continue;
            }

            int neighbors = 0;
            for (BallDetector.Ball o : wanted) {
                if (o == b) continue;
                double ox = o.distance * Math.cos(Math.toRadians(o.bearing));
                double oy = o.distance * Math.sin(Math.toRadians(o.bearing));
                if (Math.hypot(ox - relX, oy - relY) <= CLUSTER_RADIUS_IN) neighbors++;
            }
            double cost = b.distance + TURN_COST_IN_PER_DEG * Math.abs(b.bearing) - CLUSTER_BONUS_IN * neighbors;
            if (cost < bestCost) {
                bestCost = cost;
                best = b;
                bestX = wx;
                bestY = wy;
            }
        }
        if (best == null) return new Choice(null, lastSkip, 0, 0);
        return new Choice(best, String.format("best %s ball (%.0f in, %.0f deg)", best.color, best.distance, best.bearing),
                bestX, bestY);
    }

    private boolean recentlyGaveUp(double x, double y) {
        for (GaveUp g : gaveUp) {
            if (Math.hypot(g.x - x, g.y - y) <= GIVE_UP_RADIUS_IN) return true;
        }
        return false;
    }

    private static boolean blockedByRobot(BallDetector.Ball b, List<BallDetector.RobotBlob> robots) {
        if (robots == null) return false;
        for (BallDetector.RobotBlob r : robots) {
            if (r.widthFraction >= ROBOT_BLOCK_WIDTH && Math.abs(r.bearing - b.bearing) <= ROBOT_BLOCK_DEG) {
                return true;
            }
        }
        return false;
    }
}
