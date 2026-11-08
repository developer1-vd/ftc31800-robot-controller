package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * "Turn to look": when the camera sees nothing, the robot turns in place to search.
 *
 * It turns in short steps with pauses (turn TURN_MS, hold still PAUSE_MS, repeat) instead of
 * spinning continuously, because the camera image is blurry while the robot turns and balls /
 * tags are much easier to detect when it is still. Call reset() whenever the target is seen,
 * and turnPower() each loop while it isn't.
 */
public class LookAroundScanner {

    private final double power;     // turn power while stepping
    private final double turnMs;    // how long each turn step lasts
    private final double pauseMs;   // how long to hold still and look after each step
    private final ElapsedTime timer = new ElapsedTime();
    private boolean active = false;

    /** Default: 0.25 power, turn 0.35 s, look 0.45 s (roughly 30-40 degrees per step). */
    public LookAroundScanner() {
        this(0.25, 350, 450);
    }

    public LookAroundScanner(double power, double turnMs, double pauseMs) {
        this.power = power;
        this.turnMs = turnMs;
        this.pauseMs = pauseMs;
    }

    /** Target found (or searching stopped): the next search starts with a fresh look. */
    public void reset() {
        active = false;
    }

    /**
     * Turn power for right now, + = turn RIGHT (same as gamepad right_stick_x / rx).
     * Searches to the LEFT (counter-clockwise), like the goal search.
     */
    public double turnPower() {
        if (!active) {
            active = true;
            timer.reset();
            return 0;                       // look first before moving
        }
        double t = timer.milliseconds() % (turnMs + pauseMs);
        return t < pauseMs ? 0 : -power;    // pause (look), then a turn step
    }

    /** True while in a turn step (for telemetry: "turning" vs "looking"). */
    public boolean isTurning() {
        return active && timer.milliseconds() % (turnMs + pauseMs) >= pauseMs;
    }
}
