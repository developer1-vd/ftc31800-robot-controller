package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Shoots the balls in the robot: the same pattern as the competition autos.
 * Run the intake to move balls up to the feeders, shoot left, wait, shoot right, then feed again
 * for a 3rd/4th ball. We can't tell which side each ball is on, so it always fires left AND
 * right: 1-2 balls -> 2 shots, 3-4 balls -> 4 shots.
 *
 * Call start(balls), then update() every loop until isDone(). The caller must also call
 * update() on both FeederLaunchers every loop (after this), as the autos already do.
 */
public class ShotSequencer {

    private enum Step { FEED, FIRE, WAIT_LAUNCHED, DELAY }

    private final FeederLauncher left, right;
    private final DcMotor intake;          // may be null
    private final double intakePower, rpm, feedMs, delayMs;

    private Step step = Step.FEED;
    private final ElapsedTime timer = new ElapsedTime();
    private int shotsFired = 0, shotsToFire = 0;
    private boolean leftNext = true;

    public ShotSequencer(FeederLauncher left, FeederLauncher right, DcMotor intake,
                         double intakePower, double rpm, double feedMs, double delayMs) {
        this.left = left;
        this.right = right;
        this.intake = intake;
        this.intakePower = intakePower;
        this.rpm = rpm;
        this.feedMs = feedMs;
        this.delayMs = delayMs;
    }

    /** Begin shooting this many balls (rounded up to left+right pairs). */
    public void start(int balls) {
        shotsFired = 0;
        shotsToFire = ((Math.max(balls, 1) + 1) / 2) * 2;
        leftNext = true;
        step = Step.FEED;
        timer.reset();
    }

    public boolean isDone() {
        return shotsFired >= shotsToFire;
    }

    /** Advance the sequence. Returns a status line for telemetry. */
    public String update() {
        if (isDone()) {
            setIntake(0);
            return "done shooting";
        }
        switch (step) {
            case FEED:
                setIntake(intakePower);
                if (timer.milliseconds() > feedMs) {
                    setIntake(0);
                    step = Step.FIRE;
                }
                return "feeding balls to launcher";

            case FIRE:
                (leftNext ? left : right).launch(rpm, 0, true, true);
                step = Step.WAIT_LAUNCHED;
                return "spinning up";

            case WAIT_LAUNCHED:
                if ((leftNext ? left : right).getStatus() == FeederLauncher.LaunchState.LAUNCHED) {
                    shotsFired++;
                    // After a left+right pair, feed the next balls up; otherwise switch sides
                    boolean pairDone = !leftNext;
                    leftNext = !leftNext;
                    step = pairDone ? Step.FEED : Step.DELAY;
                    timer.reset();
                }
                return "shooting (" + (shotsFired + 1) + " of " + shotsToFire + ")";

            case DELAY:
            default:
                if (timer.milliseconds() > delayMs) step = Step.FIRE;
                return "waiting between shots";
        }
    }

    /** Stop the intake and both flywheels/feeders. */
    public void stop() {
        setIntake(0);
        left.stop();
        right.stop();
    }

    private void setIntake(double power) {
        if (intake != null) intake.setPower(power);
    }
}
