package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.mechanisms.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.mechanisms.CellTarget;
import org.firstinspires.ftc.teamcode.mechanisms.FeederLauncher;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.LimelightVision;
import org.firstinspires.ftc.teamcode.mechanisms.ShotPlanner;
import org.firstinspires.ftc.teamcode.mechanisms.ShotSequencer;

/**
 * BIOBUZZ path-following autonomous (Pedro Pathing). The Red/Blue OpModes give the field poses.
 *
 *   start -> SHOOT pose -> camera fine-aim at our HIVE CELL -> shoot the 4 preloads
 *         -> COLLECT start -> sweep to COLLECT end with the intake on (e.g. along a GARDEN)
 *         -> SHOOT pose -> fine-aim -> shoot -> PARK pose (LOADING ZONE)
 * With PARK_TIME_MS left it stops whatever it's doing and goes to PARK.
 *
 * Poses are Pedro field coordinates (inches, 0-144, heading in degrees), the same system as
 * last season's Pedro autos. Until posesSet() is true it does NOT drive the planned route
 * (wrong poses would drive it into the HIVE or a wall): it only moves LEAVE_ONLY_IN straight
 * forward for LEAVE points.
 */
public abstract class BiobuzzPathAuto extends OpMode {

    // ---- Things each alliance's OpMode defines ----
    protected abstract boolean redAlliance();
    protected abstract boolean posesSet();
    protected abstract Pose startPose();
    protected abstract Pose shootPose();
    protected abstract Pose collectStartPose();
    protected abstract Pose collectEndPose();
    protected abstract Pose parkPose();

    // ---- Tuning ----
    private static final double AUTO_LENGTH_MS   = 30000;
    private static final double PARK_TIME_MS     = 5000;    // with this much left, go park
    private static final double LEAVE_ONLY_IN    = 14;      // poses not set: just drive this far forward
    private static final double COLLECT_POWER    = 0.45;    // slower while sweeping up balls
    private static final int    PRELOAD_BALLS    = 4;
    private static final int    BALLS_AFTER_COLLECT = 2;    // one left+right pair after collecting
    // Shooting: same as BallSpeakerAuto / the teleop (re-tune for the HIVE)
    private static final double SHOOT_RPM        = 2400;
    private static final double INTAKE_IN_POWER  = 1.0;     // +1 = in (direction: Intake.INTAKE_DIRECTION)
    private static final double FEED_INTAKE_MS   = 1000;
    private static final double SHOT_DELAY_MS    = 2000;
    private static final double SHOOT_TIMEOUT_MS = 12000;
    // Camera fine-aim at the HIVE before shooting
    private static final double AIM_BEARING_DEG  = -3.0;    // same aim offset as the teleop shoot zone
    private static final double AIM_TOL_DEG      = 2.0;
    private static final double AIM_GAIN         = 0.02;    // turn power per degree
    private static final double AIM_MAX_TURN     = 0.35;
    private static final double AIM_TIMEOUT_MS   = 1500;    // give up aiming and shoot anyway

    private enum State { DRIVE_TO_SHOOT, AIM, SHOOT, DRIVE_TO_COLLECT, COLLECT, DRIVE_TO_PARK, LEAVE_ONLY, DONE }

    private Follower follower;
    private State state;
    private boolean secondVolley = false;
    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime matchTimer = new ElapsedTime();

    private final Intake intake = new Intake();   // goBILDA Yellow Jacket 312 RPM
    private boolean hasIntake = false;
    private final FeederLauncher left = new FeederLauncher(), right = new FeederLauncher();
    private ShotSequencer shots;
    private LimelightVision limelight;
    private final AprilTagWebcam webcam = new AprilTagWebcam();
    private boolean webcamOk = false;
    private String problem = "";

    @Override
    public void init() {
        try {
            follower = Constants.createFollower(hardwareMap);
        } catch (Exception e) {
            problem = "Pedro follower failed (drive motors / pinpoint?): " + e.getMessage();
        }
        hasIntake = intake.init(hardwareMap, telemetry, "intake");
        left.init(hardwareMap, telemetry, "launcher", "feederServoLeft");
        right.init(hardwareMap, telemetry, "launcher", "feederServoRight");
        shots = new ShotSequencer(left, right, hasIntake ? intake : null, INTAKE_IN_POWER, SHOOT_RPM, FEED_INTAKE_MS, SHOT_DELAY_MS);

        // HIVE camera: Limelight if configured, otherwise the webcam (AprilTags only)
        limelight = LimelightVision.tryCreate(hardwareMap, telemetry);
        if (limelight == null) {
            try {
                webcam.init(hardwareMap, telemetry);
                webcamOk = true;
            } catch (Exception e) {
                telemetry.addLine("No HIVE camera: shooting without camera fine-aim.");
            }
        }
        if (follower != null) follower.setStartingPose(startPose());

        telemetry.addData("Alliance", redAlliance() ? "RED" : "BLUE");
        if (!posesSet()) {
            telemetry.addLine("⚠️ Field poses not set in " + getClass().getSimpleName()
                    + ": it will only drive " + (int) LEAVE_ONLY_IN + " in forward (LEAVE).");
        }
        if (!problem.isEmpty()) telemetry.addLine("⚠️ " + problem);
    }

    @Override
    public void start() {
        matchTimer.reset();
        if (follower == null) {
            setState(State.DONE);
        } else if (!posesSet()) {
            Pose s = startPose();
            Pose ahead = new Pose(s.getX() + LEAVE_ONLY_IN * Math.cos(s.getHeading()),
                    s.getY() + LEAVE_ONLY_IN * Math.sin(s.getHeading()), s.getHeading());
            follower.followPath(line(s, ahead), true);
            setState(State.LEAVE_ONLY);
        } else {
            follower.followPath(line(startPose(), shootPose()), true);
            left.launch(0, 0, false, false);      // start the flywheel early (like the other autos)
            right.launch(0, 0, false, false);
            setState(State.DRIVE_TO_SHOOT);
        }
    }

    @Override
    public void loop() {
        if (follower != null) follower.update();
        if (limelight != null) {
            limelight.setMode(LimelightVision.Mode.HIVE, null);
            limelight.update();
        } else if (webcamOk) {
            webcam.update();
        }
        double msLeft = AUTO_LENGTH_MS - matchTimer.milliseconds();
        if (posesSet() && follower != null && msLeft < PARK_TIME_MS
                && state != State.DRIVE_TO_PARK && state != State.DONE) {
            goPark();
        }

        String status;
        switch (state) {
            case DRIVE_TO_SHOOT:
                status = "driving to the shooting spot";
                if (!follower.isBusy()) startAim();
                break;

            case AIM:
                status = aim();
                break;

            case SHOOT:
                status = shots.update();
                if (shots.isDone() || stateTimer.milliseconds() > SHOOT_TIMEOUT_MS) {
                    shots.stop();
                    if (secondVolley) {
                        goPark();
                    } else {
                        secondVolley = true;
                        follower.followPath(line(follower.getPose(), collectStartPose()), true);
                        setState(State.DRIVE_TO_COLLECT);
                    }
                }
                break;

            case DRIVE_TO_COLLECT:
                status = "driving to collect";
                if (!follower.isBusy()) {
                    setIntake(INTAKE_IN_POWER);
                    follower.setMaxPower(COLLECT_POWER);
                    follower.followPath(line(collectStartPose(), collectEndPose()), true);
                    setState(State.COLLECT);
                }
                break;

            case COLLECT:
                status = "collecting (intake on)";
                setIntake(INTAKE_IN_POWER);
                if (!follower.isBusy()) {
                    setIntake(0);
                    follower.setMaxPower(1.0);
                    follower.followPath(line(collectEndPose(), shootPose()), true);
                    setState(State.DRIVE_TO_SHOOT);
                }
                break;

            case DRIVE_TO_PARK:
                status = "parking";
                if (!follower.isBusy()) setState(State.DONE);
                break;

            case LEAVE_ONLY:
                status = "poses not set: LEAVE only";
                if (!follower.isBusy()) setState(State.DONE);
                break;

            case DONE:
            default:
                status = "done";
                setIntake(0);
                break;
        }
        left.update();
        right.update();

        telemetry.addData("Auto", "%s  (%.1f s left)", status, Math.max(0, msLeft) / 1000);
        if (follower != null) {
            Pose p = follower.getPose();
            telemetry.addData("Pose", "x %.1f  y %.1f  heading %.0f", p.getX(), p.getY(), Math.toDegrees(p.getHeading()));
        }
        if (!problem.isEmpty()) telemetry.addLine("⚠️ " + problem);
        telemetry.update();
    }

    // -------------------------------------------------

    private void startAim() {
        follower.startTeleopDrive();   // turn in place under our control
        setState(State.AIM);
    }

    /** Turn in place until our HIVE CELL is centered (camera), then shoot. */
    private String aim() {
        CellTarget cell = limelight != null ? limelight.getCellTarget(redAlliance())
                : webcamOk ? webcam.getCellTarget(redAlliance()) : null;
        double error = cell == null ? 0 : cell.bearingDeg - AIM_BEARING_DEG;   // + = HIVE is LEFT
        boolean aimed = cell != null && Math.abs(error) <= AIM_TOL_DEG;
        if (aimed || stateTimer.milliseconds() > AIM_TIMEOUT_MS) {
            follower.setTeleOpDrive(0, 0, 0, true);
            // Flywheel speed from the distance to the HIVE (ShotPlanner's table), if we can see it
            shots.setRpm(cell != null ? ShotPlanner.rpmForRange(cell.rangeIn) : SHOOT_RPM);
            shots.start(secondVolley ? BALLS_AFTER_COLLECT : PRELOAD_BALLS);
            setState(State.SHOOT);
            return aimed ? "aimed at the HIVE" : "aim timeout - shooting from the path heading";
        }
        // Pedro teleop drive: turn > 0 = turn LEFT
        double turn = cell == null ? 0 : Range.clip(error * AIM_GAIN, -AIM_MAX_TURN, AIM_MAX_TURN);
        follower.setTeleOpDrive(0, 0, turn, true);
        return cell == null ? "looking for the HIVE" : String.format("aiming: %.1f deg off", error);
    }

    private void goPark() {
        shots.stop();
        setIntake(0);
        follower.setMaxPower(1.0);
        follower.followPath(line(follower.getPose(), parkPose()), true);
        setState(State.DRIVE_TO_PARK);
    }

    private static Path line(Pose from, Pose to) {
        Path path = new Path(new BezierLine(from, to));
        path.setLinearHeadingInterpolation(from.getHeading(), to.getHeading());
        return path;
    }

    private void setState(State s) {
        state = s;
        stateTimer.reset();
    }

    private void setIntake(double power) {
        if (hasIntake) intake.run(power);
    }

    /** Pedro pose from field inches and a heading in degrees. */
    protected static Pose pose(double x, double y, double headingDeg) {
        return new Pose(x, y, Math.toRadians(headingDeg));
    }

    @Override
    public void stop() {
        shots.stop();
        setIntake(0);
        if (limelight != null) limelight.stop();
        else if (webcamOk) webcam.stop();
    }
}
