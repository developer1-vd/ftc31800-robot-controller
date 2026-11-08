package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.mechanisms.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.mechanisms.BallChooser;
import org.firstinspires.ftc.teamcode.mechanisms.BallDetector;
import org.firstinspires.ftc.teamcode.mechanisms.CellTarget;
import org.firstinspires.ftc.teamcode.mechanisms.FeederLauncher;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.LimelightVision;
import org.firstinspires.ftc.teamcode.mechanisms.LookAroundScanner;
import org.firstinspires.ftc.teamcode.mechanisms.PinpointOdometry;
import org.firstinspires.ftc.teamcode.mechanisms.ShotPlanner;
import org.firstinspires.ftc.teamcode.mechanisms.ShotSequencer;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * BIOBUZZ (2026-27) autonomous. Watches for yellow POLLEN and red/blue NECTAR, says the color
 * out loud on the Driver Station ("red ball"), turns to face the closest ball it wants (POLLEN
 * or our alliance's NECTAR, see RED_ALLIANCE), drives to it, and picks it up with the intake.
 *
 * Works with whatever hardware is configured:
 * - Camera only (Control Hub + battery + "Webcam 1", or a Limelight 3A named "limelight"):
 *   detects and speaks. With a Limelight it is used instead of the webcam: HIVE pipeline while
 *   finding / lining up on the goal, the POLLEN pipeline (or LIMELIGHT_BALL_COLOR) otherwise.
 *   See LimelightVision for the pipeline setup.
 * - + drive motors: also turns and drives to the ball, stopping STOP_DISTANCE_IN away.
 * - + "intake" motor: drives onto the ball with the intake running and picks it up,
 *   says "got ball", then goes after the next one (up to MAX_BALLS).
 * - + "launcher" motor and feeder servos: when full (or holding balls with none left in
 *   view), turns until it sees our HIVE CELL's AprilTag cluster, lines up in the same shoot
 *   zone as TeamBotTeleop_latest, LAUNCHES into the CELL, then collects again.
 * - + "pinpoint" odometry: after shooting, drives back to HOME_OFFSET_IN in front of where it
 *   started (off the wall, so LEAVE still counts), says "home", then collects again. At the
 *   end it goes there, or to a LOADING ZONE PARK spot if PARK_IN_LOADING_ZONE is set.
 *
 * Avoiding other robots (AVOID_ROBOTS):
 * - "frontDistance" distance sensor (optional): something closer than OBSTACLE_STOP_IN while
 *   driving forward = robot ahead. Mount it ABOVE ball height so balls don't trigger it.
 * - Camera (CAMERA_AVOID): a large red/blue ROBOT SIGN-colored blob straight ahead, wider than
 *   ROBOT_CLOSE_WIDTH of the image and not in the top of the image = robot ahead. Skipped while
 *   a HIVE CELL tag cluster is in view and while lining up, since the HIVE is red/blue too.
 * - Pinpoint odometry: told to drive but not moving for STALL_MS = bumped into something.
 *
 * Avoiding field walls (AVOID_WALLS, needs Pinpoint): set START_FIELD_X/Y/HEADING to where the
 * robot starts, in the same 0-144 inch field coordinates the Pedro autos use. The robot then
 * knows where it is on the field, and any drive command toward a wall closer than
 * WALL_MARGIN_IN is removed (it can still slide along or back away from the wall). If a wall
 * keeps it from getting anywhere for WALL_STUCK_MS, it backs off and turns away.
 * Either way it stops and waits briefly (the other robot may move), then backs up, turns
 * away (alternating left/right), and goes back to what it was doing.
 *
 * The "brain" (each part can be turned off with its SMART_ flag):
 * - SMART_SEARCH: when it loses sight of something it turns toward where it last saw it; after
 *   looking all the way around (360 deg) with nothing found it drives to a new spot (toward the
 *   field middle if the field position is known) and looks again; if it keeps having to avoid
 *   robots in one place it moves elsewhere. The HIVE search turns straight to where the HIVE was.
 * - SMART_BALL_CHOICE (BallChooser): goes for the quickest ball (distance + turning), prefers
 *   clusters, skips balls against walls, behind robots, or ones it recently gave up on.
 * - SMART_SHOOTING (ShotPlanner): shoots from anywhere inside its RPM table's range with the
 *   flywheel speed set from the HIVE distance, and only once aimed and holding still.
 * - SMART_TIME: only chases a ball if there's time to get it AND still shoot and get home; goes to
 *   shoot what it has when time gets short; gives up on a ball it can't reach (7 s), and on
 *   shooting if the HIVE can't be found twice; heads home based on how far away home is.
 *
 * The whole cycle (collect -> shoot -> return to start) repeats until AUTO_LENGTH_MS runs out.
 * FINAL_RETURN_MS before the end it stops whatever it is doing, drives back to the start
 * one last time, and stops.
 *
 * The robot never moves during INIT, and stops when no ball is in view.
 * Assumes the camera and intake both face the FRONT of the robot.
 */
@Autonomous(name = "BallSpeakerAuto", group = "Vision")
public class BallSpeakerAuto extends OpMode {

    // Spoken announcements on the Driver Station. Set to false for silence.
    private static final boolean SOUND_ENABLED = true;

    // Turn toward the closest ball. Set to false to only detect and speak.
    private static final boolean TURN_TO_BALL = true;

    // Drive up to the ball after facing it. Set to false to only turn.
    private static final boolean DRIVE_TO_BALL = true;

    // Pick the ball up with the intake (needs an "intake" motor). Set to false to just drive up to it.
    private static final boolean PICK_UP_BALL = true;

    // Proportional turn gain (power per degree of bearing) and limits
    private static final double TURN_GAIN     = 0.02;
    private static final double MAX_TURN      = 0.4;
    private static final double AIM_TOLERANCE = 2.0;  // degrees

    // Driving: only drive forward when roughly facing the ball, slow down as it gets close
    private static final double DRIVE_GAIN         = 0.02;  // power per inch of remaining distance
    private static final double MAX_DRIVE          = 0.3;   // start slow; raise once it works
    private static final double MIN_DRIVE          = 0.1;   // enough power to actually move
    private static final double DRIVE_MAX_BEARING  = 15.0;  // degrees; turn first if more off than this
    private static final double STOP_DISTANCE_IN   = 8.0;   // no intake: stop this far from the ball

    // Pickup (with intake)
    // How hard to pull in (+1 = full IN_RPM). Which way is "in" is set in Intake.INTAKE_DIRECTION.
    private static final double INTAKE_IN_POWER     = 1.0;
    private static final double CAPTURE_DISTANCE_IN = 10.0;  // this close -> drive straight onto it
    private static final double CAPTURE_DRIVE       = 0.25;  // forward power while swallowing the ball
    private static final double CAPTURE_MS          = 800;   // how long to drive onto the ball
    private static final double INTAKE_SETTLE_MS    = 500;   // keep intake running after, to pull it in
    private static final int    MAX_BALLS           = 4;     // robot capacity (BIOBUZZ preload is 4 POLLEN)

    // Shooting (needs "launcher" motor + "feederServoLeft"/"feederServoRight"). Set false to skip.
    private static final boolean SHOOT_BALLS = true;
    // BIOBUZZ: which alliance we are. SET THIS before every match!
    // The robot only LAUNCHES into its own alliance's HIVE CELL (red tags 30-37, blue 38-45)
    // and only picks up yellow POLLEN and its own alliance's NECTAR.
    // (BiobuzzAutoRed / BiobuzzAutoBlue set this for you; this value is for BallSpeakerAuto.)
    private static final boolean RED_ALLIANCE         = true;
    // Shoot zone, same values as TeamBotTeleop_latest (range/bearing to the CELL tag cluster).
    // These came from last season's goal; the CELL is ~44 in up in the field center: RE-TUNE.
    private static final double SHOOT_RANGE_IN        = 36.0;
    private static final double SHOOT_RANGE_TOL_IN    = 6.0;
    private static final double SHOOT_BEARING_DEG     = -3.0;
    private static final double SHOOT_BEARING_TOL_DEG = 3.0;
    private static final double SHOOT_RPM             = 2400;   // teleop default / RT preset
    private static final double SHOOT_ARM_POSITION    = 0.5;    // same arm position the autos use
    private static final double GOAL_SEARCH_MS        = 6000;   // give up finding the goal after this
    private static final double GOAL_LOST_MS          = 500;    // tag unseen this long -> search again
    private static final double SHOOT_WHEN_IDLE_MS    = 6000;   // holding balls, none in view -> go shoot
    private static final double FEED_INTAKE_MS        = 1000;   // intake run that moves balls to feeders
    private static final double SHOT_DELAY_MS         = 2000;   // between shots (same as the autos)
    private static final double SHOOT_TIMEOUT_MS      = 15000;  // safety: give up a stuck shot sequence

    // Drive back to the starting spot after shooting (needs the "pinpoint" odometry computer).
    // Set to false to keep collecting and shooting instead.
    private static final boolean RETURN_TO_START = true;
    // BIOBUZZ: robots start against the perimeter wall, and LEAVE (3 pts) only counts if the robot
    // is NOT touching the wall at the end of AUTO. So "home" is this far in front of the start.
    private static final double HOME_OFFSET_IN = 12.0;
    // End of AUTO: PARK (5 pts) = at least partially in your alliance's LOADING ZONE (23 x 11 in
    // against the perimeter by your ALLIANCE AREA). Set PARK_IN_LOADING_ZONE = true and the field
    // coordinates (same 0-144 in system as START_FIELD_*) of a spot partly inside it, not touching
    // the wall. Otherwise the final trip goes to "home" above.
    private static final boolean PARK_IN_LOADING_ZONE    = false;
    private static final double  PARK_FIELD_X            = 0.0;
    private static final double  PARK_FIELD_Y            = 0.0;
    private static final double  PARK_FIELD_HEADING_DEG  = 0.0;
    // Pinpoint pod setup, same as pedroPathing/Constants.java (forwardPodY = 0, strafePodX = -2)
    private static final double PINPOINT_X_OFFSET_IN = 0.0;
    private static final double PINPOINT_Y_OFFSET_IN = -2.0;
    // Return-to-start driving
    private static final double HOME_DRIVE_GAIN    = 0.04;   // power per inch of distance left
    private static final double HOME_TURN_GAIN     = 0.02;   // power per degree of heading left
    private static final double HOME_MAX_POWER     = 0.4;
    private static final double HOME_MIN_POWER     = 0.1;
    private static final double HOME_POSITION_TOL_IN = 2.0;
    private static final double HOME_HEADING_TOL_DEG = 3.0;
    private static final double HOME_TIMEOUT_MS    = 10000;  // safety: stop trying after this

    // Repeat collect -> shoot -> return until time runs out
    private static final double AUTO_LENGTH_MS  = 30000;   // FTC autonomous is 30 s; raise for practice
    private static final double FINAL_RETURN_MS = 6000;    // with this much left, head back to start

    // Avoid other robots. Set to false to turn off.
    private static final boolean AVOID_ROBOTS        = true;
    private static final double OBSTACLE_STOP_IN     = 14.0;   // frontDistance closer than this = blocked
    private static final double STALL_MIN_POWER      = 0.15;   // only check for bumps when driving this hard
    private static final double STALL_SPEED_IN_PER_S = 2.0;    // slower than this = not moving
    private static final double STALL_MS             = 700;    // not moving this long = bumped
    private static final double AVOID_WAIT_MS        = 1000;   // wait for the other robot to move first
    private static final double AVOID_BACK_POWER     = 0.3;
    private static final double AVOID_BACK_MS        = 500;
    private static final double AVOID_TURN_POWER     = 0.35;
    private static final double AVOID_TURN_MS        = 600;    // roughly a 45-90 degree turn; tune
    // Field walls (needs Pinpoint). Field is 144 x 144 inches, same coordinates as the Pedro autos.
    private static final boolean AVOID_WALLS          = true;
    // Where the robot starts on the field. SET THESE for your starting spot!
    // Default is the field center facing +X, which keeps the robot >= ~60 in from walls only
    // if it really starts there.
    private static final double START_FIELD_X           = 72.0;
    private static final double START_FIELD_Y           = 72.0;
    private static final double START_FIELD_HEADING_DEG = 0.0;
    private static final double FIELD_SIZE_IN    = 144.0;
    private static final double WALL_MARGIN_IN   = 12.0;   // robot center stays this far from walls
    private static final double WALL_STUCK_MS    = 1500;   // blocked by a wall this long -> back off
    // Camera robot detection
    private static final boolean CAMERA_AVOID        = true;
    private static final double ROBOT_CLOSE_WIDTH    = 0.30;   // blob this wide (fraction of image) = close
    private static final double ROBOT_AHEAD_DEG      = 20.0;   // blob within this angle of center = in our path
    private static final double ROBOT_MIN_IMAGE_Y    = 0.30;   // ignore blobs in the top 30% (the HIVE)

    // Turn to look: with no ball in view, turn in place in steps to search (see LookAroundScanner)
    private static final boolean LOOK_AROUND_FOR_BALLS = true;
    private final LookAroundScanner ballScanner = new LookAroundScanner();
    private final LookAroundScanner goalScanner = new LookAroundScanner();

    // ---- The "brain": each part can be turned off ----
    private static final boolean SMART_SEARCH      = true;   // remember where things were, full scan, then move
    private static final boolean SMART_BALL_CHOICE = true;   // quickest ball, skip walls/robots/given-up, clusters
    private static final boolean SMART_SHOOTING    = true;   // RPM from HIVE distance, shoot only when steady
    private static final boolean SMART_TIME        = true;   // plan around the clock
    private static final double CHASE_GIVE_UP_MS     = 7000;  // chasing one ball this long without it -> give up
    private static final double SCAN_NO_ODOM_MS      = 12000; // no Pinpoint: scanning this long ~ a full circle
    private static final double RELOCATE_IN          = 24;    // then drive this far to look from a new spot
    private static final double RELOCATE_POWER       = 0.35;
    private static final double RELOCATE_NO_ODOM_MS  = 1500;
    private static final double AIM_STEADY_MS        = 300;   // aimed + in range this long before shooting
    private static final double STEADY_SPEED_IN_PER_S = 3;
    private static final double LINE_UP_TIME_MS      = 3000;  // allowance to find + line up on the HIVE
    private static final int    MAX_GOAL_FAILURES    = 2;     // give up shooting after this many failed searches
    private static final int    AVOID_LOOP_COUNT     = 3;     // this many avoids...
    private static final double AVOID_LOOP_WINDOW_MS = 15000; // ...this close together = crowded, move away
    private static final double FINAL_RETURN_MIN_MS  = 2500;
    private static final double ROBOT_FRONT_IN       = 9;     // robot center to the intake (half robot length)

    private final BallChooser chooser = new BallChooser();
    private BallChooser.Choice lastChoice = null;
    private String brain = "";                       // latest decision, shown in telemetry
    private final ElapsedTime clock = new ElapsedTime();
    // Search memory
    private double lastBallBearing = 1;              // + = it was to the LEFT
    private double lastBallHeadingDeg = Double.NaN;  // odometry heading it was seen at
    private double lastGoalHeadingDeg = Double.NaN;
    private double lastGoalRange = ShotPlanner.MIN_RANGE_IN;
    private boolean goalMemoryUsed = false;
    private double scanTurnedDeg = 0, scanPrevHeading = Double.NaN;
    private boolean relocating = false, relocateForced = false, relocateRequested = false;
    private double relocateStartX = 0, relocateStartY = 0;
    private final ElapsedTime relocateTimer = new ElapsedTime();
    // Chase / shooting / failure memory
    private double chaseX = Double.NaN, chaseY = Double.NaN;
    private final ElapsedTime chaseTimer = new ElapsedTime();
    private boolean steady = false;
    private final ElapsedTime steadyTimer = new ElapsedTime();
    private int goalFailures = 0;
    private boolean shootingGivenUp = false;
    private final java.util.ArrayDeque<Long> avoidTimes = new java.util.ArrayDeque<>();

    // Leaving the wall at the start (competition autos): drive forward this far / this long
    private static final double LEAVE_WALL_POWER = 0.3;
    private static final double LEAVE_WALL_MS    = 1500;   // time limit (and the only limit without Pinpoint)

    private enum State { LEAVE_WALL, SEEK, CAPTURE, SETTLE, FIND_GOAL, LINE_UP, SHOOT, RETURN_HOME, AVOID, DONE, FULL }
    private enum AvoidStep { WAIT, BACK_UP, TURN }
    private State state = State.SEEK;
    private final ElapsedTime stateTimer = new ElapsedTime();
    private int ballsCollected = 0;
    private final ElapsedTime lastBallSeenTimer = new ElapsedTime();
    private final ElapsedTime goalLastSeenTimer = new ElapsedTime();

    // Shooting (feed, left, right, ...): see ShotSequencer
    private ShotSequencer shots = null;

    private boolean announcedArrival = false;

    private final BallDetector ballDetector = new BallDetector();
    private final AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();   // shares "Webcam 1"
    private LimelightVision limelight = null;   // used instead of the webcam if configured
    // With both cameras: the Limelight only watches AprilTags, the webcam finds the balls
    private boolean limelightTagsOnly = false;
    // Which ball color pipeline the Limelight runs while collecting (one color at a time)
    private static final BallDetector.BallColor LIMELIGHT_BALL_COLOR = BallDetector.BallColor.YELLOW;
    private final FeederLauncher leftLauncher  = new FeederLauncher();
    private final FeederLauncher rightLauncher = new FeederLauncher();

    private DcMotor frontLeft  = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft   = null;
    private DcMotor backRight  = null;
    private final Intake intake = new Intake();   // goBILDA Yellow Jacket 312 RPM
    private boolean hasIntake  = false;
    private Servo   arm        = null;
    private PinpointOdometry odometry = null;   // null if no "pinpoint" configured
    private DistanceSensor frontDistance = null; // null if no "frontDistance" configured

    // Obstacle avoidance
    private double lastForward = 0, lastStrafe = 0;   // last commanded drive (for bump detection)
    private final ElapsedTime stallTimer = new ElapsedTime();
    private final ElapsedTime avoidTimer = new ElapsedTime();
    private AvoidStep avoidStep = AvoidStep.WAIT;
    private State resumeState = State.SEEK;
    private boolean avoidBySensor = false;
    private double avoidTurnDirection = 1;             // +1 right, -1 left; alternates each time
    private boolean wallBlocking = false;              // last drive command was cut by a wall
    private final ElapsedTime wallStuckTimer = new ElapsedTime();
    private final ElapsedTime matchTimer = new ElapsedTime();
    private boolean finalReturn = false;        // true once we're heading home for the last time
    private boolean started = false;            // Play has been pressed
    private boolean hasDrive   = false;
    private boolean hasLauncher = false;

    @Override
    public void init() {
        frontLeft  = getMotor("left_front_drive");
        frontRight = getMotor("right_front_drive");
        backLeft   = getMotor("left_back_drive");
        backRight  = getMotor("right_back_drive");
        hasIntake  = intake.init(hardwareMap, telemetry, "intake");
        hasLauncher = getMotor("launcher") != null;
        hasDrive = frontLeft != null || frontRight != null || backLeft != null || backRight != null;

        if (frontLeft  != null) frontLeft.setDirection(DcMotor.Direction.REVERSE);
        if (backLeft   != null) backLeft.setDirection(DcMotor.Direction.REVERSE);
        if (frontRight != null) frontRight.setDirection(DcMotor.Direction.FORWARD);
        if (backRight  != null) backRight.setDirection(DcMotor.Direction.FORWARD);
        for (DcMotor m : new DcMotor[] { frontLeft, frontRight, backLeft, backRight }) {
            if (m != null) m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // Camera: a Limelight 3A named "limelight" if configured, otherwise "Webcam 1"
        limelight = LimelightVision.tryCreate(hardwareMap, telemetry);
        if (limelight != null && BallDetector.webcamConfigured(hardwareMap)) {
            // Both cameras: Limelight = HIVE AprilTags only, C270 webcam = all balls
            limelightTagsOnly = true;
            ballDetector.init(hardwareMap, telemetry);
            telemetry.addLine("Cameras: Limelight for AprilTags, Webcam 1 for balls");
        } else if (limelight != null) {
            ballDetector.useLimelight(limelight, telemetry);
        } else {
            // Ball detection + AprilTags (for finding the goal) on the same webcam
            ballDetector.buildProcessors(telemetry);
            try {
                aprilTagWebcam.init(hardwareMap, telemetry, ballDetector.getProcessors());
            } catch (Exception e) {
                telemetry.addLine("⚠️ No camera found: no Limelight 3A and no \"Webcam 1\" in the robot"
                        + " configuration. Run LimelightTest for setup help.");
            }
        }
        ballDetector.setSoundEnabled(SOUND_ENABLED);

        if (hasLauncher) {
            leftLauncher.init(hardwareMap, telemetry, "launcher", "feederServoLeft");
            rightLauncher.init(hardwareMap, telemetry, "launcher", "feederServoRight");
            shots = new ShotSequencer(leftLauncher, rightLauncher, hasIntake ? intake : null, INTAKE_IN_POWER,
                    SHOOT_RPM, FEED_INTAKE_MS, SHOT_DELAY_MS);
        }
        try {
            // Start pose (0, 0, 0) = wherever the robot is when Play is pressed
            odometry = new PinpointOdometry(hardwareMap, 0, 0, 0,
                    PINPOINT_X_OFFSET_IN, PINPOINT_Y_OFFSET_IN,
                    GoBildaPinpointDriver.EncoderDirection.REVERSED,
                    GoBildaPinpointDriver.EncoderDirection.REVERSED);
        } catch (Exception e) {
            odometry = null;
        }
        try {
            frontDistance = hardwareMap.get(DistanceSensor.class, "frontDistance");
        } catch (Exception e) {
            frontDistance = null;
        }
        try {
            arm = hardwareMap.get(Servo.class, "armServo");
            arm.scaleRange(0.2, 0.8);   // same range as the autos
        } catch (Exception e) {
            arm = null;
        }

        if (hasDrive && canShoot()) {
            telemetry.addLine("Ready. After Play, picks up balls and shoots them into the goal.");
        } else if (hasDrive && canPickUp()) {
            telemetry.addLine("Ready. After Play, drives to balls and picks them up.");
        } else if (hasDrive && TURN_TO_BALL) {
            telemetry.addLine("Ready. After Play, turns and drives to the closest ball.");
        } else {
            telemetry.addLine("Camera-only ball speaker ready. Show it a ball.");
        }
    }

    @Override
    public void init_loop() {
        // Detect and speak only; the robot must not move during INIT.
        updateVision();
        report();
    }

    // -------------------------------------------------
    // Settings the competition autos (BiobuzzAutoRed / BiobuzzAutoBlue) override
    // -------------------------------------------------

    /** Which alliance we are: our HIVE CELL is the target, and the other alliance's NECTAR is skipped. */
    protected boolean redAlliance() {
        return RED_ALLIANCE;
    }

    /** Balls already in the robot at Play (BIOBUZZ robots start with 4 POLLEN). Shot first. */
    protected int preloadedBalls() {
        return 0;
    }

    /**
     * Go looking for more balls after the preloads (true), or just shoot the preloads and finish
     * (false: the simple autos). Without collecting, the robot heads home / parks when done.
     */
    protected boolean collectBalls() {
        return true;
    }

    /** How long the run lasts before the final trip home and stop (TEST MODE uses a longer one). */
    protected double autoLengthMs() {
        return AUTO_LENGTH_MS;
    }

    /** Drive off the perimeter wall first (BIOBUZZ LEAVE = not touching the wall at the end of AUTO). */
    protected boolean leaveWallFirst() {
        return false;
    }

    @Override
    public void start() {
        if (arm != null && canShoot()) arm.setPosition(SHOOT_ARM_POSITION);
        if (odometry != null) odometry.setPose(0, 0, 0);   // remember this spot as "start"
        // A ball this close to a wall can't be reached: the wall filter keeps the robot's center
        // WALL_MARGIN_IN away and the intake is ROBOT_FRONT_IN in front of it
        chooser.setWallCheck((wx, wy) -> wallsActive()
                && nearWallAt(fieldXOf(wx, wy), fieldYOf(wx, wy), WALL_MARGIN_IN - ROBOT_FRONT_IN));
        clock.reset();
        lastBallSeenTimer.reset();
        matchTimer.reset();
        started = true;

        ballsCollected = Math.min(preloadedBalls(), MAX_BALLS);
        if (leaveWallFirst() && hasDrive) {
            setState(State.LEAVE_WALL);
        } else {
            setState(afterStartState());
        }
    }

    /** With preloads, go shoot them first; otherwise start collecting. */
    private State afterStartState() {
        return (canShoot() && ballsCollected > 0) ? State.FIND_GOAL : State.SEEK;
    }

    /** Drive straight off the wall: HOME_OFFSET_IN with Pinpoint, otherwise for LEAVE_WALL_MS. */
    private String leaveWall() {
        setIntake(0);
        boolean farEnough = odometry != null && odometry.getX() >= HOME_OFFSET_IN;
        if (farEnough || stateTimer.milliseconds() > LEAVE_WALL_MS) {
            setDrivePower(0, 0, 0);
            setState(afterStartState());
            return "off the wall";
        }
        setDrivePower(LEAVE_WALL_POWER, 0, 0);
        return "leaving the wall";
    }

    @Override
    public void loop() {
        updateVision();
        if (odometry != null) odometry.update();
        if (ballDetector.hasBall()) lastBallSeenTimer.reset();
        checkTime();
        String status;

        switch (state) {
            case LEAVE_WALL:
                status = leaveWall();
                break;

            case CAPTURE:
                // The ball is now too close for the camera to see; drive straight onto it.
                setIntake(INTAKE_IN_POWER);
                setDrivePower(CAPTURE_DRIVE, 0);
                status = "picking up ball";
                if (stateTimer.milliseconds() > CAPTURE_MS) {
                    setDrivePower(0, 0);
                    ballsCollected++;
                    ballDetector.announce("got ball");
                    setState(State.SETTLE);
                }
                break;

            case SETTLE:
                // Stopped; keep the intake running a moment to pull the ball fully in.
                setDrivePower(0, 0);
                setIntake(INTAKE_IN_POWER);
                status = "pulling ball in";
                if (stateTimer.milliseconds() > INTAKE_SETTLE_MS) {
                    setIntake(0);
                    if (ballsCollected < MAX_BALLS) {
                        setState(State.SEEK);
                    } else {
                        setState(canShoot() ? State.FIND_GOAL : State.FULL);
                    }
                }
                break;

            case FIND_GOAL:
                status = findGoal();
                break;

            case LINE_UP:
                status = lineUp();
                break;

            case SHOOT:
                status = shoot();
                break;

            case RETURN_HOME:
                status = returnHome();
                break;

            case AVOID:
                status = avoid();
                break;

            case DONE:
                setDrivePower(0, 0);
                setIntake(0);
                stopLaunchers();
                status = finalReturn ? "finished - stopped" : "time's up - done";
                break;

            case FULL:
                setDrivePower(0, 0);
                setIntake(0);
                status = "full (" + ballsCollected + " balls) - stopped";
                break;

            case SEEK:
            default:
                // Holding balls but none left in view, or time getting short: go shoot what we have
                boolean idle = lastBallSeenTimer.milliseconds() > SHOOT_WHEN_IDLE_MS;
                boolean timeShort = SMART_TIME && msLeft() < LINE_UP_TIME_MS
                        + ShotPlanner.shootTimeMs(ballsCollected, FEED_INTAKE_MS, SHOT_DELAY_MS) + finalReturnMs() + 2000;
                if (canShoot() && ballsCollected > 0 && (idle || timeShort)) {
                    setState(State.FIND_GOAL);
                    status = timeShort ? "time is short - going to shoot" : "no more balls - going to shoot";
                    brain = status;
                } else {
                    status = seek();
                }
                break;
        }

        status = checkForRobots(status);

        // Launchers run their own spin-up/feed timing; keep them updated every loop
        if (hasLauncher) {
            leftLauncher.update();
            rightLauncher.update();
        }

        if (hasDrive && TURN_TO_BALL) {
            telemetry.addData("Robot", status);
        }
        report();
    }

    /**
     * The ball to go for: with SMART_BALL_CHOICE the quickest worthwhile one (BallChooser),
     * otherwise the closest yellow POLLEN / our alliance's NECTAR.
     */
    private BallDetector.Ball chooseBall() {
        BallDetector.BallColor opponent = redAlliance() ? BallDetector.BallColor.BLUE : BallDetector.BallColor.RED;
        if (!SMART_BALL_CHOICE) {
            lastChoice = null;
            for (BallDetector.Ball ball : ballDetector.getBalls()) {
                if (ball.color != opponent) return ball;
            }
            return null;
        }
        boolean pose = odometry != null && started;
        lastChoice = chooser.choose(ballDetector.getBalls(), opponent, pose,
                pose ? odometry.getX() : 0, pose ? odometry.getY() : 0,
                pose ? Math.toRadians(odometry.getHeadingDeg()) : 0,
                ballDetector.getRobotBlobs(),
                SMART_TIME ? msLeft() : Double.POSITIVE_INFINITY,
                SMART_TIME ? msNeededAfterCollect() : 0, (long) clock.milliseconds());
        brain = lastChoice.why;
        return lastChoice.ball;
    }

    /** Time still needed after picking up one more ball: find + line up + shoot, then go home. */
    private double msNeededAfterCollect() {
        double shoot = canShoot()
                ? LINE_UP_TIME_MS + ShotPlanner.shootTimeMs(ballsCollected + 1, FEED_INTAKE_MS, SHOT_DELAY_MS) : 0;
        return shoot + finalReturnMs();
    }

    /** Turn toward and drive to the closest ball. Returns a status line. */
    private String seek() {
        if (!collectBalls() && started) {
            goFinalTrip();
            return "not collecting - finishing";
        }
        // Crowded here (kept having to avoid robots): move somewhere else first
        if (relocateRequested) startRelocate(true);
        if (relocating && relocateForced) return relocate();

        BallDetector.Ball ball = chooseBall();
        if (!TURN_TO_BALL || ball == null) {
            setIntake(0);
            chaseX = Double.NaN;
            if (TURN_TO_BALL && LOOK_AROUND_FOR_BALLS && hasDrive && started) {
                return smartSearch();
            }
            setDrivePower(0, 0);
            return "no ball - holding still";
        }
        ballScanner.reset();
        relocating = false;
        scanTurnedDeg = 0;

        // Remember where it was, to look there first if it's lost
        lastBallBearing = ball.bearing;
        lastBallHeadingDeg = odometry != null ? odometry.getHeadingDeg() + ball.bearing : Double.NaN;

        // Give up on a ball it can't seem to get (blocked, rolling away...) and pick another
        if (SMART_TIME && odometry != null && lastChoice != null && lastChoice.ball != null && started) {
            if (Double.isNaN(chaseX) || Math.hypot(lastChoice.worldX - chaseX, lastChoice.worldY - chaseY) > 15) {
                chaseX = lastChoice.worldX;
                chaseY = lastChoice.worldY;
                chaseTimer.reset();
            } else if (chaseTimer.milliseconds() > CHASE_GIVE_UP_MS) {
                chooser.giveUpOn(chaseX, chaseY, (long) clock.milliseconds());
                chaseX = Double.NaN;
                setDrivePower(0, 0);
                setIntake(0);
                brain = "gave up on a ball it couldn't get";
                ballDetector.announce("skip");
                return brain;
            }
        }

        double turn = 0;     // + = turn right (same as right stick in teleop)
        double forward = 0;  // + = drive forward
        boolean facing = Math.abs(ball.bearing) <= DRIVE_MAX_BEARING;

        if (Math.abs(ball.bearing) > AIM_TOLERANCE) {
            // bearing > 0 means ball is to the LEFT, so turn left (negative turn)
            turn = Range.clip(-ball.bearing * TURN_GAIN, -MAX_TURN, MAX_TURN);
        }

        if (canPickUp()) {
            // Drive all the way onto the ball with the intake running
            if (ball.distance <= CAPTURE_DISTANCE_IN && facing) {
                setState(State.CAPTURE);
                return "picking up " + ball.color + " ball";
            }
            if (facing) {
                double remaining = ball.distance - CAPTURE_DISTANCE_IN;
                forward = Range.clip(remaining * DRIVE_GAIN, MIN_DRIVE, MAX_DRIVE);
                setIntake(INTAKE_IN_POWER);
            } else {
                setIntake(0);
            }
            setDrivePower(forward, turn);
            return (forward > 0 ? "driving to " : "turning to ") + ball.color + " ball";
        }

        // No intake: drive up and stop STOP_DISTANCE_IN away
        double remaining = ball.distance - STOP_DISTANCE_IN;
        if (DRIVE_TO_BALL && remaining > 0 && facing) {
            forward = Range.clip(remaining * DRIVE_GAIN, MIN_DRIVE, MAX_DRIVE);
        }
        setDrivePower(forward, turn);

        if (DRIVE_TO_BALL && remaining <= 0 && turn == 0) {
            if (!announcedArrival) {
                ballDetector.announce("at ball");
                announcedArrival = true;
            }
            return "at " + ball.color + " ball";
        }
        if (remaining > 4) announcedArrival = false;   // moved away -> can announce again
        return (forward > 0 ? "driving to " : "turning to ") + ball.color + " ball";
    }

    /**
     * No ball in view. Look toward where one was last seen first; after a full circle (or
     * SCAN_NO_ODOM_MS without Pinpoint) with nothing, drive to a new spot and look again.
     */
    private String smartSearch() {
        if (relocating) return relocate();
        if (SMART_SEARCH) {
            if (ballScanner.searchMs() == 0) {   // a new search: turn toward where the ball was
                double toward = lastBallBearing;
                if (odometry != null && !Double.isNaN(lastBallHeadingDeg)) {
                    toward = AngleUnit.normalizeDegrees(lastBallHeadingDeg - odometry.getHeadingDeg());
                }
                ballScanner.searchToward(toward);
                scanTurnedDeg = 0;
                scanPrevHeading = Double.NaN;
            }
            if (odometry != null) {
                double h = odometry.getHeadingDeg();
                if (!Double.isNaN(scanPrevHeading)) scanTurnedDeg += Math.abs(AngleUnit.normalizeDegrees(h - scanPrevHeading));
                scanPrevHeading = h;
            }
            boolean lookedEverywhere = odometry != null ? scanTurnedDeg >= 360
                    : ballScanner.searchMs() > SCAN_NO_ODOM_MS;
            if (lookedEverywhere) {
                startRelocate(false);
                return relocate();
            }
        }
        setDrivePower(0, ballScanner.turnPower());
        brain = SMART_SEARCH ? String.format("searching %s (%.0f deg looked)",
                ballScanner.getDirection() < 0 ? "left" : "right", scanTurnedDeg) : "searching";
        return ballScanner.isTurning() ? "no ball - turning to look" : "no ball - looking";
    }

    private void startRelocate(boolean forced) {
        relocating = true;
        relocateForced = forced;
        relocateRequested = false;
        relocateTimer.reset();
        if (odometry != null) {
            relocateStartX = odometry.getX();
            relocateStartY = odometry.getY();
        }
        ballScanner.reset();
        scanTurnedDeg = 0;
        brain = forced ? "crowded here - moving somewhere else" : "nothing here - looking from a new spot";
        ballDetector.announce("moving");
    }

    /** Drive RELOCATE_IN to a new spot: toward the field middle if the field position is known. */
    private String relocate() {
        boolean far = odometry != null
                ? Math.hypot(odometry.getX() - relocateStartX, odometry.getY() - relocateStartY) >= RELOCATE_IN
                : relocateTimer.milliseconds() > RELOCATE_NO_ODOM_MS;
        if (far || relocateTimer.milliseconds() > 4000) {
            relocating = false;
            setDrivePower(0, 0, 0);
            ballScanner.reset();
            return "at a new spot - looking";
        }
        double turn = 0;
        if (wallsActive()) {
            double toMiddle = Math.toDegrees(Math.atan2(FIELD_SIZE_IN / 2 - fieldY(), FIELD_SIZE_IN / 2 - fieldX()));
            double err = AngleUnit.normalizeDegrees(toMiddle - Math.toDegrees(fieldHeadingRad()));   // + = LEFT
            turn = Range.clip(-err * TURN_GAIN, -MAX_TURN, MAX_TURN);
            if (Math.abs(err) > 30) {
                setDrivePower(0, turn);
                return "turning toward the field middle";
            }
        }
        setDrivePower(RELOCATE_POWER, turn);
        return "moving to look from a new spot";
    }

    // -------------------------------------------------
    // Shooting
    // -------------------------------------------------

    /** Turn in place until our HIVE CELL's tag cluster is visible. */
    private String findGoal() {
        setIntake(0);
        if (getGoalTag() != null) {
            goalScanner.reset();
            goalFailures = 0;
            goalLastSeenTimer.reset();
            setState(State.LINE_UP);
            return "found goal";
        }
        if (stateTimer.milliseconds() > GOAL_SEARCH_MS) {
            setDrivePower(0, 0);
            ballDetector.announce("can't find goal");
            goalFailures++;
            if (SMART_TIME && goalFailures >= MAX_GOAL_FAILURES) {
                shootingGivenUp = true;   // stop wasting time on it (canShoot() is now false)
                brain = "couldn't find the HIVE twice - stopped trying to shoot";
            }
            // Full: stop here. Otherwise keep collecting and try again later.
            if (!collectBalls()) {
                goFinalTrip();   // simple autos: no HIVE found, so just go park
            } else {
                setState(ballsCollected >= MAX_BALLS ? State.FULL : State.SEEK);
            }
            lastBallSeenTimer.reset();
            return "can't find goal";
        }
        // Turn straight to where the HIVE was last seen, then look around from there
        if (SMART_SEARCH && odometry != null && !Double.isNaN(lastGoalHeadingDeg) && !goalMemoryUsed) {
            double err = AngleUnit.normalizeDegrees(lastGoalHeadingDeg - odometry.getHeadingDeg());   // + = LEFT
            if (Math.abs(err) > 15) {
                setDrivePower(0, Range.clip(-err * TURN_GAIN, -MAX_TURN, MAX_TURN));
                brain = String.format("turning to where the HIVE was (%.0f deg)", err);
                return "turning to where the HIVE was";
            }
            goalMemoryUsed = true;
            goalScanner.searchToward(err);
        }
        // Turn to look for the HIVE in steps (sharper camera images while still)
        setDrivePower(0, goalScanner.turnPower());
        return goalScanner.isTurning() ? "turning to look for the HIVE" : "looking for the HIVE";
    }

    /** Turn to face the goal and drive into the shoot zone. */
    private String lineUp() {
        CellTarget tag = getGoalTag();
        if (tag == null) {
            setDrivePower(0, 0);
            if (goalLastSeenTimer.milliseconds() > GOAL_LOST_MS) {
                setState(State.FIND_GOAL);
            }
            return "lost goal";
        }
        goalLastSeenTimer.reset();

        double bearingError = tag.bearingDeg - SHOOT_BEARING_DEG;   // + = goal is LEFT
        double rangeError   = tag.rangeIn - SHOOT_RANGE_IN;         // + = too far
        boolean inRange     = Math.abs(rangeError) <= SHOOT_RANGE_TOL_IN;
        if (SMART_SHOOTING) {
            // Flywheel speed follows the distance, so anywhere inside the RPM table is fine
            inRange = ShotPlanner.inShootingRange(tag.rangeIn);
            rangeError = tag.rangeIn < ShotPlanner.MIN_RANGE_IN ? tag.rangeIn - ShotPlanner.MIN_RANGE_IN
                    : tag.rangeIn > ShotPlanner.MAX_RANGE_IN ? tag.rangeIn - ShotPlanner.MAX_RANGE_IN : 0;
        }
        lastGoalRange = tag.rangeIn;
        if (odometry != null) lastGoalHeadingDeg = odometry.getHeadingDeg() + tag.bearingDeg;

        boolean aimed   = Math.abs(bearingError) <= SHOOT_BEARING_TOL_DEG;
        if (aimed && inRange) {
            setDrivePower(0, 0);
            if (SMART_SHOOTING) {
                // Shoot only once aimed AND holding still for a moment
                boolean still = odometry == null
                        || Math.hypot(odometry.getVelocityX(), odometry.getVelocityY()) < STEADY_SPEED_IN_PER_S;
                if (!steady || !still) {
                    steady = true;
                    steadyTimer.reset();
                }
                if (steadyTimer.milliseconds() < AIM_STEADY_MS) return "aimed - holding steady";
            }
            steady = false;
            startShooting();
            return "in shoot zone";
        }
        steady = false;

        double turn = aimed ? 0 : Range.clip(-bearingError * TURN_GAIN, -MAX_TURN, MAX_TURN);
        double forward = 0;
        if (!inRange && Math.abs(bearingError) <= DRIVE_MAX_BEARING) {
            forward = Math.signum(rangeError)
                    * Range.clip(Math.abs(rangeError) * DRIVE_GAIN, MIN_DRIVE, MAX_DRIVE);
        }
        setDrivePower(forward, turn);
        return String.format("lining up: %.0f in, %.1f deg", tag.rangeIn, tag.bearingDeg);
    }

    private void startShooting() {
        int balls = ballsCollected;
        if (SMART_TIME) {
            // Only shoot what fits before it has to head home
            int fit = ShotPlanner.ballsThatFit(balls, msLeft() - finalReturnMs(), FEED_INTAKE_MS, SHOT_DELAY_MS);
            if (fit <= 0) {
                brain = "no time left to shoot - going home";
                goFinalTrip();
                return;
            }
            balls = Math.min(fit, Math.max(balls, 1));
        }
        if (SMART_SHOOTING) shots.setRpm(ShotPlanner.rpmForRange(lastGoalRange));
        brain = String.format("shooting %d from %.0f in at %.0f RPM", balls, lastGoalRange, shots.getRpm());
        shots.start(balls);
        ballDetector.announce("shooting");
        setState(State.SHOOT);
    }

    /** Feed and fire left/right pairs (ShotSequencer), with a safety timeout. */
    private String shoot() {
        setDrivePower(0, 0);
        if (shots.isDone() || stateTimer.milliseconds() > SHOOT_TIMEOUT_MS) {
            finishShooting();
            return "done shooting";
        }
        return shots.update();
    }

    private void finishShooting() {
        setIntake(0);
        stopLaunchers();         // flywheel off until the next round
        ballsCollected = 0;
        lastBallSeenTimer.reset();
        setState(canReturnHome() ? State.RETURN_HOME : State.SEEK);
    }

    // -------------------------------------------------
    // Return to start (Pinpoint odometry)
    // -------------------------------------------------

    /**
     * Where to drive back to, relative to the start ({x forward, y left, heading deg}):
     * HOME_OFFSET_IN in front of the start, or the LOADING ZONE park spot on the final trip.
     */
    private double[] homeTarget() {
        if (finalReturn && PARK_IN_LOADING_ZONE) {
            double sh = Math.toRadians(START_FIELD_HEADING_DEG);
            double dx = PARK_FIELD_X - START_FIELD_X;
            double dy = PARK_FIELD_Y - START_FIELD_Y;
            return new double[] {
                    dx * Math.cos(sh) + dy * Math.sin(sh),
                    -dx * Math.sin(sh) + dy * Math.cos(sh),
                    PARK_FIELD_HEADING_DEG - START_FIELD_HEADING_DEG };
        }
        return new double[] { HOME_OFFSET_IN, 0, 0 };
    }

    /** Drive straight to homeTarget() and turn to its heading, using mecanum strafing. */
    private String returnHome() {
        double[] target = homeTarget();
        double x = odometry.getX();              // inches, + = forward of start
        double y = odometry.getY();              // inches, + = left of start
        double heading = odometry.getHeadingDeg();
        double distance = Math.hypot(target[0] - x, target[1] - y);
        double headingError = AngleUnit.normalizeDegrees(target[2] - heading);   // + = need to turn LEFT

        if ((distance <= HOME_POSITION_TOL_IN && Math.abs(headingError) <= HOME_HEADING_TOL_DEG)
                || stateTimer.milliseconds() > HOME_TIMEOUT_MS) {
            setDrivePower(0, 0, 0);
            ballDetector.announce(finalReturn && PARK_IN_LOADING_ZONE ? "parked" : "home");
            // Last trip: stop. Otherwise go collect the next round.
            lastBallSeenTimer.reset();
            setState(finalReturn ? State.DONE : State.SEEK);
            return "back at start";
        }

        // Field-frame error to start, rotated into the robot's frame
        double h = Math.toRadians(heading);
        double dx = target[0] - x, dy = target[1] - y;
        double forwardErr = dx * Math.cos(h) + dy * Math.sin(h);    // + = start is ahead
        double leftErr    = -dx * Math.sin(h) + dy * Math.cos(h);   // + = start is to the left

        double forward = 0, strafeRight = 0;
        if (distance > HOME_POSITION_TOL_IN) {
            double speed = Range.clip(distance * HOME_DRIVE_GAIN, HOME_MIN_POWER, HOME_MAX_POWER);
            forward     = speed * forwardErr / distance;
            strafeRight = speed * -leftErr / distance;
        }
        double turn = Math.abs(headingError) > HOME_HEADING_TOL_DEG
                ? Range.clip(-headingError * HOME_TURN_GAIN, -MAX_TURN, MAX_TURN)   // + turn = right
                : 0;
        setDrivePower(forward, strafeRight, turn);
        return String.format("returning to start: %.0f in, %.0f deg away", distance, headingError);
    }

    // -------------------------------------------------
    // Avoiding other robots
    // -------------------------------------------------

    /** After this loop's drive command: if something is in the way, switch to AVOID. */
    private String checkForRobots(String status) {
        if (!AVOID_ROBOTS || !hasDrive) return status;

        // Only while driving around (not while shooting, swallowing a ball, avoiding, or done)
        boolean driving = state == State.SEEK || state == State.FIND_GOAL
                || state == State.LINE_UP || state == State.RETURN_HOME;
        if (!driving) {
            stallTimer.reset();
            return status;
        }

        // A wall has been cutting our drive command for a while: back off and turn
        if (!wallBlocking) wallStuckTimer.reset();
        if (wallBlocking && wallStuckTimer.milliseconds() > WALL_STUCK_MS) {
            startAvoid(false, false, "wall");
            return "blocked by wall - backing off";
        }

        boolean robotAhead = lastForward > 0.05
                && (obstacleAhead() || (state != State.LINE_UP && robotSeenAhead()));
        boolean bumped = isStalled();
        if (robotAhead || bumped) {
            startAvoid(robotAhead, true, robotAhead ? "robot ahead" : "bumped");
            return robotAhead ? "robot ahead - stopping" : "bumped something - stopping";
        }
        return status;
    }

    /** Camera: a big alliance-color blob in our path (ignored while a HIVE CELL is visible). */
    private boolean robotSeenAhead() {
        if (!CAMERA_AVOID) return false;
        if (cellTarget(true) != null || cellTarget(false) != null) {
            return false;   // the big red/blue shape is probably the HIVE
        }
        for (BallDetector.RobotBlob robot : ballDetector.getRobotBlobs()) {
            // The HIVE CELLS hang ~44 in up, so they show up near the top of the image;
            // robots (and their signs) are lower down.
            if (robot.centerY < ROBOT_MIN_IMAGE_Y * BallDetector.IMAGE_HEIGHT) continue;
            if (robot.widthFraction >= ROBOT_CLOSE_WIDTH
                    && Math.abs(robot.bearing) <= ROBOT_AHEAD_DEG) {
                return true;
            }
        }
        return false;
    }

    private boolean obstacleAhead() {
        return frontDistance != null
                && frontDistance.getDistance(DistanceUnit.INCH) < OBSTACLE_STOP_IN;
    }

    /** Told to drive (not just turn) but the odometry says we aren't moving. */
    private boolean isStalled() {
        if (odometry == null) return false;
        boolean pushing = Math.hypot(lastForward, lastStrafe) >= STALL_MIN_POWER;
        double speed = Math.hypot(odometry.getVelocityX(), odometry.getVelocityY());
        if (pushing && speed < STALL_SPEED_IN_PER_S) {
            return stallTimer.milliseconds() > STALL_MS;
        }
        stallTimer.reset();
        return false;
    }

    /**
     * @param bySensor  true if a sensor/camera can tell us when the way is clear again
     * @param wait      wait for the obstacle to move first (robots move; walls don't)
     */
    private void startAvoid(boolean bySensor, boolean wait, String announcement) {
        // Where to pick up again afterwards
        if (state == State.RETURN_HOME) {
            resumeState = State.RETURN_HOME;
        } else if (state == State.FIND_GOAL || state == State.LINE_UP) {
            resumeState = State.FIND_GOAL;
        } else {
            resumeState = State.SEEK;
        }
        // Avoiding again and again here = crowded: move somewhere else afterwards
        long now = (long) clock.milliseconds();
        avoidTimes.add(now);
        while (!avoidTimes.isEmpty() && now - avoidTimes.peekFirst() > AVOID_LOOP_WINDOW_MS) avoidTimes.pollFirst();
        if (SMART_SEARCH && avoidTimes.size() >= AVOID_LOOP_COUNT && resumeState == State.SEEK) {
            relocateRequested = true;
            avoidTimes.clear();
            brain = "crowded here - will move somewhere else";
        }
        avoidBySensor = bySensor;
        avoidStep = wait ? AvoidStep.WAIT : AvoidStep.BACK_UP;
        avoidTimer.reset();
        wallStuckTimer.reset();
        setDrivePower(0, 0, 0);
        setIntake(0);
        ballDetector.announce(announcement);
        setState(State.AVOID);
    }

    /** Wait -> back up -> turn away -> resume. */
    private String avoid() {
        switch (avoidStep) {
            case WAIT:
                setDrivePower(0, 0, 0);
                // The other robot moved out of the way: carry on
                if (avoidBySensor && !obstacleAhead() && !robotSeenAhead()
                        && avoidTimer.milliseconds() > 300) {
                    resumeAfterAvoid();
                    return "path clear";
                }
                if (avoidTimer.milliseconds() > AVOID_WAIT_MS) {
                    avoidStep = AvoidStep.BACK_UP;
                    avoidTimer.reset();
                }
                return "waiting for robot to move";

            case BACK_UP:
                setDrivePower(-AVOID_BACK_POWER, 0, 0);
                if (avoidTimer.milliseconds() > AVOID_BACK_MS) {
                    avoidStep = AvoidStep.TURN;
                    avoidTimer.reset();
                }
                return "backing up";

            case TURN:
            default:
                setDrivePower(0, 0, avoidTurnDirection * AVOID_TURN_POWER);
                if (avoidTimer.milliseconds() > AVOID_TURN_MS) {
                    avoidTurnDirection = -avoidTurnDirection;   // try the other way next time
                    resumeAfterAvoid();
                }
                return "turning away";
        }
    }

    private void resumeAfterAvoid() {
        setDrivePower(0, 0, 0);
        stallTimer.reset();
        wallStuckTimer.reset();
        lastBallSeenTimer.reset();
        setState(resumeState);
    }

    // -------------------------------------------------
    // Time
    // -------------------------------------------------

    private double msLeft() {
        return autoLengthMs() - matchTimer.milliseconds();
    }

    /** Near the end, drop everything and head back to the start; at the end, stop. */
    private void checkTime() {
        if (state == State.DONE) return;

        if (msLeft() <= 0) {
            stopEverything();
            setState(State.DONE);
            return;
        }
        if (!finalReturn && canReturnHome() && msLeft() < finalReturnMs()) {
            finalReturn = true;
            stopEverything();
            ballDetector.announce("going home");
            setState(State.RETURN_HOME);
        }
    }

    /**
     * How long before the end to head home for the last time: with SMART_TIME, based on how far
     * home (or the park spot) is; otherwise FINAL_RETURN_MS.
     */
    private double finalReturnMs() {
        if (!SMART_TIME || odometry == null) return FINAL_RETURN_MS;
        boolean was = finalReturn;
        finalReturn = true;                  // where the LAST trip goes (park spot if set)
        double[] t = homeTarget();
        finalReturn = was;
        double distance = Math.hypot(t[0] - odometry.getX(), t[1] - odometry.getY());
        return Math.max(FINAL_RETURN_MIN_MS, ShotPlanner.driveTimeMs(distance));
    }

    /** Head home / park for the last time now (then stop). */
    private void goFinalTrip() {
        finalReturn = true;
        stopEverything();
        setState(canReturnHome() ? State.RETURN_HOME : State.DONE);
    }

    private void stopEverything() {
        setDrivePower(0, 0, 0);
        setIntake(0);
        stopLaunchers();
    }

    private void stopLaunchers() {
        if (hasLauncher) {
            leftLauncher.stop();
            rightLauncher.stop();
        }
    }

    // -------------------------------------------------
    // Field walls
    // -------------------------------------------------

    private boolean wallsActive() {
        return AVOID_WALLS && odometry != null && started;
    }

    /** Robot position in field coordinates (0-144 in), from the start pose + odometry. */
    /** Field position (0-144 in) of a point in the odometry frame. */
    private double fieldXOf(double ox, double oy) {
        double h = Math.toRadians(START_FIELD_HEADING_DEG);
        return START_FIELD_X + ox * Math.cos(h) - oy * Math.sin(h);
    }

    private double fieldYOf(double ox, double oy) {
        double h = Math.toRadians(START_FIELD_HEADING_DEG);
        return START_FIELD_Y + ox * Math.sin(h) + oy * Math.cos(h);
    }

    private static boolean nearWallAt(double fx, double fy, double margin) {
        return fx < margin || fx > FIELD_SIZE_IN - margin || fy < margin || fy > FIELD_SIZE_IN - margin;
    }

    private double fieldX() {
        double h = Math.toRadians(START_FIELD_HEADING_DEG);
        return START_FIELD_X + odometry.getX() * Math.cos(h) - odometry.getY() * Math.sin(h);
    }

    private double fieldY() {
        double h = Math.toRadians(START_FIELD_HEADING_DEG);
        return START_FIELD_Y + odometry.getX() * Math.sin(h) + odometry.getY() * Math.cos(h);
    }

    private double fieldHeadingRad() {
        return Math.toRadians(START_FIELD_HEADING_DEG + odometry.getHeadingDeg());
    }

    /**
     * Takes a robot-relative drive command and removes the part that would move the robot
     * toward a wall it is already close to. Returns {forward, strafeRight}.
     */
    private double[] keepAwayFromWalls(double forward, double strafeRight) {
        if (!wallsActive() || (forward == 0 && strafeRight == 0)) {
            return new double[] { forward, strafeRight };
        }
        double h = fieldHeadingRad();
        double left = -strafeRight;
        // Robot frame -> field frame
        double vx = forward * Math.cos(h) - left * Math.sin(h);
        double vy = forward * Math.sin(h) + left * Math.cos(h);

        double x = fieldX(), y = fieldY();
        if (x < WALL_MARGIN_IN && vx < 0) vx = 0;
        if (x > FIELD_SIZE_IN - WALL_MARGIN_IN && vx > 0) vx = 0;
        if (y < WALL_MARGIN_IN && vy < 0) vy = 0;
        if (y > FIELD_SIZE_IN - WALL_MARGIN_IN && vy > 0) vy = 0;

        // Field frame -> robot frame
        double newForward = vx * Math.cos(h) + vy * Math.sin(h);
        double newLeft    = -vx * Math.sin(h) + vy * Math.cos(h);
        return new double[] { newForward, -newLeft };
    }

    private boolean nearWall() {
        if (!wallsActive()) return false;
        double x = fieldX(), y = fieldY();
        return x < WALL_MARGIN_IN || x > FIELD_SIZE_IN - WALL_MARGIN_IN
                || y < WALL_MARGIN_IN || y > FIELD_SIZE_IN - WALL_MARGIN_IN;
    }

    private boolean canReturnHome() {
        return RETURN_TO_START && odometry != null && hasDrive;
    }

    /** Our alliance's visible HIVE CELL (AprilTag cluster), or null. */
    private CellTarget getGoalTag() {
        return cellTarget(redAlliance());
    }

    /** A HIVE CELL from whichever camera is in use. */
    private CellTarget cellTarget(boolean red) {
        return limelight != null ? limelight.getCellTarget(red) : aprilTagWebcam.getCellTarget(red);
    }

    /**
     * Update the camera(s). The Limelight runs one pipeline at a time: the HIVE (AprilTags)
     * while finding / lining up on the goal, a ball color pipeline the rest of the time.
     */
    private void updateVision() {
        if (limelight != null) {
            boolean lookingForHive = limelightTagsOnly
                    || state == State.FIND_GOAL || state == State.LINE_UP;
            limelight.setMode(lookingForHive ? LimelightVision.Mode.HIVE : LimelightVision.Mode.BALLS,
                    LIMELIGHT_BALL_COLOR);
            limelight.update();
        } else {
            aprilTagWebcam.update();
        }
        ballDetector.update();
    }

    private boolean canShoot() {
        return SHOOT_BALLS && canPickUp() && hasLauncher && !shootingGivenUp;
    }

    private boolean canPickUp() {
        return PICK_UP_BALL && DRIVE_TO_BALL && TURN_TO_BALL && hasDrive && hasIntake;
    }

    private void setState(State newState) {
        if (newState == State.FIND_GOAL && state != State.FIND_GOAL) {
            goalMemoryUsed = false;   // a new HIVE search: try where it was last seen first
            goalScanner.reset();
        }
        state = newState;
        stateTimer.reset();
    }

    private void report() {
        if (!brain.isEmpty()) telemetry.addData("Brain", brain);
        if (started) {
            telemetry.addData("Time left", "%.1f s", Math.max(0, msLeft()) / 1000.0);
        }
        telemetry.addData("Alliance", redAlliance() ? "RED" : "BLUE");
        if (limelight != null) limelight.displayTelemetry();
        telemetry.addData("Sound", ballDetector.isSoundEnabled() ? "ON" : "OFF");
        telemetry.addData("Drive", hasDrive ? "connected" : "none (camera-only)");
        if (hasIntake) intake.addTelemetry(); else telemetry.addData("Intake", "none");
        telemetry.addData("Launcher", hasLauncher ? "connected" : "none");
        if (frontDistance != null) {
            telemetry.addData("Front distance", "%.1f in", frontDistance.getDistance(DistanceUnit.INCH));
        }
        if (odometry != null) {
            telemetry.addData("Position from start", "x %.1f  y %.1f in  heading %.0f deg",
                    odometry.getX(), odometry.getY(), odometry.getHeadingDeg());
            if (AVOID_WALLS && started) {
                telemetry.addData("Field position", "x %.1f  y %.1f in%s",
                        fieldX(), fieldY(), nearWall() ? "  (NEAR WALL)" : "");
            }
        } else {
            telemetry.addData("Odometry", "none (won't return to start)");
        }
        if (canPickUp()) {
            telemetry.addData("Balls collected", "%d / %d", ballsCollected, MAX_BALLS);
        }
        ballDetector.displayTelemetry();
        telemetry.update();
    }

    /** Forward + turn, no strafe. turn > 0 = turn right. */
    private void setDrivePower(double forward, double turn) {
        setDrivePower(forward, 0, turn);
    }

    /** Mecanum drive, same mixing as the teleop: strafeRight > 0 = right, turn > 0 = turn right. */
    private void setDrivePower(double forward, double strafeRight, double turn) {
        // Remove any motion toward a nearby field wall
        double[] safe = keepAwayFromWalls(forward, strafeRight);
        double requested = Math.hypot(forward, strafeRight);
        wallBlocking = requested > 0.05 && Math.hypot(safe[0], safe[1]) < 0.5 * requested;
        forward = safe[0];
        strafeRight = safe[1];

        lastForward = forward;
        lastStrafe = strafeRight;
        double max = Math.max(Math.abs(forward) + Math.abs(strafeRight) + Math.abs(turn), 1);
        if (frontLeft  != null) frontLeft.setPower((forward + strafeRight + turn) / max);
        if (backLeft   != null) backLeft.setPower((forward - strafeRight + turn) / max);
        if (frontRight != null) frontRight.setPower((forward - strafeRight - turn) / max);
        if (backRight  != null) backRight.setPower((forward + strafeRight - turn) / max);
    }

    private void setIntake(double power) {
        if (hasIntake) intake.run(power);
    }

    @Override
    public void stop() {
        setDrivePower(0, 0);
        setIntake(0);
        stopLaunchers();
        if (limelight != null) {
            limelight.stop();
        } else {
            aprilTagWebcam.stop();   // closes the shared camera
        }
        ballDetector.stop();         // closes the webcam if the ball detector opened it
    }

    private DcMotor getMotor(String name) {
        try {
            return hardwareMap.get(DcMotor.class, name);
        } catch (Exception e) {
            return null;
        }
    }
}
