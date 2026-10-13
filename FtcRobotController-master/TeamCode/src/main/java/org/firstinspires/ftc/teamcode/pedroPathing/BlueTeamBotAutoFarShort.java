package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.BallCollector;
import org.firstinspires.ftc.teamcode.mechanisms.FeederLauncher;

@Autonomous(name ="FarTeamBotAutoFarShort", group = "Examples")
public class BlueTeamBotAutoFarShort extends OpMode {

    private Follower follower;
    private ElapsedTime pathTimer, actionTimer, opmodeTimer, intakeTimer;

    private ElapsedTime shootDelayTimer; // Add this at the top along with other timers
    final double SHOOT_DELAY_MILLISECONDS = 2000; // delay between left and right shots
    private DcMotor intake = null;
    private boolean hasIntake = false;
    private Servo arm = null;

    private enum PATHSTATE {
        PRELOAD_TO_SCORE,
        SHOOT,
        SHOOT_UPDATE,
        SCORE_TO_LEAVE,
        END,
        COLLECT_BALL,
        RETURN_TO_SCORE,
        SHOOT_COLLECTED,
        LEAVE_AGAIN,
        EXIT
    }

    private enum SHOOTSTATE {

        SHOOT_1,
        SHOOT_1_LEFT_UPDATE,

        SHOOT_1_DELAY,
        SHOOT_1_RIGHT_UPDATE,

        INTAKE,

        SHOOT_2,
        SHOOT_2_LEFT_UPDATE,
        SHOOT_2_DELAY,
        SHOOT_2_RIGHT_UPDATE,

        SHOOT_END
    }


    private SHOOTSTATE shootstate;
    private PATHSTATE pathState;

    private final Pose scorePose = new Pose(62.824, 12.706, Math.toRadians(295));
    private final Pose endPose = new Pose(40.000, 13.882, Math.toRadians(295));

    private Path leavelaunchline;

    private FeederLauncher leftFeederLauncher = new FeederLauncher();
    private FeederLauncher rightFeederLauncher = new FeederLauncher();

    // After the path, use the camera to find and pick up a ball with the time left.
    // Set to false to skip ball pickup.
    private static final boolean COLLECT_BALL_AFTER_PATH = true;
    private static final double AUTO_LENGTH_MS = 30000;
    private final BallCollector ballCollector = new BallCollector();

    // After picking up a ball: drive back to scorePose, shoot it, then leave again.
    // Set to false to just stop after pickup.
    private static final boolean SHOOT_COLLECTED_BALL = true;
    // Time to save for driving back, shooting, and leaving; pickup gives up earlier than this.
    private static final double RETURN_AND_SHOOT_MS = 10000;
    // If time gets this low while returning/shooting, give up and leave so we still get LEAVE.
    private static final double LEAVE_TIME_MS = 3000;
    private boolean shootingCollectedBall = false;
    // Driver Station says "red ball", "got ball", etc. Set to false for silence.
    private static final boolean SOUND_ENABLED = true;

    // ------------------- PATH BUILDING -------------------
    public void buildPaths() {
        leavelaunchline = new Path(new BezierLine(scorePose, endPose));
        leavelaunchline.setLinearHeadingInterpolation(scorePose.getHeading(), endPose.getHeading());
    }

    // ------------------- PATH STATE MACHINE -------------------
    public void autonomousPathUpdate() {
        switch (pathState) {
            case PRELOAD_TO_SCORE:
                actionTimer.reset();
                setPathState(PATHSTATE.SHOOT_UPDATE);
                shootstate = SHOOTSTATE.SHOOT_1;
                break;

            case SHOOT_UPDATE:
                shootUpdate();
                break;

            case SCORE_TO_LEAVE:
                if (!follower.isBusy()) {
                    follower.followPath(leavelaunchline, true);
                    setPathState(PATHSTATE.END);
                }
                break;

            case END:
                if (!follower.isBusy()) {
                    if (COLLECT_BALL_AFTER_PATH) {
                        ballCollector.start();
                        setPathState(PATHSTATE.COLLECT_BALL);
                    } else {
                        setPathState(PATHSTATE.EXIT);
                    }
                }
                break;

            case COLLECT_BALL:
                ballCollector.update(msLeftInAuto());
                if (ballCollector.isDone()) {
                    if (SHOOT_COLLECTED_BALL && ballCollector.capturedBall()) {
                        follower.followPath(pathTo(scorePose), true);
                        setPathState(PATHSTATE.RETURN_TO_SCORE);
                    } else {
                        setPathState(PATHSTATE.EXIT);
                    }
                }
                break;

            case RETURN_TO_SCORE:
                if (msLeftInAuto() < LEAVE_TIME_MS) {
                    setPathState(PATHSTATE.LEAVE_AGAIN);
                } else if (!follower.isBusy()) {
                    // Same feed-then-shoot steps as the second preload volley (INTAKE -> SHOOT_2)
                    shootingCollectedBall = true;
                    actionTimer.reset();
                    intakeTimer.reset();
                    shootstate = SHOOTSTATE.INTAKE;
                    setPathState(PATHSTATE.SHOOT_COLLECTED);
                }
                break;

            case SHOOT_COLLECTED:
                if (msLeftInAuto() < LEAVE_TIME_MS) {
                    if (hasIntake) intake.setPower(0.0);
                    setPathState(PATHSTATE.LEAVE_AGAIN);
                } else {
                    shootUpdate();   // SHOOT_END moves on to LEAVE_AGAIN
                }
                break;

            case LEAVE_AGAIN:
                // Drive back off the launch line so LEAVE still counts
                follower.followPath(pathTo(endPose), true);
                setPathState(PATHSTATE.EXIT);
                break;
        }
    }

    public void setPathState(PATHSTATE state) {
        pathState = state;
        pathTimer.reset();
    }

    private double msLeftInAuto() {
        return AUTO_LENGTH_MS - opmodeTimer.milliseconds();
    }

    /** Straight path from where the robot is now to the target pose. */
    private Path pathTo(Pose target) {
        Pose current = follower.getPose();
        Path path = new Path(new BezierLine(current, target));
        path.setLinearHeadingInterpolation(current.getHeading(), target.getHeading());
        return path;
    }

    // ------------------- SHOOTING -------------------
    public void shootLeftArtifacts() {
        leftFeederLauncher.launch(3150, 0, true, true);
    }

    public void shootRightArtifacts() {
        rightFeederLauncher.launch(3150, 0, true, true);
    }

    public void shootUpdate() {
        switch (shootstate) {
            case SHOOT_1:
                // Start left shoot first
                shootLeftArtifacts();
                shootDelayTimer = new ElapsedTime(); // initialize timer for delay
                shootstate = SHOOTSTATE.SHOOT_1_LEFT_UPDATE;
                break;

            case SHOOT_1_LEFT_UPDATE:
                // Wait until left has launched
                if (leftFeederLauncher.getStatus() == FeederLauncher.LaunchState.LAUNCHED) {
                    shootDelayTimer.reset(); // start counting for right shoot delay
                    shootstate = SHOOTSTATE.SHOOT_1_DELAY;
                }
                break;

            case SHOOT_1_DELAY:
                if (shootDelayTimer.milliseconds() >= SHOOT_DELAY_MILLISECONDS) {
                    shootRightArtifacts();
                    shootDelayTimer.reset();

                    shootstate = SHOOTSTATE.SHOOT_1_RIGHT_UPDATE;
                }
                break;

            case SHOOT_1_RIGHT_UPDATE:
                // Wait until delay has passed


                if (rightFeederLauncher.getStatus() == FeederLauncher.LaunchState.LAUNCHED) {
                    shootstate = SHOOTSTATE.INTAKE;
                    intakeTimer.reset();
                }
                break;

            case INTAKE:
                if (intakeTimer.milliseconds() < 1000) {
                    if (hasIntake) {
                        intake.setPower(-1.0);
                    }
                } else {
                    if (hasIntake) {
                        intake.setPower(0.0);
                    }
                    shootstate = SHOOTSTATE.SHOOT_2;
                }
                break;

            case SHOOT_2:
                shootLeftArtifacts();
                shootDelayTimer.reset();
                shootstate = SHOOTSTATE.SHOOT_2_LEFT_UPDATE;
                break;

            case SHOOT_2_LEFT_UPDATE:
                if (leftFeederLauncher.getStatus() == FeederLauncher.LaunchState.LAUNCHED) {
                    shootDelayTimer.reset();
                    shootstate = SHOOTSTATE.SHOOT_2_DELAY;
                }
                break;

            case SHOOT_2_DELAY:
                if (shootDelayTimer.milliseconds() >= SHOOT_DELAY_MILLISECONDS) {
                    shootRightArtifacts();
                    shootDelayTimer.reset();

                    shootstate = SHOOTSTATE.SHOOT_2_RIGHT_UPDATE;
                }

                break;
            case SHOOT_2_RIGHT_UPDATE:

                if (rightFeederLauncher.getStatus() == FeederLauncher.LaunchState.LAUNCHED) {
                    shootstate = SHOOTSTATE.SHOOT_END;
                }
                break;

            case SHOOT_END:
                if (shootingCollectedBall) {
                    setPathState(PATHSTATE.LEAVE_AGAIN);
                } else {
                    setPathState(PATHSTATE.SCORE_TO_LEAVE);
                }
                break;
        }

        // Update launchers
        leftFeederLauncher.update();
        rightFeederLauncher.update();

        // Recovery timeout
        if (actionTimer.milliseconds() >= 23000) {
            setPathState(PATHSTATE.SCORE_TO_LEAVE);
            shootstate = SHOOTSTATE.SHOOT_END;
        }

        telemetry.addData("Shoot State: ", shootstate);
    }


    // ------------------- OP MODE LOOP -------------------
    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("LeftFeederLauncher state", leftFeederLauncher.getStatus());
        telemetry.addData("RightFeederLauncher state", rightFeederLauncher.getStatus());
        leftFeederLauncher.reportHardwareStatus();
        rightFeederLauncher.reportHardwareStatus();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    // ------------------- INIT -------------------
    @Override
    public void init() {
        pathTimer = new ElapsedTime();
        opmodeTimer = new ElapsedTime();
        actionTimer = new ElapsedTime();
        intakeTimer = new ElapsedTime();
        opmodeTimer.reset();

        shootstate = SHOOTSTATE.SHOOT_1;

        leftFeederLauncher.init(hardwareMap, telemetry, "launcher", "feederServoLeft");
        rightFeederLauncher.init(hardwareMap, telemetry, "launcher", "feederServoRight");

        intake = hardwareMap.get(DcMotor.class, "intake");
        hasIntake = (intake != null);
        if (hasIntake) intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm = hardwareMap.get(Servo.class, "armServo");

        follower = Constants.createFollower(hardwareMap);
        ballCollector.init(hardwareMap, telemetry, follower, intake);
        if (SHOOT_COLLECTED_BALL) ballCollector.setStopBeforeEndMs(RETURN_AND_SHOOT_MS);
        ballCollector.setSoundEnabled(SOUND_ENABLED);
        buildPaths();
        follower.setStartingPose(scorePose);

        telemetry.addData("intake", hasIntake ? "OK" : "MISSING");
        arm.scaleRange(0.2, 0.8);
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        opmodeTimer.reset();
        setPathState(PATHSTATE.PRELOAD_TO_SCORE);

        leftFeederLauncher.launch(0, 0, false, false);
        rightFeederLauncher.launch(0, 0, false, false);

        arm.setPosition(0.5);
    }

    @Override
    public void stop() {
        ballCollector.stop();
        leftFeederLauncher.stop();
        rightFeederLauncher.stop();
    }
}
