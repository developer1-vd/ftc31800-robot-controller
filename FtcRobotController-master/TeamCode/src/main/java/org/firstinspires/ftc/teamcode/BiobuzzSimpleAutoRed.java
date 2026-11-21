package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * BIOBUZZ simple, safe autonomous for the RED alliance: no ball hunting.
 *
 *  1. drive off the perimeter wall (LEAVE, 3 pts),
 *  2. find our RED HIVE CELL by its AprilTag and LAUNCH the 4 pre-loaded POLLEN,
 *  3. drive back off the wall, or PARK in the LOADING ZONE if set up in BallSpeakerAuto (5 pts),
 *     and stop.
 * If the HIVE isn't found (or there is no launcher), it skips shooting and parks. It only needs
 * the HIVE camera (Limelight or webcam), not ball detection.
 */
@Autonomous(name = "BIOBUZZ Simple Auto RED", group = "BIOBUZZ", preselectTeleOp = "TeamBotTeleop_latest")
public class BiobuzzSimpleAutoRed extends BallSpeakerAuto {

    @Override
    protected boolean redAlliance() {
        return true;
    }

    @Override
    protected int preloadedBalls() {
        return 4;
    }

    @Override
    protected boolean leaveWallFirst() {
        return true;
    }

    @Override
    protected boolean collectBalls() {
        return false;
    }
}
