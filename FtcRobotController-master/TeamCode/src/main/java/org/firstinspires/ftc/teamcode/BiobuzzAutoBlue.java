package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * BIOBUZZ (2026-27) competition autonomous for the BLUE alliance.
 *
 * Select it with the Driver Station 30-second timer enabled. After Play it:
 *  1. drives off the perimeter wall (LEAVE, 3 pts),
 *  2. finds our BLUE HIVE CELL by its AprilTag cluster, lines up and LAUNCHES the
 *     4 pre-loaded POLLEN (enough in the CELL tips the HIVE, 20 pts),
 *  3. collects more POLLEN / BLUE NECTAR with the camera and LAUNCHES again while time allows,
 *  4. with 6 s left drives back off the wall, or PARKs in the LOADING ZONE if set up (5 pts).
 *
 * All the behavior and tuning lives in BallSpeakerAuto; this class only picks the alliance,
 * the preloads, and leaving the wall first.
 */
@Autonomous(name = "BIOBUZZ Auto BLUE", group = "BIOBUZZ", preselectTeleOp = "TeamBotTeleop_latest")
public class BiobuzzAutoBlue extends BallSpeakerAuto {

    @Override
    protected boolean redAlliance() {
        return false;
    }

    @Override
    protected int preloadedBalls() {
        return 4;   // every BIOBUZZ robot starts the match with 4 POLLEN
    }

    @Override
    protected boolean leaveWallFirst() {
        return true;
    }
}
