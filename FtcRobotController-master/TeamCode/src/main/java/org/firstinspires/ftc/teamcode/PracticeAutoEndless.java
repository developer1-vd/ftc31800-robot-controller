package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Endless practice / demo autonomous: collects balls with the camera and shoots them into the
 * HIVE over and over, with no time limit, until you press Stop.
 *
 * When selecting it on the Driver Station, turn OFF the 30-second autonomous timer, or the
 * Driver Station stops it after 30 s. Uses BallSpeakerAuto's settings (alliance = RED_ALLIANCE
 * there). Practice only.
 */
@Autonomous(name = "Practice Auto (endless)", group = "Practice")
public class PracticeAutoEndless extends BallSpeakerAuto {

    @Override
    protected double autoLengthMs() {
        return Double.POSITIVE_INFINITY;   // never runs out: no final trip, keeps going
    }
}
