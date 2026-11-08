package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * BIOBUZZ path auto, RED alliance (see BiobuzzPathAuto for what it does).
 *
 * SET THE FIELD POSES BELOW, then set POSES_SET = true. Until then it only drives forward
 * a little (LEAVE). Coordinates: Pedro field inches (0-144) and heading in degrees, the same
 * system as last season's Pedro autos. Measure them on your field (or read them from the
 * Pedro visualizer), for the RED starting spot your team uses:
 *   START   where the robot sits at the start (against the wall, touching 4 POLLEN)
 *   SHOOT   where it shoots into the RED HIVE CELL (in your shoot window, facing the HIVE)
 *   COLLECT start/end: a straight sweep with the intake on, e.g. along the RED GARDEN
 *   PARK    partly inside the RED LOADING ZONE, not touching the wall
 */
@Autonomous(name = "BIOBUZZ Path Auto RED", group = "BIOBUZZ", preselectTeleOp = "TeamBotTeleop_latest")
public class BiobuzzPathAutoRed extends BiobuzzPathAuto {

    private static final boolean POSES_SET = false;   // <- true once the poses below are measured

    private static final Pose START         = pose(0, 0, 0);
    private static final Pose SHOOT         = pose(0, 0, 0);
    private static final Pose COLLECT_START = pose(0, 0, 0);
    private static final Pose COLLECT_END   = pose(0, 0, 0);
    private static final Pose PARK          = pose(0, 0, 0);

    @Override protected boolean redAlliance()   { return true; }
    @Override protected boolean posesSet()      { return POSES_SET; }
    @Override protected Pose startPose()        { return START; }
    @Override protected Pose shootPose()        { return SHOOT; }
    @Override protected Pose collectStartPose() { return COLLECT_START; }
    @Override protected Pose collectEndPose()   { return COLLECT_END; }
    @Override protected Pose parkPose()         { return PARK; }
}
