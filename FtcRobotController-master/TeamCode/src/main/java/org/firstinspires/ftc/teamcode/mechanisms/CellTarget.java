package org.firstinspires.ftc.teamcode.mechanisms;

/**
 * Where our HIVE CELL is, relative to the camera, from whichever camera is in use
 * (webcam AprilTag cluster or Limelight fiducials).
 */
public class CellTarget {
    public final double rangeIn;     // straight-line distance to the CELL target, inches
    public final double bearingDeg;  // + = target is LEFT of center (same as AprilTag ftcPose.bearing)

    public CellTarget(double rangeIn, double bearingDeg) {
        this.rangeIn = rangeIn;
        this.bearingDeg = bearingDeg;
    }
}
