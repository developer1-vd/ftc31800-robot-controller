
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.ConfigurationType;
//import com.qualcomm.robotcore.hardware.HardwareDevice;

import com.qualcomm.robotcore.util.Range;

// DECODE (2025-26) autonomous: last season's field and goal. Disabled for BIOBUZZ;
// use BiobuzzAutoRed / BiobuzzAutoBlue instead.
@Autonomous(name="TeamBotAuto", group="Drive")
@Disabled
public class TeamBotAuto extends OpMode {

    // Drivetrain motors (312 rpm goBILDA)
    private DcMotor frontLeft  = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft   = null;
    private DcMotor backRight  = null;

    // Intake (312 rpm goBILDA)
    private DcMotor intake = null;

    // Flywheel (high-speed shooter)
    private DcMotorEx flywheel = null;

    // Flywheel target RPM (we’ll let you tune live)
    private double targetFlywheelRPM = 1500; // start around mid-speed; tune with dpad
    private double MinFlywheelRPM = targetFlywheelRPM * 0.80; // start around mid-speed; tune with dpad

    // Convenience: did each motor initialize?
    private boolean hasFrontLeft  = false;
    private boolean hasFrontRight = false;
    private boolean hasBackLeft   = false;
    private boolean hasBackRight  = false;
    private boolean hasIntake     = false;
    private boolean hasFlywheel   = false;

    static final double AUTOPOWER   = 0.1;

    double flPower = AUTOPOWER;
    double blPower = AUTOPOWER;
    double frPower = AUTOPOWER;
    double brPower = AUTOPOWER;

    double intakePower = 0.0;

    ElapsedTime DriveTimer = new ElapsedTime();
    ElapsedTime FeedTimer = new ElapsedTime();

    double ticksPerRev = 28.0; // *** TODO: PUT YOUR FLYWHEEL ENCODER TICKS/REV HERE ***
    double targetTicksPerSec = rpmToTicksPerSec(targetFlywheelRPM, ticksPerRev);
    double targetMinTicksPerSec = rpmToTicksPerSec(MinFlywheelRPM, ticksPerRev);

    final double DRIVETIME = 2.0;

    private enum AutonomousState {
        LAUNCHING,
        LAUNCH,
        DRIVING,
        DRIVE_TO_POSITION,
        STOP,
    }
    private AutonomousState autoState;

    @Override
    public void init() {
        /*
         * Try to grab each motor safely. If it's missing from the config,
         * we catch the exception and record a flag so we don't try to drive it later.
         */
        frontLeft  = getMotor("left_front_drive");
        frontRight = getMotor("right_front_drive");
        backLeft   = getMotor("left_back_drive");
        backRight  = getMotor("right_back_drive");
        intake     = getMotor("intake");
        flywheel   = getMotorEx("launcher"); // needs DcMotorEx for velocity

        hasFrontLeft  = (frontLeft  != null);
        hasFrontRight = (frontRight != null);
        hasBackLeft   = (backLeft   != null);
        hasBackRight  = (backRight  != null);
        hasIntake     = (intake     != null);
        hasFlywheel   = (flywheel   != null);

        // Reverse directions so forward stick actually drives forward.
        // Common mecanum setup: left side reversed, right side normal.
        if (hasFrontLeft)  frontLeft.setDirection(DcMotor.Direction.REVERSE);
        if (hasBackLeft)   backLeft.setDirection(DcMotor.Direction.REVERSE);
        if (hasFrontRight) frontRight.setDirection(DcMotor.Direction.FORWARD);
        if (hasBackRight)  backRight.setDirection(DcMotor.Direction.FORWARD);

        // Zero power behavior so robot stops instead of coasting
        if (hasFrontLeft)  frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (hasFrontRight) frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (hasBackLeft)   backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (hasBackRight)  backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (hasIntake) {
            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        if (hasFlywheel) {
            // Shooter usually FLOATS on zero, so wheel can spin down naturally.
            flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            // Optional: run without encoder for now, we’ll manually set velocity later.
            // We’ll switch to RUN_USING_ENCODER so .setVelocity() works in loop()
            flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            flywheel.setDirection(DcMotor.Direction.REVERSE);




        }

        autoState = AutonomousState.LAUNCHING;

        telemetry.addLine("Init complete. Check missing hardware below.");
        reportHardwareStatus();
    }

    @Override
    public void loop() {


        switch (autoState) {


            case LAUNCHING:
            {
                // Spin up the flywheel to target RPM
                if (hasFlywheel) {
                    flywheel.setVelocity(targetTicksPerSec);
                }

                autoState = AutonomousState.LAUNCH;
                break;
            }



            case LAUNCH:
            {

                if (hasFlywheel) {
                    if (flywheel.getVelocity() < targetMinTicksPerSec) {
                        // Not up to speed yet, wait
                        autoState = AutonomousState.LAUNCH;
                        break;
                    }
                }
                // Activate intake to feed rings into flywheel
                if (hasIntake) {
                    intake.setPower(1.0); // Full power intake
                }




                autoState = AutonomousState.DRIVING;
                break;
            }

            case DRIVING:{

                // Stop intake and flywheel
                if (hasIntake) {
                    intake.setPower(0);
                }
                if (hasFlywheel) {
                    flywheel.setPower(0);
                }

                flPower = AUTOPOWER;
                blPower = AUTOPOWER;
                frPower = AUTOPOWER;
                brPower = AUTOPOWER;

                // Send power only if that motor exists
                if (hasFrontLeft)  frontLeft.setPower(flPower);
                if (hasBackLeft)   backLeft.setPower(blPower);
                if (hasFrontRight) frontRight.setPower(frPower);
                if (hasBackRight)  backRight.setPower(brPower);
                DriveTimer.reset();


                autoState = AutonomousState.DRIVE_TO_POSITION;
                break;
            }


            case DRIVE_TO_POSITION:
            {
                // Continue driving until a certain condition is met.
                // For example, drive for 2 seconds or until a sensor is triggered.
                // Here, we'll just simulate a wait using a simple counter or timer.
                // In a real robot, you'd use elapsed time or encoder counts.

                // For demonstration, let's assume we drive for 2 seconds.
                // You would need to implement a timer to track this.

                if (DriveTimer.seconds() < DRIVETIME) {
                    // Keep driving
                    autoState = AutonomousState.DRIVE_TO_POSITION;
                    if (hasFrontLeft)  frontLeft.setPower(flPower);
                    if (hasBackLeft)   backLeft.setPower(blPower);
                    if (hasFrontRight) frontRight.setPower(frPower);
                    if (hasBackRight)  backRight.setPower(brPower);
                    break;
                }




                // After driving, stop the robot
                if (hasFrontLeft)  frontLeft.setPower(0);
                if (hasBackLeft)   backLeft.setPower(0);
                if (hasFrontRight) frontRight.setPower(0);
                if (hasBackRight)  backRight.setPower(0);

                autoState = AutonomousState.STOP;
                break;
            }


            case STOP:
            {
                // Ensure all motors are stopped
                if (hasFrontLeft)  frontLeft.setPower(0);
                if (hasBackLeft)   backLeft.setPower(0);
                if (hasFrontRight) frontRight.setPower(0);
                if (hasBackRight)  backRight.setPower(0);
                if (hasIntake)     intake.setPower(0);
                if (hasFlywheel)   flywheel.setPower(0);

                DriveTimer.reset();

                // Remain in STOP state
                break;
            }
        }

        //----------------------------------
        // 2. INTAKE CONTROL (gamepad1 A/B/X)
        //----------------------------------
        //  A = intake forward
        //  B = intake reverse
        //  X = stop intake
       /* double intakePower = 0.0;
        if (gamepad1.a) {
            intakePower = 1.0;    // full forward
        } else if (gamepad1.b) {
            intakePower = -1.0;   // reverse / spit out
        } else if (gamepad1.x) {
            intakePower = 0.0;
        }
        if (hasIntake) {
            intake.setPower(intakePower);
        }

        //----------------------------------
        // 3. FLYWHEEL CONTROL (gamepad1)
        //----------------------------------
        //  Right bumper  = enable flywheel at target RPM
        //  Left bumper   = stop flywheel
        //  Dpad up/down  = tune target RPM in steps
        //
        // We'll convert RPM -> ticksPerSecond and call setVelocity().
        //
        // NOTE: You *must* update "TICKS_PER_REV" below to match your flywheel motor encoder.
        // goBILDA 1172rpm motors often ~28 ticks per rev. 312rpm gobilda motor ~537.6 tpr.
        // Your ~6000rpm motor may be a coreless or planetary w/ custom encoder or no encoder.
        // If it has no encoder, setVelocity won't really work; you'd fall back to simple setPower().

        // Live adjust RPM setpoint:
        if (gamepad1.dpad_up) {
            targetFlywheelRPM += 10; // +100 RPM
        } else if (gamepad1.dpad_down) {
            targetFlywheelRPM -= 10; // -100 RPM
        }
        targetFlywheelRPM = Range.clip(targetFlywheelRPM, 500, 6000); // keep sane

        // Spin logic L1: Left bumper, R1: Right bumper
        boolean flywheelRequestedOn = gamepad1.right_bumper;
        boolean flywheelRequestedOff = gamepad1.left_bumper;

        if (hasFlywheel) {
            if (flywheelRequestedOn) {
                // Convert RPM to ticks/sec and set velocity
                double ticksPerRev = 28.0; // *** TODO: PUT YOUR FLYWHEEL ENCODER TICKS/REV HERE ***
                double targetTicksPerSec = rpmToTicksPerSec(targetFlywheelRPM, ticksPerRev);

                // Ask motor to run at that velocity.
                // The SDK will do PIDF internally in RUN_USING_ENCODER mode.
                flywheel.setVelocity(targetTicksPerSec);
            } else if (flywheelRequestedOff) {
                flywheel.setPower(0);
            }
        }
     */
        //----------------------------------
        // 4. TELEMETRY / DRIVER FEEDBACK
        //----------------------------------
        telemetry.addLine("== DRIVE ==");
        telemetry.addData("fl/br/bl/fr", "%.2f %.2f %.2f %.2f",
                flPower, brPower, blPower, frPower);

        telemetry.addLine("== INTAKE ==");
        telemetry.addData("intakePower", "%.2f", intakePower);
        telemetry.addData("intakePresent", hasIntake);

        telemetry.addLine("== FLYWHEEL ==");
        telemetry.addData("targetRPM", "%.0f", targetFlywheelRPM);
        telemetry.addData("flywheelPresent", hasFlywheel);

        if (hasFlywheel) {
            telemetry.addData("right_bumper", "spin @ target RPM");
            telemetry.addData("left_bumper",  "stop");
            telemetry.addData("dpad up/down", "RPM +/- 100");

            // If encoder exists, show current velocity estimate
            double currentVel = flywheel.getVelocity(); // ticks/sec
            telemetry.addData("flywheelVel(ticks/s)", "%.1f", currentVel);

            double ticksPerRev = 28.0; // same number we used above
            double currentRPM = ticksPerSecToRPM(currentVel, ticksPerRev);
            telemetry.addData("flywheelVel(RPM est)", "%.0f", currentRPM);
        }

        // Hardware health summary so drivers instantly see missing motors
        reportHardwareStatus();

        telemetry.update();
    }

    // -------------------------------------------------
    // Helper: safe motor fetch (DcMotor)
    // -------------------------------------------------
    private DcMotor getMotor(String name) {
        try {
            return hardwareMap.get(DcMotor.class, name);
        } catch (Exception e) {
            // Could be not found in config or wrong type
            return null;
        }
    }

    // -------------------------------------------------
    // Helper: safe motor fetch as DcMotorEx
    // -------------------------------------------------
    private DcMotorEx getMotorEx(String name) {
        try {
            return hardwareMap.get(DcMotorEx.class, name);
        } catch (Exception e) {
            return null;
        }
    }

    // -------------------------------------------------
    // Helper: report which motors are missing
    // -------------------------------------------------
    private void reportHardwareStatus() {
        telemetry.addLine("== HARDWARE STATUS ==");

        telemetry.addData("frontLeft",  hasFrontLeft  ? "OK" : "MISSING");
        telemetry.addData("frontRight", hasFrontRight ? "OK" : "MISSING");
        telemetry.addData("backLeft",   hasBackLeft   ? "OK" : "MISSING");
        telemetry.addData("backRight",  hasBackRight  ? "OK" : "MISSING");

        telemetry.addData("intake",     hasIntake     ? "OK" : "MISSING");

        telemetry.addData("flywheel",   hasFlywheel   ? "OK" : "MISSING");
        telemetry.addData("AutonomousState: ",autoState);
    }

    // -------------------------------------------------
    // Math helpers for flywheel RPM <-> ticks/sec
    // -------------------------------------------------
    private double rpmToTicksPerSec(double rpm, double ticksPerRev) {
        // rpm * (ticks/rev) / 60 sec/min
        // Step by step to avoid mistakes:
        // 1. rpm * ticksPerRev = ticks per minute
        double ticksPerMinute = rpm * ticksPerRev;
        // 2. ticks per minute / 60 = ticks per second
        return ticksPerMinute / 60.0;
    }

    private double ticksPerSecToRPM(double ticksPerSec, double ticksPerRev) {
        // ticks/sec * 60 sec/min / ticksPerRev
        double ticksPerMin = ticksPerSec * 60.0;
        return ticksPerMin / ticksPerRev;
    }
}
