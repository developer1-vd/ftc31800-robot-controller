package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * The intake, on a goBILDA Yellow Jacket 312 RPM motor (5203 series, 19.2:1 gearbox,
 * 537.7 encoder ticks per output turn, ~9.2 A stall), named "intake" in the configuration.
 *
 *  - run(+1) = full speed IN, run(-1) = full speed OUT, run(0) = stop; in between = slower.
 *    "In" is set once, by INTAKE_DIRECTION, so every OpMode agrees which way is in.
 *  - Speed control: with the encoder plugged in it holds IN_RPM / OUT_RPM (steady as the battery
 *    drains) using the Control Hub's velocity control.
 *  - Jam clearing: if it's pulling in but barely turning while drawing a lot of current for
 *    JAM_MS, a ball is stuck: it reverses for UNJAM_MS, then pulls in again.
 *  - No encoder cable? If the motor is powered but the encoder count never moves and the current
 *    is low (so it isn't jammed), it switches to plain power control and stops jam checking.
 *
 * Call run() every loop while it should run (the OpModes already do).
 */
public class Intake {

    // goBILDA Yellow Jacket 312 RPM
    public static final double TICKS_PER_REV = 537.7;
    public static final double MAX_RPM       = 312;

    /** Which motor direction pulls balls IN. Flip to REVERSE if run(+1) pushes balls out. */
    public static final DcMotorSimple.Direction INTAKE_DIRECTION = DcMotorSimple.Direction.FORWARD;

    public static final double IN_RPM  = 300;    // just under the motor's 312 so velocity control has headroom
    public static final double OUT_RPM = 300;

    // Jam clearing
    public static final boolean JAM_CLEARING = true;
    private static final double SPIN_UP_MS        = 400;   // ignore low speed while starting
    private static final double JAM_SPEED_FRACTION = 0.25; // below 25% of the wanted speed...
    private static final double JAM_CURRENT_A     = 3.0;   // ...while drawing more than this...
    private static final double JAM_MS            = 300;   // ...for this long = jammed
    private static final double UNJAM_MS          = 250;   // reverse this long to clear it
    private static final double UNJAM_FRACTION    = 0.6;
    // Encoder check
    private static final double ENCODER_CHECK_MS  = 600;   // running this long with no encoder movement...

    private DcMotorEx motor = null;
    private Telemetry telemetry;
    private boolean velocityMode = true;
    private boolean encoderMissing = false;

    private double fraction = 0;          // what was asked for: -1..1
    private boolean running = false;
    private int startPosition = 0;
    private final ElapsedTime runTimer = new ElapsedTime();
    private final ElapsedTime slowTimer = new ElapsedTime();
    private final ElapsedTime reverseTimer = new ElapsedTime();
    private boolean reversing = false;
    private int jams = 0;

    /** Finds and sets up the motor. Returns false if there is no motor with that name. */
    public boolean init(HardwareMap hwMap, Telemetry telemetry, String name) {
        this.telemetry = telemetry;
        try {
            motor = hwMap.get(DcMotorEx.class, name);
        } catch (Exception e) {
            motor = null;
            return false;
        }
        motor.setDirection(INTAKE_DIRECTION);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        velocityMode = true;
        encoderMissing = false;
        return true;
    }

    public boolean isConnected() {
        return motor != null;
    }

    /** +1 = full speed in, -1 = full speed out, 0 = stop. Call every loop while running. */
    public void run(double amount) {
        if (motor == null) return;
        amount = Math.max(-1, Math.min(1, amount));

        if (amount == 0) {
            motor.setPower(0);
            running = false;
            reversing = false;
            fraction = 0;
            return;
        }
        // Starting, or changing direction: restart the timers
        if (!running || Math.signum(amount) != Math.signum(fraction)) {
            running = true;
            reversing = false;
            runTimer.reset();
            slowTimer.reset();
            startPosition = motor.getCurrentPosition();
        }
        fraction = amount;

        // Clearing a jam: reverse briefly, then pull in again
        if (reversing) {
            if (reverseTimer.milliseconds() < UNJAM_MS) {
                command(-Math.signum(amount) * UNJAM_FRACTION);
                return;
            }
            reversing = false;
            runTimer.reset();
            slowTimer.reset();
            startPosition = motor.getCurrentPosition();
        }
        command(amount);

        // Encoder not plugged in? Powered, encoder never moved, and not straining (so not jammed)
        if (velocityMode && runTimer.milliseconds() > ENCODER_CHECK_MS
                && motor.getCurrentPosition() == startPosition && amps() < JAM_CURRENT_A) {
            encoderMissing = true;
            velocityMode = false;
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            command(amount);
        }

        // Jam: pulling in, much too slow, and straining
        if (JAM_CLEARING && velocityMode && amount > 0 && runTimer.milliseconds() > SPIN_UP_MS) {
            boolean slow = Math.abs(getRpm()) < JAM_SPEED_FRACTION * amount * IN_RPM;
            if (!slow) {
                slowTimer.reset();
            } else if (slowTimer.milliseconds() > JAM_MS && amps() > JAM_CURRENT_A) {
                jams++;
                reversing = true;
                reverseTimer.reset();
            }
        }
    }

    /** Full speed in. */
    public void in() {
        run(1);
    }

    /** Full speed out. */
    public void out() {
        run(-1);
    }

    public void stop() {
        run(0);
    }

    /** Output shaft speed (RPM, + = in); 0 without an encoder. */
    public double getRpm() {
        if (motor == null || encoderMissing) return 0;
        return motor.getVelocity() / TICKS_PER_REV * 60.0;
    }

    public int getJamCount() {
        return jams;
    }

    public boolean isClearingJam() {
        return reversing;
    }

    public boolean isEncoderMissing() {
        return encoderMissing;
    }

    public void addTelemetry() {
        if (telemetry == null) return;
        if (motor == null) {
            telemetry.addData("Intake", "MISSING (no \"intake\" motor)");
            return;
        }
        String dir = fraction > 0 ? "IN" : fraction < 0 ? "OUT" : "stopped";
        if (reversing) dir = "clearing jam";
        telemetry.addData("Intake", "%s  %.0f RPM  %.1f A  jams %d  %s", dir, getRpm(), amps(), jams,
                encoderMissing ? "(no encoder: power mode)" : "");
    }

    // -------------------------------------------------

    private void command(double amount) {
        double rpm = amount * (amount > 0 ? IN_RPM : OUT_RPM);
        if (velocityMode) {
            motor.setVelocity(rpm / 60.0 * TICKS_PER_REV);   // ticks per second
        } else {
            motor.setPower(rpm / MAX_RPM);
        }
    }

    private double amps() {
        try {
            return motor.getCurrent(CurrentUnit.AMPS);
        } catch (Exception e) {
            return 0;   // current reading not available: jam detection can't confirm, so it won't trigger
        }
    }
}
