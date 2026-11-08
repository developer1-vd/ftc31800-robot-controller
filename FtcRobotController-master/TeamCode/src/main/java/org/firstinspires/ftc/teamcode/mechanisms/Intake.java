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
 *  - Drive: plain motor power by default, exactly like the old teleop's intake.setPower(), so it
 *    always turns. SPEED_CONTROL = true holds IN_RPM / OUT_RPM with the hub's velocity control
 *    instead (needs this motor's encoder cable in this motor's encoder port).
 *  - Finds the motor as "intake" (or a few common spellings) and, if there is none, lists the
 *    motor names that ARE in the configuration so the name mismatch is obvious.
 *  - Jam clearing (needs the encoder): if it's pulling in but barely turning while drawing a lot of current for
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

    /** false = plain power (always moves, like the old code); true = hold IN_RPM with the encoder. */
    public static final boolean SPEED_CONTROL = false;

    /** Names tried, in order, when the asked-for name isn't in the configuration. */
    private static final String[] OTHER_NAMES = { "intake", "intakeMotor", "intake_motor", "Intake", "intake motor" };

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
    private boolean velocityMode = SPEED_CONTROL;
    private String foundName = "";
    private String problem = "";
    private boolean encoderMissing = false;
    private boolean encoderSeen = false;  // the encoder count has moved at least once = it's plugged in

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
        motor = null;
        problem = "";
        String wrongType = "";
        String[] names = new String[OTHER_NAMES.length + 1];
        names[0] = name;
        System.arraycopy(OTHER_NAMES, 0, names, 1, OTHER_NAMES.length);
        for (String n : names) {
            try {
                motor = hwMap.get(DcMotorEx.class, n);
                foundName = n;
                break;
            } catch (Exception e) {
                // Is it there, but configured as something other than a motor (e.g. a servo)?
                try {
                    Object other = hwMap.get(n);
                    if (other != null && wrongType.isEmpty()) {
                        wrongType = "\"" + n + "\" is configured as " + other.getClass().getSimpleName()
                                + ", not a motor: set its type to the goBILDA 5203 312 RPM motor.";
                    }
                } catch (Exception ignored) { }
            }
        }
        if (motor == null) {
            problem = !wrongType.isEmpty() ? wrongType
                    : "No motor named \"" + name + "\" in the robot configuration. Motors found: "
                    + motorNames(hwMap) + ". Rename the intake motor to \"" + name + "\".";
            if (telemetry != null) telemetry.addLine("⚠️ INTAKE: " + problem);
            return false;
        }
        try {
            motor.setDirection(INTAKE_DIRECTION);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(SPEED_CONTROL ? DcMotor.RunMode.RUN_USING_ENCODER : DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception e) {
            problem = "intake motor setup failed: " + e.getMessage();
        }
        velocityMode = SPEED_CONTROL;
        encoderMissing = false;
        if (telemetry != null && !foundName.equals(name)) {
            telemetry.addLine("Intake: using the motor named \"" + foundName + "\".");
        }
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
        if (motor.getCurrentPosition() != startPosition) encoderSeen = true;

        // Encoder not plugged in? Powered and the encoder never moved. (With speed control, don't
        // wait for low current: speed control without an encoder would run it wrong.)
        if (!encoderMissing && !encoderSeen && runTimer.milliseconds() > ENCODER_CHECK_MS
                && motor.getCurrentPosition() == startPosition && (velocityMode || amps() < JAM_CURRENT_A)) {
            encoderMissing = true;
            if (velocityMode) {
                velocityMode = false;
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                command(amount);
            }
        }

        // Jam: pulling in, much too slow, and straining (only with a working encoder)
        if (JAM_CLEARING && encoderSeen && amount > 0 && runTimer.milliseconds() > SPIN_UP_MS) {
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

    /** Why the intake can't run ("" if it's fine). */
    public String getProblem() {
        return problem;
    }

    public boolean isEncoderMissing() {
        return encoderMissing;
    }

    public void addTelemetry() {
        if (telemetry == null) return;
        if (motor == null) {
            telemetry.addData("Intake", "MISSING: " + problem);
            return;
        }
        String dir = fraction > 0 ? "IN" : fraction < 0 ? "OUT" : "stopped";
        if (reversing) dir = "clearing jam";
        telemetry.addData("Intake", "%s  %.0f RPM  %.1f A  jams %d  %s", dir, getRpm(), amps(), jams,
                encoderMissing ? "(no encoder: no jam clearing)" : velocityMode ? "(speed control)" : "");
        if (!problem.isEmpty()) telemetry.addLine("⚠️ INTAKE: " + problem);
    }

    // -------------------------------------------------

    private void command(double amount) {
        if (velocityMode) {
            double rpm = amount * (amount > 0 ? IN_RPM : OUT_RPM);
            motor.setVelocity(rpm / 60.0 * TICKS_PER_REV);   // ticks per second
        } else {
            motor.setPower(amount);                          // same as the old teleop: +1 = full power
        }
    }

    private static String motorNames(HardwareMap hwMap) {
        StringBuilder sb = new StringBuilder();
        try {
            for (DcMotor m : hwMap.getAll(DcMotor.class)) {
                for (String n : hwMap.getNamesOf(m)) {
                    if (sb.length() > 0) sb.append(", ");
                    sb.append('"').append(n).append('"');
                }
            }
        } catch (Exception ignored) { }
        return sb.length() == 0 ? "none" : sb.toString();
    }

    private double amps() {
        try {
            return motor.getCurrent(CurrentUnit.AMPS);
        } catch (Exception e) {
            return 0;   // current reading not available: jam detection can't confirm, so it won't trigger
        }
    }
}
