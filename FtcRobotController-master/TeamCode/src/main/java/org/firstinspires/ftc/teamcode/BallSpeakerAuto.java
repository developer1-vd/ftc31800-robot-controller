package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mechanisms.BallDetector;

/**
 * Camera-only autonomous: watches for yellow / blue / red balls and the Driver Station
 * says the color out loud ("red ball") when one comes into view.
 *
 * Needs only a Control Hub, battery, and a webcam named "Webcam 1" in the robot
 * configuration. No motors, servos, or gamepad. Works during INIT and after Play.
 */
@Autonomous(name = "BallSpeakerAuto", group = "Vision")
public class BallSpeakerAuto extends OpMode {

    // Spoken announcements on the Driver Station. Set to false for silence.
    private static final boolean SOUND_ENABLED = true;

    private final BallDetector ballDetector = new BallDetector();

    @Override
    public void init() {
        ballDetector.init(hardwareMap, telemetry);
        ballDetector.setSoundEnabled(SOUND_ENABLED);
        telemetry.addLine("Camera-only ball speaker ready. Show it a ball.");
    }

    @Override
    public void init_loop() {
        detectAndReport();
    }

    @Override
    public void loop() {
        detectAndReport();
    }

    private void detectAndReport() {
        ballDetector.update();
        telemetry.addData("Sound", ballDetector.isSoundEnabled() ? "ON" : "OFF");
        ballDetector.displayTelemetry();
        telemetry.update();
    }

    @Override
    public void stop() {
        ballDetector.stop();
    }
}
