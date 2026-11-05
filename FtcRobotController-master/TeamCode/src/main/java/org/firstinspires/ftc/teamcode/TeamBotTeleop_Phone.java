package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.PhoneGamepadServer;

/**
 * PRACTICE / DEMO ONLY: TeamBotTeleop_latest, plus phone touchscreen controls.
 *
 * Exactly the same robot behavior as TeamBotTeleop_latest; the only difference is that a phone
 * on the Control Hub's Wi-Fi can open http://192.168.43.1:8090 and drive with on-screen
 * joysticks and buttons, as gamepad 1 ("Driver 1") or gamepad 2 ("Driver 2"). The real
 * gamepads on the Driver Station keep working at the same time.
 *
 * Do NOT use this in a match: only the Driver Station may control the robot, and no other
 * device may be on the robot's Wi-Fi during a match. Use TeamBotTeleop_latest for matches.
 */
@TeleOp(name = "TeamBotTeleop_Phone (practice)", group = "Drive")
public class TeamBotTeleop_Phone extends TeamBotTeleop_latest {

    private PhoneGamepadServer phone = null;

    @Override
    public void init() {
        super.init();
        try {
            phone = new PhoneGamepadServer(hardwareMap.appContext);
            phone.start();
            telemetry.addLine("📱 Phone controls: join the robot Wi-Fi, open " + PhoneGamepadServer.ADDRESS);
        } catch (Exception e) {
            phone = null;
            telemetry.addData("⚠️ Phone controls unavailable", e.getMessage());
        }
    }

    @Override
    public void init_loop() {
        applyPhone();
        super.init_loop();
    }

    @Override
    public void loop() {
        applyPhone();
        super.loop();
    }

    /**
     * The SDK refreshes gamepad1/gamepad2 from the Driver Station before every loop, so the
     * phone's input is blended in fresh each time (nothing can stay "stuck" pressed).
     */
    private void applyPhone() {
        if (phone == null) return;
        phone.applyTo(gamepad1, gamepad2);
        telemetry.addData("📱 Phone", phone.isPhoneConnected()
                ? "connected" : "not connected (" + PhoneGamepadServer.ADDRESS + ")");
    }

    @Override
    public void stop() {
        if (phone != null) phone.stop();
        super.stop();
    }
}
