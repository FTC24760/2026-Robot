package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

/**
 * "TeleOp (Shooter Assist)" with the original hold-to-run controls instead of one-click
 * shooting: same rapid-fire shooter fixes and CSV logging, but the driver holds the
 * right trigger to spin the shooter, holds the left trigger to intake, and opens the
 * blocker with A. Controls are listed in ManualControls.
 */
@TeleOp(name = "TeleOp (Shooter Assist, manual)", group = "Competition")
public class ShooterAssistManualTeleOp extends OpMode {
    private Robot robot;

    @Override
    public void init() {
        robot = new Robot(this)
                .enableShooterAssist()
                .enableShooterLogging()
                .useManualControls();
    }

    @Override
    public void start() {
        robot.start();
    }

    @Override
    public void loop() {
        robot.update();
    }

    @Override
    public void stop() {
        if (robot != null) {
            robot.stop();
        }
    }
}
