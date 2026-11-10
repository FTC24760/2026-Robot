package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

/**
 * Same robot and controls as the competition TeleOp, but records shooter RPM to a CSV
 * (see AsyncRpmLogger) for tuning the flywheel controller.
 */
@TeleOp(name = "Shooter Diagnostic Logger", group = "Test")
public class ShooterDiagnosticTeleOp extends OpMode {
    private Robot robot;

    @Override
    public void init() {
        robot = new Robot(this).enableShooterLogging();
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
