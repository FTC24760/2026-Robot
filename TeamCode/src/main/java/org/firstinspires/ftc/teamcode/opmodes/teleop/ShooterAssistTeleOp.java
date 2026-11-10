package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

/**
 * Same robot and controls as the competition TeleOp, plus the rapid-fire shooter fixes
 * (recovery boost, feed gating, working integral, idle spin; see ShooterAssistConstants).
 * Also records shooter RPM and power to a CSV so a burst can be compared against the
 * "Shooter Diagnostic Logger" OpMode, which runs the original controller.
 *
 * Extra control: Y stops the idle spin (the wheel keeps turning after the first shot).
 */
@TeleOp(name = "TeleOp (Shooter Assist)", group = "Competition")
public class ShooterAssistTeleOp extends OpMode {
    private Robot robot;

    @Override
    public void init() {
        robot = new Robot(this).enableShooterAssist().enableShooterLogging();
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
