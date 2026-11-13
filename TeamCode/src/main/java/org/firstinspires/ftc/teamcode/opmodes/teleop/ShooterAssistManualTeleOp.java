package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

/**
 * "TeleOp (Shooter Assist)" with the original hold-to-run controls (see ManualControls):
 * hold the right trigger to spin the shooter, hold the left trigger to intake, and open
 * the blocker with A. Feeding is staggered: each ball waits for the wheel to recover.
 */
@TeleOp(name = "TeleOp (Shooter Assist, manual)", group = "Competition")
public class ShooterAssistManualTeleOp extends RobotTeleOp {
    @Override
    protected Robot createRobot() {
        return new Robot(this).enableShooterAssist().enableShooterLogging().useManualControls();
    }
}
