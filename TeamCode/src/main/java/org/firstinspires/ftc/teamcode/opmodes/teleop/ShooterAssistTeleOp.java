package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

/**
 * One-click controls plus the shooter fixes in ShooterAssistConstants (recovery boost,
 * staggered feed, working integral, idle spin). Records shooter RPM and power to a CSV.
 */
@TeleOp(name = "TeleOp (Shooter Assist)", group = "Competition")
public class ShooterAssistTeleOp extends RobotTeleOp {
    @Override
    protected Robot createRobot() {
        return new Robot(this).enableShooterAssist().enableShooterLogging();
    }
}
