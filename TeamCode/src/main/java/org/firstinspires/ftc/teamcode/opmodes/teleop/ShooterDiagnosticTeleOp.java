package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

/**
 * The plain robot with the original shooter controller, recording shooter RPM and power
 * to a CSV (see AsyncRpmLogger) as a baseline to compare the assist TeleOps against.
 */
@TeleOp(name = "Shooter Diagnostic Logger", group = "Test")
public class ShooterDiagnosticTeleOp extends RobotTeleOp {
    @Override
    protected Robot createRobot() {
        return new Robot(this).enableShooterLogging();
    }
}
