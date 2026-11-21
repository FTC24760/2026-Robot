package org.firstinspires.ftc.teamcode.opmodes.teleop;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.constants.TeleOpConstants;
import org.firstinspires.ftc.teamcode.utils.Alliance;

/**
 * Competition TeleOp: manual turret aiming, guarded shooting, and flower intake.
 * TeleOp Red and TeleOp Blue select their own starting pose. Limelight is disabled.
 *
 * Options that used to be separate OpModes are switches in RobotConstants, read at INIT:
 *   RAPID_FIRE       fixed-beat feeding instead of waiting for the wheel each ball
 *   LOG_SHOOTER      record shooter RPM to a CSV
 * Controls are listed in DriverControls.
 */
public abstract class MatchTeleOp extends RobotTeleOp {
    private final Alliance alliance;

    protected MatchTeleOp(Alliance alliance) {
        this.alliance = alliance;
    }

    @Override
    protected Robot createRobot() {
        Robot robot = new Robot(this).enableShooterAssist();
        robot.setPose(TeleOpConstants.startingPose(alliance));
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Start X (in)", robot.drivetrain.getPose().x());
        telemetry.addData("Start Y (in)", robot.drivetrain.getPose().y());
        telemetry.addData("Start heading (deg)",
                Math.toDegrees(robot.drivetrain.getPose().heading()));
        telemetry.addLine("Verify TeleOpConstants matches the actual starting position.");
        if (RobotConstants.RAPID_FIRE) {
            robot.enableRapidFire();
        }
        if (RobotConstants.LOG_SHOOTER) {
            robot.enableShooterLogging();
        }
        return robot;
    }
}
