package org.firstinspires.ftc.teamcode.opmodes.teleop;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.utils.Alliance;

/**
 * The competition TeleOp: shooter fixes, Limelight auto-aim for our HIVE, flower intake.
 * TeleOp Red and TeleOp Blue differ only in which HIVE they aim at.
 *
 * Options that used to be separate OpModes are switches in RobotConstants, read at INIT:
 *   MANUAL_CONTROLS  hold-to-run controls (ManualControls) instead of one-click
 *   RAPID_FIRE       fixed-beat feeding instead of waiting for the wheel each ball
 *   LOG_SHOOTER      record shooter RPM to a CSV
 * Controls are listed in DriverControls (or ManualControls).
 */
public abstract class MatchTeleOp extends RobotTeleOp {
    private final Alliance alliance;

    protected MatchTeleOp(Alliance alliance) {
        this.alliance = alliance;
    }

    @Override
    protected Robot createRobot() {
        Robot robot = new Robot(this);
        return robot;
    }
}