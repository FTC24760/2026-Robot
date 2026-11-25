package org.firstinspires.ftc.teamcode.opmodes.teleop;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.Alliance;

/**
 * The competition TeleOp. TeleOp Red and TeleOp Blue differ only in the alliance they
 * give the Robot, which picks the HIVE to aim at. Button bindings are listed in Controls.
 */
public abstract class MatchTeleOp extends RobotTeleOp {
    private final Alliance alliance;

    protected MatchTeleOp(Alliance alliance) {
        this.alliance = alliance;
    }

    @Override
    protected Robot createRobot() {
        return new Robot(this, alliance);
    }
}