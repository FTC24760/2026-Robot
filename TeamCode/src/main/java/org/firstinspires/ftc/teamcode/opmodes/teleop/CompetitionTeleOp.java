package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

/** The plain robot: one-click controls, original shooter controller, no extras. */
@TeleOp(name = "TeleOp", group = "Competition")
public class CompetitionTeleOp extends RobotTeleOp {
    @Override
    protected Robot createRobot() {
        return new Robot(this);
    }
}
