package org.firstinspires.ftc.teamcode.opmodes.teleop;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.Alliance;

@TeleOp(name = "TeleOp", group = "temp")
public class TempTeleOp extends OpMode {
    Robot robot;
    @Override
    public void init() {
        robot = new Robot(this);
        robot.setAlliance(Alliance.RED);
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
        robot.stop();
    }
}
