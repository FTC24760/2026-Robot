package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name = "TeleOp", group = "Competition")
public class CompetitionTeleOp extends OpMode {
    private Robot robot;

    @Override
    public void init() {
        robot = new Robot(this);
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
        // robot is null if init() failed (e.g. a device missing from the configuration).
        if (robot != null) {
            robot.stop();
        }
    }
}
