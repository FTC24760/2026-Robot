package org.firstinspires.ftc.teamcode.opmodes.auto;

import static com.pedropathing.api.Paths.line;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot;

/** Drive straight backward about one tile, then stop. Same for both alliances. */
@Autonomous(name = "Leave", group = "Competition")
public class Leave extends OpMode {
    public static double DISTANCE_IN = 30;

    private Robot robot;

    @Override
    public void init() {
        robot = new Robot(this);
    }

    @Override
    public void start() {
        robot.start();
        Pose here = new Pose(0, 0, 0);
        robot.setPose(here);
        robot.drivetrain.follow(line(here, new Pose(-DISTANCE_IN, 0, 0)).constant(0));
    }

    @Override
    public void loop() {
        robot.beginLoop();
        robot.drivetrain.update();
        robot.endLoop();
    }

    @Override
    public void stop() {
        if (robot != null) {
            robot.stop();
        }
    }
}
