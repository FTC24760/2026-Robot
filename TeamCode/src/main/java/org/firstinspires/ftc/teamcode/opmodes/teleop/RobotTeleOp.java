package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot;

/**
 * Base for every TeleOp that runs the whole robot. A TeleOp only says how its Robot is
 * set up (which options are switched on); this class forwards the OpMode lifecycle.
 */
public abstract class RobotTeleOp extends OpMode {
    private Robot robot;

    /** Build the Robot with the options this TeleOp wants. Called once from init(). */
    protected abstract Robot createRobot();

    @Override
    public void init() {
        robot = createRobot();
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
