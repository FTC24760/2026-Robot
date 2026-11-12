package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.Alliance;

/**
 * Competition controls plus the rapid-fire shooter fixes and Limelight auto-aim: while
 * the upward-facing CELL of our HIVE is in view, shooter speed and hood angle follow the
 * measured distance. Out of view, the shooter uses ShooterConstants.TARGET_RPM.
 *
 * Needs a Limelight ("limelight") and a hood servo ("hood") in the robot configuration.
 * The red and blue OpModes below differ only in which HIVE they aim at.
 */
public abstract class AutoAimTeleOp extends OpMode {
    private final Alliance alliance;
    private Robot robot;

    protected AutoAimTeleOp(Alliance alliance) {
        this.alliance = alliance;
    }

    @Override
    public void init() {
        robot = new Robot(this).enableShooterAssist().enableAutoAim(alliance);
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
        if (robot != null) {
            robot.stop();
        }
    }
}
