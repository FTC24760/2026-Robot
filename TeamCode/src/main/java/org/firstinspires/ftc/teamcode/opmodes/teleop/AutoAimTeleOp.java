package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.Alliance;

/**
 * The full-featured competition TeleOp: one-click shooting, the rapid-fire shooter
 * fixes, and Limelight auto-aim. While the upward-facing CELL of our HIVE is in view,
 * the turret turns to it and shooter speed and hood angle follow the measured distance.
 * Out of view (or with auto-aim switched off on A), aiming is manual and the shooter
 * uses ShooterConstants.TARGET_RPM.
 *
 * If the Limelight ("limelight") is missing from the robot configuration the OpMode
 * still runs, without auto-aim. The red and blue OpModes differ only in which HIVE's
 * AprilTags they aim at. Controls are listed in DriverControls.
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
