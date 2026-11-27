package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.constants.TeleOpConstants;
import org.firstinspires.ftc.teamcode.utils.Alliance;
import org.firstinspires.ftc.teamcode.utils.PoseHandoff;

/**
 * Competition TeleOp: Limelight auto-aim (turret, and RPM/hood from the shot table),
 * guarded shooting, and flower intake. Starts from the pose AUTO ended at when AUTO ran
 * just before for the same alliance (PoseHandoff); otherwise from TeleOpConstants.
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
        Robot robot = new Robot(this).enableShooterAssist().enableAutoAim(alliance)
                .enableTeleOpMemory();
        // Carry on from where AUTO left the robot; fall back to the fixed start pose when
        // there was no AUTO just before (practice, or AUTO for the other alliance).
        Pose fromAuto = PoseHandoff.take(alliance);
        robot.setPose(fromAuto != null ? fromAuto : TeleOpConstants.startingPose(alliance));
        // Aim at the CELL AUTO believed was up; it stays unconfirmed until the camera reads.
        robot.syncUpSide(PoseHandoff.takenUpSide());
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Start pose from", fromAuto != null
                ? "end of AUTO" : "TeleOpConstants (no recent AUTO)");
        telemetry.addData("Start X (in)", robot.drivetrain.getPose().x());
        telemetry.addData("Start Y (in)", robot.drivetrain.getPose().y());
        telemetry.addData("Start heading (deg)",
                Math.toDegrees(robot.drivetrain.getPose().heading()));
        if (fromAuto == null) {
            telemetry.addLine("Verify TeleOpConstants matches the actual starting position.");
        }
        if (RobotConstants.RAPID_FIRE) {
            robot.enableRapidFire();
        }
        if (RobotConstants.LOG_SHOOTER) {
            robot.enableShooterLogging();
        }
        return robot;
    }
}
