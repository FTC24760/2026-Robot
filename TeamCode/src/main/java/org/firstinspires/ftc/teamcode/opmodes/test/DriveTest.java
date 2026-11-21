package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.ImuMecanumDrive;

/**
 * Mecanum drivetrain only, for checking the drive base without the other mechanisms.
 * Field-relative by default; hold left bumper for robot-relative.
 * A resets heading.
 */
@TeleOp(name = "Test: Drive", group = "Test")
public class DriveTest extends OpMode {
    private ImuMecanumDrive drivetrain;

    @Override
    public void init() {
        drivetrain = new ImuMecanumDrive(hardwareMap);
    }

    @Override
    public void loop() {
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;
        boolean robotRelative = gamepad1.left_bumper;

        if (gamepad1.a) {
            drivetrain.resetYaw();
        }

        if (robotRelative) {
            drivetrain.driveRobotCentric(forward, strafe, rotate);
        } else {
            drivetrain.driveFieldCentric(forward, strafe, rotate);
        }

        telemetry.addData("Drive Mode", robotRelative ? "Robot Relative" : "Field Relative");
        telemetry.update();
    }
}
