package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Butterfly;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

/**
 * Drivetrain and butterflies only, for checking the drive base without the other mechanisms.
 * Field-relative by default; hold left bumper for robot-relative.
 * A resets heading, right bumper inverts the butterflies.
 */
@TeleOp(name = "Test: Drive", group = "Test")
public class DriveTest extends OpMode {
    private Drivetrain drivetrain;
    private Butterfly butterfly;

    @Override
    public void init() {
        drivetrain = new Drivetrain(hardwareMap);
        butterfly = new Butterfly(hardwareMap);
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

        butterfly.update(forward, strafe, rotate, gamepad1.right_bumper);

        if (robotRelative) {
            drivetrain.driveRobotCentric(forward, strafe, rotate);
        } else {
            drivetrain.driveFieldCentric(forward, strafe, rotate);
        }

        butterfly.addTelemetry(telemetry);
        telemetry.addData("Drive Mode", robotRelative ? "Robot Relative" : "Field Relative");
        telemetry.update();
    }
}
