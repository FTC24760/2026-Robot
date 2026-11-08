package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.ButterflyController;
import org.firstinspires.ftc.teamcode.subsystems.FieldCentricMecanumDrive;

@TeleOp(name = "Robot: Field Relative Mecanum Drive", group = "Robot")
public class FieldCentricDriveTest extends OpMode {
    private FieldCentricMecanumDrive drive;
    private ButterflyController butterfly;

    @Override
    public void init() {
        drive = new FieldCentricMecanumDrive(hardwareMap);
        butterfly = new ButterflyController(hardwareMap);
    }

    @Override
    public void loop() {
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;
        if (gamepad1.a) drive.resetHeading();
        butterfly.update(forward, strafe, rotate, gamepad1.right_bumper);
        if (gamepad1.left_bumper) {
            drive.driveRobotRelative(forward, strafe, rotate);
        } else {
            drive.driveFieldRelative(forward, strafe, rotate);
        }
        telemetry.addData("Butterfly", butterfly.getState());
        telemetry.addData("Drive Mode",
                gamepad1.left_bumper ? "Robot Relative" : "Field Relative");
        telemetry.update();
    }

    @Override
    public void stop() {
        if (drive != null) drive.stop();
    }
}
