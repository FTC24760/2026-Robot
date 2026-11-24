package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Hood;

import com.bylazar.configurables.annotations.Configurable;
/**
 * Drivetrain and butterflies only, for checking the drive base without the other mechanisms.
 * Field-relative by default; hold left bumper for robot-relative.
 * A resets heading, right bumper inverts the butterflies.
 */
@Configurable
@TeleOp(name = "Test: Hood", group = "Test")
public class HoodTest extends OpMode {
    private Hood hood;
    public static double hoodAngle = 0.9;

    @Override
    public void init() {
        hood = new Hood(hardwareMap);
    }

    @Override
    public void loop() {
        hoodAngle += gamepad1.right_trigger * 0.01;
        hoodAngle -= gamepad1.left_trigger * 0.01;
        hood.setPosition(hoodAngle);
        telemetry.addData("Hood Angle: ", hoodAngle);
        telemetry.update();
    }
}
