package org.firstinspires.ftc.teamcode.opmodes.test;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.FlowerArm;
import org.firstinspires.ftc.teamcode.subsystems.Hood;

/**
 * Drivetrain and butterflies only, for checking the drive base without the other mechanisms.
 * Field-relative by default; hold left bumper for robot-relative.
 * A resets heading, right bumper inverts the butterflies.
 */
@Configurable
@TeleOp(name = "Test: Flower", group = "Test")
public class FlowerTest extends OpMode {
    private Servo flowerServo;
    public static double flowerAngle = 0.5;

    @Override
    public void init() {
        flowerServo = hardwareMap.get(Servo.class, "flower");
    }

    @Override
    public void loop() {
        flowerAngle += gamepad1.right_trigger * 0.01;
        flowerAngle -= gamepad1.left_trigger * 0.01;
        flowerServo.setPosition(flowerAngle);
        telemetry.addData("Flower pos: ", flowerAngle);
        telemetry.update();
    }
}
