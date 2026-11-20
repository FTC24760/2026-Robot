package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.constants.ShotConstants;
import org.firstinspires.ftc.teamcode.subsystems.Butterfly;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.ImuMecanumDrive;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Drivetrain and butterflies only, for checking the drive base without the other mechanisms.
 * Field-relative by default; hold left bumper for robot-relative.
 * A resets heading, right bumper inverts the butterflies.
 */
@TeleOp(name = "Test: Hood Reset", group = "Test")
public class HoodReset extends OpMode {
    private  Servo leftHood;
    private  Servo rightHood;


    @Override
    public void init() {
        leftHood = hardwareMap.tryGet(Servo.class, ShotConstants.HOOD_LEFT_SERVO_NAME);
        rightHood = hardwareMap.tryGet(Servo.class, ShotConstants.HOOD_RIGHT_SERVO_NAME);
    }

    @Override
    public void loop() {
        leftHood.setPosition(0);
        rightHood.setPosition(1);
    }
}
