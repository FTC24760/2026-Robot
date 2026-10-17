package org.firstinspires.ftc.teamcode.opmodes.teleop;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Configurable
@TeleOp(
        name = "Servo Zeroer",
        group = "Test"
)
public class servoTest extends OpMode  {
    Servo servo;

    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "s");
    }

    @Override
    public void loop() {
        servo.setPosition(0);
    }
}
