package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.constants.ShotConstants;

/** Drives the hood servo to the end of its travel, for fitting the horn or linkage. */
@TeleOp(name = "Test: Hood Reset", group = "Test")
public class HoodReset extends OpMode {
    private Servo hood;

    @Override
    public void init() {
        hood = hardwareMap.tryGet(Servo.class, ShotConstants.HOOD_SERVO_NAME);
    }

    @Override
    public void loop() {
        if (hood == null) {
            telemetry.addLine("No servo \"" + ShotConstants.HOOD_SERVO_NAME + "\" in the configuration");
        } else {
            hood.setPosition(1);
            telemetry.addData("Hood servo", "at 1.0");
        }
        telemetry.update();
    }
}
