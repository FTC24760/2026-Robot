package org.firstinspires.ftc.teamcode.opmodes.teleop;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
@TeleOp(
        name = "Servo TESTS",
        group = "Test"
)
public class ServoTests extends OpMode  {
    Servo leftServo;
    Servo rightServo;

    // probably like needs to only move 0.08-0.1 to retract
    double leftServoPosition = 1.0;
    double rightServoPosition = 0.0;

    @Override
    public void init() {
        leftServo = hardwareMap.get(Servo.class, "leftbutterfly");
        rightServo = hardwareMap.get(Servo.class, "rightbutterfly");
    }

    @Override
    public void loop() {
        leftServo.setPosition(leftServoPosition);
        rightServo.setPosition(rightServoPosition);

        if (gamepad1.right_trigger > 0.2) {
            if (leftServoPosition < 1.0) {
                leftServoPosition += 0.002;
            }
        } else if (gamepad1.left_trigger > 0.2) {
            if (leftServoPosition > 0.0) {
                leftServoPosition -= 0.002;
            }
        }

        if (gamepad1.right_bumper) {
            if (rightServoPosition < 1.0) {
                rightServoPosition += 0.002;
            }
        } else if (gamepad1.left_bumper) {
            if (rightServoPosition > 0.0) {
                rightServoPosition -= 0.002;
            }
        }

        telemetry.addData("Left Servo Position", leftServoPosition);
        telemetry.addData("Right Servo Position", rightServoPosition);
        telemetry.update();
    }
}
