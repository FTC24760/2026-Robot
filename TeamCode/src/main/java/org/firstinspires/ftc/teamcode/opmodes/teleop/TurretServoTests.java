package org.firstinspires.ftc.teamcode.opmodes.teleop;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
@TeleOp(
        name = "Turret Servo Tests TESTS",
        group = "Test"
)
public class TurretServoTests extends OpMode  {
    Servo leftServo;
    Servo rightServo;

    double leftServoPosition = 0.5;
    double rightServoPosition = 0.5;

    @Override
    public void init() {
        leftServo = hardwareMap.get(Servo.class, "leftturret");
        rightServo = hardwareMap.get(Servo.class, "rightturret");
    }

    @Override
    public void loop() {
        leftServo.setPosition(leftServoPosition);
        rightServo.setPosition(rightServoPosition);

        if (gamepad1.right_trigger > 0.2) {
            if (leftServoPosition < 1.0) {
                leftServoPosition += 0.005;
            }
        } else if (gamepad1.left_trigger > 0.2) {
            if (leftServoPosition > 0.0) {
                leftServoPosition -= 0.005;
            }
        }

        if (gamepad1.right_bumper) {
            if (rightServoPosition < 1.0) {
                rightServoPosition += 0.005;
            }
        } else if (gamepad1.left_bumper) {
            if (rightServoPosition > 0.0) {
                rightServoPosition -= 0.005;
            }
        }

        telemetry.addData("Left Servo Position", leftServoPosition);
        telemetry.addData("Right Servo Position", rightServoPosition);
        telemetry.update();
    }
}
