package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Base for finding servo positions by hand on a pair of servos.
 * Triggers nudge the left servo (right trigger up, left trigger down);
 * bumpers nudge the right servo (right bumper up, left bumper down).
 * Read the positions off telemetry and copy them into the matching constants file.
 */
public abstract class DualServoTest extends OpMode {
    private final String leftName;
    private final String rightName;
    private final double step;

    private Servo leftServo;
    private Servo rightServo;
    private double leftPosition;
    private double rightPosition;

    /** @param step position change per loop while a control is held */
    protected DualServoTest(
            String leftName,
            String rightName,
            double leftStart,
            double rightStart,
            double step
    ) {
        this.leftName = leftName;
        this.rightName = rightName;
        this.leftPosition = leftStart;
        this.rightPosition = rightStart;
        this.step = step;
    }

    @Override
    public void init() {
        leftServo = hardwareMap.get(Servo.class, leftName);
        rightServo = hardwareMap.get(Servo.class, rightName);
    }

    @Override
    public void loop() {
        leftServo.setPosition(leftPosition);
        rightServo.setPosition(rightPosition);

        if (gamepad1.right_trigger > 0.2) {
            if (leftPosition < 1.0) {
                leftPosition += step;
            }
        } else if (gamepad1.left_trigger > 0.2) {
            if (leftPosition > 0.0) {
                leftPosition -= step;
            }
        }

        if (gamepad1.right_bumper) {
            if (rightPosition < 1.0) {
                rightPosition += step;
            }
        } else if (gamepad1.left_bumper) {
            if (rightPosition > 0.0) {
                rightPosition -= step;
            }
        }

        telemetry.addData("Left Servo Position", leftPosition);
        telemetry.addData("Right Servo Position", rightPosition);
        telemetry.update();
    }
}
