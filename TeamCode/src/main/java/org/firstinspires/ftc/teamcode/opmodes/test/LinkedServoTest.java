package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Base for testing a pair of servos that the robot always moves together (turret, hood).
 * Each servo gets the same position plus its own offset and direction from the constants,
 * exactly as the subsystem drives them, so what you see here is what the robot does.
 *
 *   right trigger (hold)  both servos up
 *   left trigger (hold)   both servos down
 *   d-pad up / down       trim the right servo only, to line the two sides up
 *   A                     clear the trim
 *
 * Copy "position" into the matching constants file, and the suggested right offset into
 * its RIGHT offset if you trimmed.
 */
public abstract class LinkedServoTest extends OpMode {
    private final String leftName;
    private final String rightName;
    private final double step;
    private final double trimStep;

    private Servo leftServo;
    private Servo rightServo;
    private double position;
    private double rightTrim;

    /**
     * @param start position both servos start at
     * @param step  position change per loop while a trigger is held
     */
    protected LinkedServoTest(String leftName, String rightName, double start, double step) {
        this.leftName = leftName;
        this.rightName = rightName;
        this.position = start;
        this.step = step;
        this.trimStep = step / 2;
    }

    protected abstract double leftOffset();

    protected abstract double rightOffset();

    protected boolean leftReversed() {
        return false;
    }

    protected boolean rightReversed() {
        return false;
    }

    @Override
    public void init() {
        leftServo = hardwareMap.get(Servo.class, leftName);
        rightServo = hardwareMap.get(Servo.class, rightName);
    }

    @Override
    public void loop() {
        if (gamepad1.right_trigger > 0.2) {
            position = Math.min(1.0, position + step);
        } else if (gamepad1.left_trigger > 0.2) {
            position = Math.max(0.0, position - step);
        }

        if (gamepad1.dpad_up) {
            rightTrim += trimStep;
        } else if (gamepad1.dpad_down) {
            rightTrim -= trimStep;
        }
        if (gamepad1.a) {
            rightTrim = 0;
        }

        double left = clamp(position + leftOffset());
        double right = clamp(position + rightOffset() + rightTrim);

        leftServo.setDirection(leftReversed() ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
        rightServo.setDirection(rightReversed() ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
        leftServo.setPosition(left);
        rightServo.setPosition(right);

        telemetry.addLine("Triggers: both up/down · D-pad: trim right · A: clear trim");
        telemetry.addData("Position", "%.3f", position);
        telemetry.addData("Left servo", "%.3f%s", left, leftReversed() ? " (reversed)" : "");
        telemetry.addData("Right servo", "%.3f%s", right, rightReversed() ? " (reversed)" : "");
        if (rightTrim != 0) {
            telemetry.addData("Suggested right offset", "%.3f", rightOffset() + rightTrim);
        }
        telemetry.update();
    }

    private static double clamp(double value) {
        return Math.max(0.0, Math.min(1.0, value));
    }
}
