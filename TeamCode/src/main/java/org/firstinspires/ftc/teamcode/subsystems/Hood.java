package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.ShotConstants;
import org.firstinspires.ftc.teamcode.utils.MathUtil;

/**
 * Shooter hood: a left and a right servo that set the launch angle together. Both get
 * the same position, plus each one's offset, with direction set in ShotConstants.
 *
 * A servo missing from the robot configuration is skipped. With neither present every
 * call does nothing, so auto-aim still works on a fixed-hood robot (set the two hood
 * angles in ShotConstants to the same value in that case).
 */
public class Hood implements Subsystem {
    private final Servo leftServo;
    private final Servo rightServo;
    private double position = Double.NaN;

    /** Does not move the servos; they first move on the first call to setPosition. */
    public Hood(HardwareMap hardwareMap) {
        leftServo = hardwareMap.tryGet(Servo.class, ShotConstants.HOOD_LEFT_SERVO_NAME);
        rightServo = hardwareMap.tryGet(Servo.class, ShotConstants.HOOD_RIGHT_SERVO_NAME);
    }

    public void setPosition(double position) {
        double low = Math.min(ShotConstants.HOOD_DOWN_POSITION, ShotConstants.HOOD_UP_POSITION);
        double high = Math.max(ShotConstants.HOOD_DOWN_POSITION, ShotConstants.HOOD_UP_POSITION);

        this.position = MathUtil.clamp(position, low, high);
        drive(leftServo, ShotConstants.HOOD_LEFT_REVERSED, ShotConstants.HOOD_LEFT_OFFSET);
        drive(rightServo, ShotConstants.HOOD_RIGHT_REVERSED, ShotConstants.HOOD_RIGHT_OFFSET);
    }

    public void setDown() {
        setPosition(ShotConstants.HOOD_DOWN_POSITION);
    }

    /** True if at least one hood servo is configured. */
    public boolean isInstalled() {
        return leftServo != null || rightServo != null;
    }

    public double getPosition() {
        return position;
    }

    // Direction is set every call so a change made live in Panels takes effect.
    private void drive(Servo servo, boolean reversed, double offset) {
        if (servo == null) {
            return;
        }
        servo.setDirection(reversed ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
        servo.setPosition(MathUtil.clamp(position + offset, 0, 1));
    }

    @Override
    public void stop() {
        // The servos hold their last position; nothing to power down.
    }

    @Override
    public void addTelemetry(Telemetry telemetry) {
        if (!isInstalled()) {
            telemetry.addData("Hood", "no servos \"%s\" / \"%s\" configured",
                    ShotConstants.HOOD_LEFT_SERVO_NAME, ShotConstants.HOOD_RIGHT_SERVO_NAME);
            return;
        }

        telemetry.addData("Hood position", "%.3f", position);
        if (leftServo == null || rightServo == null) {
            telemetry.addData("Hood", "only one servo configured (missing \"%s\")",
                    leftServo == null ? ShotConstants.HOOD_LEFT_SERVO_NAME
                            : ShotConstants.HOOD_RIGHT_SERVO_NAME);
        }
    }
}
