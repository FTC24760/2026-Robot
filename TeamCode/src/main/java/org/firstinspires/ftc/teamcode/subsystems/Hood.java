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
    private final Servo leftHood;
    private final Servo rightHood;
    private double position = Double.NaN;

    /** Does not move the servos; they first move on the first call to setPosition. */
    public Hood(HardwareMap hardwareMap) {
        leftHood = hardwareMap.tryGet(Servo.class, ShotConstants.HOOD_LEFT_SERVO_NAME);
        rightHood = hardwareMap.tryGet(Servo.class, ShotConstants.HOOD_RIGHT_SERVO_NAME);
    }

    public void setPosition(double position) {
        double low = Math.min(ShotConstants.HOOD_DOWN_POSITION, ShotConstants.HOOD_UP_POSITION);
        double high = Math.max(ShotConstants.HOOD_DOWN_POSITION, ShotConstants.HOOD_UP_POSITION);

        this.position = MathUtil.clamp(position, low, high);
        if (leftHood != null) {
            leftHood.setPosition(ShotConstants.LEFT_HOOD_OFFSET - this.position);
        }
        if (rightHood != null) {
            rightHood.setPosition(ShotConstants.RIGHT_HOOD_OFFSET + this.position);
        }
    }

    public void setDown() {
        setPosition(ShotConstants.HOOD_DOWN_POSITION);
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
        telemetry.addData("Hood position", "%.3f", position);
    }
}
