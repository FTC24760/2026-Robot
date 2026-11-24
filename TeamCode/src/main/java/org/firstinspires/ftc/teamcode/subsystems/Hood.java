package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.ShotConstants;
import org.firstinspires.ftc.teamcode.utils.MathUtil;

/**
 * Shooter hood: one servo that sets the launch angle.
 *
 * If the servo is missing from the robot configuration every call does nothing and
 * telemetry says so, so auto-aim still works on a fixed hood.
 */
public class Hood implements Subsystem {
    private final Servo servo;
    private double position = Double.NaN;

    /** Does not move the servo; it first moves on the first call to setPosition. */
    public Hood(HardwareMap hardwareMap) {
        servo = hardwareMap.tryGet(Servo.class, ShotConstants.HOOD_SERVO_NAME);
    }

    public void setPosition(double position) {
        double low = Math.min(ShotConstants.HOOD_DOWN_POSITION, ShotConstants.HOOD_UP_POSITION);
        double high = Math.max(ShotConstants.HOOD_DOWN_POSITION, ShotConstants.HOOD_UP_POSITION);
        this.position = MathUtil.clamp(position, low, high);

        if (servo != null) {
            // Direction is set every call so a change made live in Panels takes effect.
            servo.setDirection(ShotConstants.HOOD_REVERSED
                    ? Servo.Direction.REVERSE
                    : Servo.Direction.FORWARD);
            servo.setPosition(MathUtil.clamp(this.position + ShotConstants.HOOD_OFFSET, 0, 1));
        }
    }

    public void setDown() {
        setPosition(ShotConstants.HOOD_DOWN_POSITION);
    }

    public boolean isInstalled() {
        return servo != null;
    }

    public double getPosition() {
        return position;
    }
    @Override
    public void stop() {
        // The servo holds its last position; nothing to power down.
    }
    @Override
    public void addTelemetry(Telemetry telemetry) {
        if (servo == null) {
            telemetry.addData("Hood", "NO SERVO \"%s\" in the configuration", ShotConstants.HOOD_SERVO_NAME);
        } else {
            telemetry.addData("Hood position", "%.3f", position);
        }
    }
}
