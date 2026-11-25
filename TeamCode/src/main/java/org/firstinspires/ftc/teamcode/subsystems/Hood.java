package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.ShootGateConstants;
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
    // The position at the last real move, and when that move was commanded. The servo has
    // no feedback, so "settled" is a hold time since the last move.
    private double movedFrom = Double.NaN;
    private long movedAtNs = 0;

    /** Does not move the servo; it first moves on the first call to setPosition. */
    public Hood(HardwareMap hardwareMap) {
        servo = hardwareMap.tryGet(Servo.class, ShotConstants.HOOD_SERVO_NAME);
    }

    public void setPosition(double position) {
        double low = Math.min(ShotConstants.HOOD_DOWN_POSITION, ShotConstants.HOOD_UP_POSITION);
        double high = Math.max(ShotConstants.HOOD_DOWN_POSITION, ShotConstants.HOOD_UP_POSITION);
        this.position = MathUtil.clamp(position, low, high);
        if (Double.isNaN(movedFrom)
                || Math.abs(this.position - movedFrom) > ShootGateConstants.HOOD_SETTLE_DEADBAND) {
            movedFrom = this.position;
            movedAtNs = System.nanoTime();
        }

        if (servo != null) {
            // Direction is set every call so a change made live in Panels takes effect.
            Servo.Direction direction = ShotConstants.HOOD_REVERSED
                    ? Servo.Direction.REVERSE : Servo.Direction.FORWARD;
            double command = MathUtil.clamp(this.position + ShotConstants.HOOD_OFFSET, 0, 1);
            servo.setDirection(direction);
            servo.setPosition(command);
        }
    }

    /**
     * True once the hood has been commanded to (about) the same place for {@code seconds},
     * standing in for the servo's travel time. Always true with no servo installed.
     */
    public boolean isSettled(double seconds) {
        if (servo == null) {
            return true;
        }
        return !Double.isNaN(movedFrom) && (System.nanoTime() - movedAtNs) * 1e-9 >= seconds;
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
