package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.TurretConstants;
import org.firstinspires.ftc.teamcode.utils.MathUtil;

/**
 * Turret aimed by two servos that are always commanded to the same position.
 * Angles are radians to the left of robot forward. The servo range is set in degrees
 * (TurretConstants.DEGREES_PER_FULL_TRAVEL and the deadbands) and converted here.
 */
public class Turret implements Subsystem {
    private final Servo leftServo;
    private final Servo rightServo;

    private double position = TurretConstants.START_POSITION;
    private double feedforward = 0;

    /** Does not move the servos; they first move on the first call to {@link #move} or {@link #pointAt}. */
    public Turret(HardwareMap hardwareMap) {
        leftServo = hardwareMap.get(Servo.class, TurretConstants.LEFT_SERVO_NAME);
        rightServo = hardwareMap.get(Servo.class, TurretConstants.RIGHT_SERVO_NAME);
    }

    public void setFeedforward(double angularVelocity) {
        feedforward = angularVelocity * TurretConstants.kF;
    }

    /**
     * Manual aiming at up to TurretConstants.MANUAL_RATE radians per second. Call every
     * loop, including with direction 0 to hold position.
     *
     * @param direction   -1 to 1; positive turns left
     * @param loopSeconds time since the previous loop, so speed does not depend on loop rate
     */
    public void move(double direction, double loopSeconds) {
        setPosition(position + radiansToPosition(direction * TurretConstants.MANUAL_RATE * loopSeconds));
    }

    /**
     * Auto-align from a camera fixed to the chassis: turn to the servo position that
     * points the turret at a bearing measured from robot forward. Call every loop.
     *
     * @param bearingRad  direction to the target, radians to the left of robot forward
     * @param loopSeconds time since the previous loop
     */
    public void pointAt(double bearingRad, double loopSeconds) {
        // Ignore tiny changes: chasing every bit of camera and odometry noise makes the
        // servo buzz back and forth.
        double errorRad = bearingRad - getAngle();
        if (Math.abs(errorRad) < Math.toRadians(TurretConstants.AIM_DEADBAND_DEG)) {
            setPosition(position);
            return;
        }

        double maxStep = TurretConstants.AUTO_MAX_RATE * loopSeconds;
        setPosition(position + MathUtil.clamp(radiansToPosition(errorRad), -maxStep, maxStep));
    }

    /**
     * Auto-align from a camera that rides on the turret: nudge the turret toward
     * putting the target dead ahead of the camera. Call every loop.
     *
     * @param errorRad    direction to the target, radians to the left of where the
     *                    turret is pointing now
     * @param loopSeconds time since the previous loop
     */
    public void track(double errorRad, double loopSeconds) {
        if (Math.abs(errorRad) < Math.toRadians(TurretConstants.DEADBAND_DEG)) {
            setPosition(position);
            return;
        }

        double step = radiansToPosition(errorRad) * TurretConstants.TRACKING_GAIN * loopSeconds;
        double maxStep = TurretConstants.AUTO_MAX_RATE * loopSeconds;
        setPosition(position + MathUtil.clamp(step, -maxStep, maxStep));
    }

    /** Where the turret is pointing, in radians to the left of robot forward. */
    public double getAngle() {
        return positionToRadians(position - TurretConstants.FORWARD_POSITION);
    }

    // The constants give the servo's range in degrees, because that is how it is
    // measured; everything in this class works in radians. Read each loop rather than
    // cached, so edits from Panels take effect immediately.
    private static double radiansPerFullTravel() {
        double sign = TurretConstants.INCREASING_POSITION_TURNS_LEFT ? 1 : -1;
        return sign * Math.toRadians(TurretConstants.DEGREES_PER_FULL_TRAVEL);
    }

    // A turn to the left, in radians, as a change in servo position.
    private static double radiansToPosition(double radiansLeft) {
        return radiansLeft / radiansPerFullTravel();
    }

    // A change in servo position as a turn to the left, in radians.
    private static double positionToRadians(double positionChange) {
        return positionChange * radiansPerFullTravel();
    }

    public void setPosition(double position) {
        this.position = MathUtil.clamp(
                position,
                TurretConstants.MIN_POSITION,
                TurretConstants.MAX_POSITION
        );

        leftServo.setPosition(this.position + TurretConstants.LEFT_SERVO_OFFSET + feedforward);
        rightServo.setPosition(this.position + TurretConstants.RIGHT_SERVO_OFFSET + feedforward);
    }

    public double getPosition() {
        return position;
    }
    @Override
    public void stop() {
        // Servos hold their last position; nothing to power down.
    }
    @Override
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Turret", "%.3f (%.0f deg)", position, Math.toDegrees(getAngle()));
    }
}
