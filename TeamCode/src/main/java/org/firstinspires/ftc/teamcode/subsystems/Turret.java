package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.TurretConstants;
import org.firstinspires.ftc.teamcode.utils.MathUtil;

/** Turret aimed by two servos that are always commanded to the same position. */
public class Turret implements Subsystem {
    private final Servo leftServo;
    private final Servo rightServo;

    private double position = TurretConstants.START_POSITION;

    /** Does not move the servos; they first move on the first call to {@link #move}. */
    public Turret(HardwareMap hardwareMap) {
        leftServo = hardwareMap.get(Servo.class, TurretConstants.LEFT_SERVO_NAME);
        rightServo = hardwareMap.get(Servo.class, TurretConstants.RIGHT_SERVO_NAME);
    }

    /**
     * Manual aiming. Call every loop, including with direction 0 to hold position.
     *
     * @param direction   +1, -1, or 0
     * @param loopSeconds time since the previous loop, so speed does not depend on loop rate
     */
    public void move(double direction, double loopSeconds) {
        setPosition(position + direction * TurretConstants.MANUAL_RATE * loopSeconds);
    }

    /**
     * Auto-align from a camera fixed to the chassis: turn to the servo position that
     * points the turret at a bearing measured from robot forward. Call every loop.
     *
     * @param bearingDeg  direction to the target, degrees to the left of robot forward
     * @param loopSeconds time since the previous loop
     */
    public void pointAt(double bearingDeg, double loopSeconds) {
        double wanted = TurretConstants.FORWARD_POSITION + degreesToPosition(bearingDeg);
        double maxStep = TurretConstants.AUTO_MAX_RATE * loopSeconds;
        setPosition(position + MathUtil.clamp(wanted - position, -maxStep, maxStep));
    }

    /**
     * Auto-align from a camera that rides on the turret: nudge the turret toward
     * putting the target dead ahead of the camera. Call every loop.
     *
     * @param errorDeg    direction to the target, degrees to the left of where the
     *                    turret is pointing now
     * @param loopSeconds time since the previous loop
     */
    public void track(double errorDeg, double loopSeconds) {
        if (Math.abs(errorDeg) < TurretConstants.DEADBAND_DEG) {
            setPosition(position);
            return;
        }

        double step = degreesToPosition(errorDeg) * TurretConstants.TRACKING_GAIN * loopSeconds;
        double maxStep = TurretConstants.AUTO_MAX_RATE * loopSeconds;
        setPosition(position + MathUtil.clamp(step, -maxStep, maxStep));
    }

    /** Where the turret is pointing, in degrees to the left of robot forward. */
    public double getAngleDeg() {
        double sign = TurretConstants.INCREASING_POSITION_TURNS_LEFT ? 1 : -1;
        return (position - TurretConstants.FORWARD_POSITION)
                * TurretConstants.DEGREES_PER_FULL_TRAVEL * sign;
    }

    // A turn to the left, in degrees, as a change in servo position.
    private static double degreesToPosition(double degreesLeft) {
        double sign = TurretConstants.INCREASING_POSITION_TURNS_LEFT ? 1 : -1;
        return sign * degreesLeft / TurretConstants.DEGREES_PER_FULL_TRAVEL;
    }

    public void setPosition(double position) {
        this.position = MathUtil.clamp(
                position,
                TurretConstants.MIN_POSITION,
                TurretConstants.MAX_POSITION
        );

        leftServo.setPosition(this.position + TurretConstants.LEFT_SERVO_OFFSET);
        rightServo.setPosition(this.position + TurretConstants.RIGHT_SERVO_OFFSET);
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
        telemetry.addData("Turret", "%.3f (%.0f deg)", position, getAngleDeg());
    }
}
