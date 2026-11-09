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

    public void setPosition(double position) {
        this.position = MathUtil.clamp(
                position,
                TurretConstants.MIN_POSITION,
                TurretConstants.MAX_POSITION
        );

        leftServo.setPosition(this.position);
        rightServo.setPosition(this.position);
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
        telemetry.addData("Turret position", "%.3f", position);
    }
}
