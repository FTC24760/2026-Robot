package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.ShotConstants;
import org.firstinspires.ftc.teamcode.utils.MathUtil;

/** Shooter hood: a servo that sets the launch angle. */
public class Hood implements Subsystem {
    private final Servo servo;
    private double position = Double.NaN;

    /** Does not move the servo; it first moves on the first call to setPosition. */
    public Hood(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, ShotConstants.HOOD_SERVO_NAME);
    }

    public void setPosition(double position) {
        double low = Math.min(ShotConstants.HOOD_DOWN_POSITION, ShotConstants.HOOD_UP_POSITION);
        double high = Math.max(ShotConstants.HOOD_DOWN_POSITION, ShotConstants.HOOD_UP_POSITION);

        this.position = MathUtil.clamp(position, low, high);
        servo.setPosition(this.position);
    }

    public void setDown() {
        setPosition(ShotConstants.HOOD_DOWN_POSITION);
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
        telemetry.addData("Hood position", "%.3f", position);
    }
}
