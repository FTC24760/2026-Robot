package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;

/**
 * Arm that lowers to collect POLLEN from a FLOWER. The mechanism is not built yet: if
 * no servo with the configured name exists, the arm tracks its state but moves nothing,
 * so the button and the auto can be wired up now.
 */
public class FlowerArm implements Subsystem {
    private final Servo servo;
    private boolean down;

    /** Does not move the servo; it first moves on the first call to setDown. */
    public FlowerArm(HardwareMap hardwareMap) {
        servo = hardwareMap.tryGet(Servo.class, IntakeConstants.FLOWER_SERVO_NAME);
    }

    public void setDown(boolean down) {
        this.down = down;
        if (servo != null) {
            servo.setPosition(down
                    ? IntakeConstants.FLOWER_DOWN_POSITION
                    : IntakeConstants.FLOWER_UP_POSITION);
        }
    }

    public void toggle() {
        setDown(!down);
    }

    public boolean isDown() {
        return down;
    }

    public boolean isInstalled() {
        return servo != null;
    }

    @Override
    public void stop() {
        // The servo holds its last position; nothing to power down.
    }

    @Override
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Flower arm", "%s%s", down ? "DOWN" : "up",
                servo == null ? " (no servo configured)" : "");
    }
}
