package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;

/**
 * Arm that lowers to collect POLLEN from a FLOWER. Set {@link #down}, then call update()
 * once per loop to move the servo. If no servo with the configured name exists, the arm
 * tracks its state but moves nothing.
 */
public class FlowerArm implements Subsystem {
    private final Servo servo;
    // True: arm lowered to FLOWER_DOWN_POSITION. False: raised to FLOWER_UP_POSITION.
    public boolean down;

    /** Does not move the servo; it first moves on the first call to update(). */
    public FlowerArm(HardwareMap hardwareMap) {
        servo = hardwareMap.tryGet(Servo.class, IntakeConstants.FLOWER_SERVO_NAME);
    }

    /**
     * Send the servo to the position for {@link #down}. Called every loop, so a position
     * changed live in Panels takes effect without toggling the arm.
     */
    public void update() {
        if (servo != null) {
            servo.setPosition(down
                    ? IntakeConstants.FLOWER_DOWN_POSITION
                    : IntakeConstants.FLOWER_UP_POSITION);
        }
    }

    @Override
    public void stop() {
        // The servo holds its last position; nothing to power down.
    }

    @Override
    public void addTelemetry(Telemetry telemetry) {
        if (servo == null) {
            telemetry.addData("Flower arm", "NO SERVO \"%s\" in the configuration",
                    IntakeConstants.FLOWER_SERVO_NAME);
            return;
        }
        telemetry.addData("Flower arm", "%s, servo at %.3f  (up %.3f, down %.3f)",
                down ? "DOWN" : "up", servo.getPosition(),
                IntakeConstants.FLOWER_UP_POSITION, IntakeConstants.FLOWER_DOWN_POSITION);
    }
}
