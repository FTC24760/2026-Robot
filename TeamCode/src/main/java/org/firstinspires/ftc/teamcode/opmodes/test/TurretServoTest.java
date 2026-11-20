package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.constants.TurretConstants;

/** Both turret servos move together; see LinkedServoTest for the controls. */
@TeleOp(name = "Test: Turret Servos", group = "Test")
public class TurretServoTest extends LinkedServoTest {
    public TurretServoTest() {
        super(
                TurretConstants.LEFT_SERVO_NAME,
                TurretConstants.RIGHT_SERVO_NAME,
                TurretConstants.START_POSITION,
                0.005
        );
    }

    @Override
    protected double leftOffset() {
        return TurretConstants.LEFT_SERVO_OFFSET;
    }

    @Override
    protected double rightOffset() {
        return TurretConstants.RIGHT_SERVO_OFFSET;
    }
}
