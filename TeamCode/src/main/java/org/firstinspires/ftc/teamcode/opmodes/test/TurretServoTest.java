package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.constants.TurretConstants;

@TeleOp(name = "Test: Turret Servos", group = "Test")
public class TurretServoTest extends DualServoTest {
    public TurretServoTest() {
        super(
                TurretConstants.LEFT_SERVO_NAME,
                TurretConstants.RIGHT_SERVO_NAME,
                0.5,
                0.5,
                0.005
        );
    }
}
