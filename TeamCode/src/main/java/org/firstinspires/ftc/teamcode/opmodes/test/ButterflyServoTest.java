package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.constants.ButterflyConstants;

// Probably only needs to move 0.08-0.1 to retract.
@TeleOp(name = "Test: Butterfly Servos", group = "Test")
public class ButterflyServoTest extends DualServoTest {
    public ButterflyServoTest() {
        super(
                ButterflyConstants.LEFT_SERVO_NAME,
                ButterflyConstants.RIGHT_SERVO_NAME,
                1.0,
                0.0,
                0.002
        );
    }
}
