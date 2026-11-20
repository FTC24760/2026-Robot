package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.constants.ShotConstants;

/**
 * Both hood servos move together; see LinkedServoTest for the controls.
 * First check both sides turn the same way (if not, flip HOOD_RIGHT_REVERSED), then find
 * the positions for HOOD_DOWN_POSITION and HOOD_UP_POSITION.
 */
@TeleOp(name = "Test: Hood Servos", group = "Test")
public class HoodServoTest extends LinkedServoTest {
    public HoodServoTest() {
        super(
                ShotConstants.HOOD_LEFT_SERVO_NAME,
                ShotConstants.HOOD_RIGHT_SERVO_NAME,
                ShotConstants.HOOD_DOWN_POSITION,
                0.005
        );
    }

    @Override
    protected double leftOffset() {
        return ShotConstants.HOOD_LEFT_OFFSET;
    }

    @Override
    protected double rightOffset() {
        return ShotConstants.HOOD_RIGHT_OFFSET;
    }

    @Override
    protected boolean leftReversed() {
        return ShotConstants.HOOD_LEFT_REVERSED;
    }

    @Override
    protected boolean rightReversed() {
        return ShotConstants.HOOD_RIGHT_REVERSED;
    }
}
