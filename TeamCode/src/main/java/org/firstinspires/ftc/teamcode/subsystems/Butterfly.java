/*
Author: OJB
Date: 2026-07-23
*/
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.ButterflyConstants;

/** Butterfly modules: servos that raise or lower the drive pods. */
public class Butterfly implements Subsystem {

    public enum State {
        UP,
        DOWN
    }

    private final Servo leftServo;
    private final Servo rightServo;
    private State state;

    public Butterfly(HardwareMap hardwareMap) {
        leftServo = hardwareMap.get(Servo.class, ButterflyConstants.LEFT_SERVO_NAME);
        rightServo = hardwareMap.get(Servo.class, ButterflyConstants.RIGHT_SERVO_NAME);
        setUp();
    }

    /**
     * Automatic control from the drive inputs: down when idle or driving straight
     * forward, up otherwise. manualInvert flips whatever the automatic choice is.
     */
    public void update(double forward, double strafe, double turn, boolean manualInvert) {
        boolean shouldBeDown;

        if (!ButterflyConstants.AUTOMATIC_CONTROL_ENABLED) {
            shouldBeDown = false;
        } else {
            boolean noStickInput =
                    Math.abs(forward) <= ButterflyConstants.IDLE_DEADZONE
                            && Math.abs(strafe) <= ButterflyConstants.IDLE_DEADZONE
                            && Math.abs(turn) <= ButterflyConstants.IDLE_DEADZONE;

            boolean forwardOnly =
                    forward >= ButterflyConstants.FORWARD_ENGAGE_THRESHOLD
                            && Math.abs(strafe) <= ButterflyConstants.FORWARD_STRAFE_TOLERANCE
                            && Math.abs(turn) <= ButterflyConstants.FORWARD_TURN_TOLERANCE;

            shouldBeDown = noStickInput || forwardOnly;
        }

        setDown(manualInvert != shouldBeDown);
    }

    public void setDown() {
        setDown(true);
    }

    public void setUp() {
        setDown(false);
    }

    public void setDown(boolean down) {
        State requestedState = down ? State.DOWN : State.UP;

        if (state == requestedState) {
            return;
        }

        if (down) {
            leftServo.setPosition(clip(ButterflyConstants.LEFT_DOWN_POSITION));
            rightServo.setPosition(clip(ButterflyConstants.RIGHT_DOWN_POSITION));
        } else {
            leftServo.setPosition(clip(ButterflyConstants.LEFT_UP_POSITION));
            rightServo.setPosition(clip(ButterflyConstants.RIGHT_UP_POSITION));
        }

        state = requestedState;
    }

    public State getState() {
        return state;
    }

    public boolean isDown() {
        return state == State.DOWN;
    }

    @Override
    public void stop() {
        setUp();
    }

    @Override
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Butterfly", state);
    }

    private static double clip(double position) {
        return Range.clip(position, 0.0, 1.0);
    }
}
