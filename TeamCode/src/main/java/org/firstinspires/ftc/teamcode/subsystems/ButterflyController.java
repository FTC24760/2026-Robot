/*
Author: OJB
Date: 2026-07-23
*/
package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@Configurable
public class ButterflyController {

    public static String LEFT_SERVO_NAME = "leftButterfly";
    public static String RIGHT_SERVO_NAME = "rightButterfly";

    public static double LEFT_UP_POSITION = 0.20;
    public static double LEFT_DOWN_POSITION = 0.80;
    public static double RIGHT_UP_POSITION = 0.80;
    public static double RIGHT_DOWN_POSITION = 0.20;

    public static double FORWARD_ENGAGE_THRESHOLD = 0.08;
    public static double FORWARD_STRAFE_TOLERANCE = 0.10;
    public static double FORWARD_TURN_TOLERANCE = 0.10;
    public static double IDLE_DEADZONE = 0.06;

    public static boolean AUTOMATIC_CONTROL_ENABLED = true;

    public enum State {
        UP,
        DOWN
    }

    private final Servo leftServo;
    private final Servo rightServo;
    private State state;

    public ButterflyController(HardwareMap hardwareMap) {
        this(hardwareMap, LEFT_SERVO_NAME, RIGHT_SERVO_NAME);
    }

    public ButterflyController(
            HardwareMap hardwareMap,
            String leftServoName,
            String rightServoName
    ) {
        leftServo = hardwareMap.get(Servo.class, leftServoName);
        rightServo = hardwareMap.get(Servo.class, rightServoName);
        setUp();
    }

    public void update(double forward, double strafe, double turn) {
        update(forward, strafe, turn, false);
    }

    public void update(
            double forward,
            double strafe,
            double turn,
            boolean manualInvert
    ) {
        boolean shouldBeDown;

        if (!AUTOMATIC_CONTROL_ENABLED) {
            shouldBeDown = false;
        } else {
            boolean noStickInput =
                    Math.abs(forward) <= IDLE_DEADZONE
                            && Math.abs(strafe) <= IDLE_DEADZONE
                            && Math.abs(turn) <= IDLE_DEADZONE;

            boolean forwardOnly =
                    forward >= FORWARD_ENGAGE_THRESHOLD
                            && Math.abs(strafe) <= FORWARD_STRAFE_TOLERANCE
                            && Math.abs(turn) <= FORWARD_TURN_TOLERANCE;

            shouldBeDown = noStickInput || forwardOnly;
        }

        setDown(manualInvert ? !shouldBeDown : shouldBeDown);
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
            leftServo.setPosition(
                    clampServoPosition(LEFT_DOWN_POSITION)
            );
            rightServo.setPosition(
                    clampServoPosition(RIGHT_DOWN_POSITION)
            );
        } else {
            leftServo.setPosition(
                    clampServoPosition(LEFT_UP_POSITION)
            );
            rightServo.setPosition(
                    clampServoPosition(RIGHT_UP_POSITION)
            );
        }

        state = requestedState;
    }

    public State getState() {
        return state;
    }

    public boolean isDown() {
        return state == State.DOWN;
    }

    private static double clampServoPosition(double position) {
        return Range.clip(position, 0.0, 1.0);
    }
}