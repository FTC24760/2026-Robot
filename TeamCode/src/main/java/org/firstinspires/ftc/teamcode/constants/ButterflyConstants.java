package org.firstinspires.ftc.teamcode.constants;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class ButterflyConstants {
    public static String LEFT_SERVO_NAME = "leftButterfly";
    public static String RIGHT_SERVO_NAME = "rightButterfly";

    public static double LEFT_UP_POSITION = 0.20;
    public static double LEFT_DOWN_POSITION = 0.80;
    public static double RIGHT_UP_POSITION = 0.80;
    public static double RIGHT_DOWN_POSITION = 0.20;

    // Automatic control: butterflies drop when idle or driving straight forward.
    public static boolean AUTOMATIC_CONTROL_ENABLED = true;
    public static double FORWARD_ENGAGE_THRESHOLD = 0.08;
    public static double FORWARD_STRAFE_TOLERANCE = 0.10;
    public static double FORWARD_TURN_TOLERANCE = 0.10;
    public static double IDLE_DEADZONE = 0.06;
}
