package org.firstinspires.ftc.teamcode.constants;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class TurretConstants {
    public static String LEFT_SERVO_NAME = "leftturret";
    public static String RIGHT_SERVO_NAME = "rightturret";

    public static double START_POSITION = 0.5;
    public static double MIN_POSITION = 0.0;
    public static double MAX_POSITION = 1.0;
    // Manual aiming speed, in servo position per second.
    public static double MANUAL_RATE = 0.25;
}
