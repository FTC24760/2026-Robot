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

    // ---- Auto-align with the Limelight. CALIBRATE all four. ----
    // Servo position at which the turret points straight ahead on the robot.
    public static double FORWARD_POSITION = 0.5;
    // How many degrees the turret turns between servo position 0 and position 1.
    public static double DEGREES_PER_FULL_TRAVEL = 180.0;
    // True if raising the servo position turns the turret to the left (seen from above).
    // If auto-align turns away from the HIVE, flip this.
    public static boolean INCREASING_POSITION_TURNS_LEFT = true;
    // Fastest the turret may move while auto-aligning, in servo position per second.
    public static double AUTO_MAX_RATE = 1.0;
    // Only used when the camera rides on the turret: fraction of the remaining error
    // corrected per second. Lower it if the turret hunts back and forth.
    public static double TRACKING_GAIN = 4.0;
    // Stop correcting inside this error, so the turret does not twitch on noise.
    public static double DEADBAND_DEG = 1.0;
}
