package org.firstinspires.ftc.teamcode.constants;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class TurretConstants {
    public static String LEFT_SERVO_NAME = "leftturret";
    public static String RIGHT_SERVO_NAME = "rightturret";
    public static double kF;
    public static double LEFT_SERVO_OFFSET = 0.0;
    public static double RIGHT_SERVO_OFFSET = 0.0;

    public static double START_POSITION = 0.5;
    public static double MIN_POSITION = 0.0;
    public static double MAX_POSITION = 1.0;
    // Manual aiming speed, in servo position per second.
    public static double MANUAL_RATE = 0.25;

    // ---- Auto-align with the Limelight. CALIBRATE all four. ----
    // Servo position at which the turret points straight ahead on the robot.
    public static double FORWARD_POSITION = 0.5;
    // How many degrees the turret turns between servo position 0 and position 1.
    public static double DEGREES_PER_FULL_TRAVEL = 300;
    // True if raising the servo position turns the turret to the left (seen from above).
    // If auto-align turns away from the HIVE, flip this.
    public static boolean INCREASING_POSITION_TURNS_LEFT = true;
    // Fastest the turret may move while auto-aligning, in servo position per second.
    // Raised from 1.0 so the command is never slower than the servo itself.
    public static double AUTO_MAX_RATE = 3.0;
    // Aim this far ahead of where the robot is now, using odometry velocity, to make up
    // for the servo's own lag while driving or turning. 0 turns it off. If the turret
    // overshoots when the robot stops turning, lower it.
    // Set to 0 first if the turret jitters, to see whether the lead is the cause.
    public static double AIM_LEAD_SECONDS = 0.03;
    // Auto-aim leaves the turret alone until the aim point moves more than this, so it
    // does not buzz on noise. Raise it if it still jitters; lower it if aim is too coarse.
    public static double AIM_DEADBAND_DEG = 0.75;
    // Where the turret's pivot is relative to the robot centre, metres. CALIBRATE.
    // Matters most up close.
    public static double PIVOT_FORWARD_M = 0.0;
    public static double PIVOT_LEFT_M = 0.0;
    // Only used by Turret.track() (closed-loop on camera error), which auto-aim no longer
    // calls: aim now comes from odometry plus the HIVE's remembered field position.
    public static double TRACKING_GAIN = 4.0;
    public static double DEADBAND_DEG = 1.0;
}
