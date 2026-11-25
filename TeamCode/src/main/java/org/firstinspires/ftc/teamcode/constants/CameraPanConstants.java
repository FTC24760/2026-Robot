package org.firstinspires.ftc.teamcode.constants;

/**
 * Optional servo that turns the Limelight about a vertical axis so it keeps facing our
 * HIVE while the robot drives. Off by default; turn ENABLED on once the servo is mounted
 * and the CALIBRATE values are set. Angles are degrees to the left of robot forward.
 */
public class CameraPanConstants {
    public static boolean ENABLED = false;
    public static String SERVO_NAME = "camerapan";

    // ---- CALIBRATE ----
    // Servo position at which the camera looks along VisionConstants.CAMERA_YAW_DEG.
    public static double CENTER_POSITION = 0.5;
    // Degrees the camera turns between servo position 0 and 1 (270 for a 270 degree servo,
    // or the output angle after any gearing).
    public static double DEGREES_PER_FULL_TRAVEL = 270;
    // True if raising the servo position turns the camera to the left (seen from above).
    // If the camera turns away from the HIVE, flip this.
    public static boolean INCREASING_POSITION_TURNS_LEFT = true;
    // Where the pan axis is relative to the robot centre, metres. The lens swings around
    // this point; the lens position at centre is VisionConstants.CAMERA_FORWARD_M / LEFT_M.
    public static double AXIS_FORWARD_M = VisionConstants.CAMERA_FORWARD_M;
    public static double AXIS_LEFT_M = VisionConstants.CAMERA_LEFT_M;

    // ---- Tracking ----
    // Fastest the command may move, degrees per second. Raise if it lags a fast spin.
    public static double MAX_RATE_DEG_PER_S = 600;
    // Ignore aim changes smaller than this so the servo does not buzz.
    public static double DEADBAND_DEG = 1.0;
    // Aim this far ahead using odometry velocity, to make up for the servo's lag.
    public static double LEAD_S = 0.05;
    // The servo has no feedback. The real angle is taken as the commanded angle this long
    // ago when a camera frame is placed on the field. Tag positions are wrong by roughly
    // (angle error) while the camera is moving, so keep this honest.
    public static double SERVO_LAG_S = 0.1;
}
