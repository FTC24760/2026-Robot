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

    // Automatic control (Butterfly.update): butterflies drop when idle or driving straight
    // forward. Only DriveTest uses it; TeleOp drops them for tank drive instead.
    public static boolean AUTOMATIC_CONTROL_ENABLED = true;
    public static double FORWARD_ENGAGE_THRESHOLD = 0.08;
    public static double FORWARD_STRAFE_TOLERANCE = 0.10;
    public static double FORWARD_TURN_TOLERANCE = 0.10;
    public static double IDLE_DEADZONE = 0.06;

    // Tank drive (driver right stick Y past ControlsConstants.BUTTERFLY_THRESHOLD):
    // left stick below this magnitude counts as released, and the heading is held.
    public static double TANK_STICK_DEADBAND = 0.1;
    // Scales Pedro's tuned heading controller (pedro/Constants headingFeedback) in tank
    // drive. Pedro's default for heading lock is 0.5.
    public static double TANK_HEADING_STRENGTH = 0.5;
}
