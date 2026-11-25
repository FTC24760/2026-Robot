package org.firstinspires.ftc.teamcode.constants;


/**
 * The operator's manual aiming (gamepad 2, see OperatorControls). Turret and hood stay
 * inside their own limits: TurretConstants.MIN/MAX_POSITION and
 * ShotConstants.HOOD_DOWN/UP_POSITION.
 */
public class OperatorConstants {
    // Stick input smaller than this is ignored.
    public static double STICK_DEADZONE = 0.1;
    // Turret speed at full stick, in servo position per second.
    public static double TURRET_RATE = 0.4;
    // Hood speed while a d-pad button is held, in servo position per second.
    public static double HOOD_RATE = 0.15;
    // Wheel speed change per press.
    public static double RPM_STEP = 50;
    // The rpm up/down buttons trim the shot (table or fixed) by at most this much.
    public static double MAX_RPM_TRIM = 500;
    public static double MIN_RPM = 500;
    public static double MAX_RPM = 4000;
}
