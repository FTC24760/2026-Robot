package org.firstinspires.ftc.teamcode.constants;

import com.bylazar.configurables.annotations.Configurable;

/**
 * The operator's manual aiming (gamepad 2, see OperatorControls). Turret and hood stay
 * inside their own limits: TurretConstants.MIN/MAX_POSITION and
 * HoodConstants.HOOD_DOWN/UP_POSITION.
 */
@Configurable
public class OperatorConstants {
    // Stick input smaller than this is ignored.
    public static double STICK_DEADZONE = 0.1;
    // Turret speed at full stick, in servo position per second.
    public static double TURRET_RATE = 0.4;
    // Hood speed while a d-pad button is held, in servo position per second.
    public static double HOOD_RATE = 0.15;
    // Wheel speed change per press.
    public static double RPM_STEP = 50;
    public static double MIN_RPM = 500;
    public static double MAX_RPM = 4000;
}
