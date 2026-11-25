package org.firstinspires.ftc.teamcode.constants;

import com.bylazar.configurables.annotations.Configurable;

/** Match settings exposed in Panels. Consumers read these values every loop.
 * Servo positions are 0..1; motor powers are 0..1; shooter speed is RPM.
 * Hardware names, PID gains, limits and autonomous geometry remain in constants/.
 */
@Configurable
public class Configurables {
    public static double SHOOTER_RPM = 3350;
    public static double HOOD_POSITION = 0.95;
    public static double INTAKE_POWER = 1.0;
    public static double TRANSFER_POWER = 1.0;
    public static double OUTTAKE_POWER = 0.6;
    public static double BLOCKER_OPEN_POSITION = 0.032;
    public static double BLOCKER_CLOSED_POSITION = 0.0;
    public static double FLOWER_UP_POSITION = 0.9184;
    public static double FLOWER_DOWN_POSITION = 0.7909;
    public static double SHOOT_FEED_SECONDS = 5.5;
    // Start opening the (slow) blocker when a shot starts spinning up instead of after
    // the wheel is ready, so its travel overlaps spin-up (saves up to
    // ShooterConstants.BLOCKER_OPEN_SETTLE_S per shot). The transfer stays off until
    // the wheel is ready either way. Turn off if a ball rolls into the wheel on its own
    // when the gate opens.
    public static boolean BLOCKER_OPEN_EARLY = true;
    // Pulse shoot (operator X toggles it): push for PUSH, pull back for RETRACT, repeat
    // for PULSE_BALLS balls, then keep pushing.
    public static double PULSE_PUSH_SECONDS = 0.18;
    public static double PULSE_RETRACT_SECONDS = 0.18;
    public static double PULSE_RETRACT_POWER = 0.5;
    public static int PULSE_BALLS = 4;
}
