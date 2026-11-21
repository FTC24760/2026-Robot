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
    public static double FLOWER_UP_POSITION = 0.59;
    public static double FLOWER_DOWN_POSITION = 0.69;
    public static double SHOOT_FEED_SECONDS = 5.5;
}
