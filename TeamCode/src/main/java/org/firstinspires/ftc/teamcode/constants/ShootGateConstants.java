package org.firstinspires.ftc.teamcode.constants;

import com.bylazar.configurables.annotations.Configurable;

/**
 * The fire gate: a ball is only fed when the wheel is at speed (Shooter) AND the aim is
 * ready (these). Applies to TeleOp and Auto. Tune on the robot; raise the limits if good
 * shots are held back, lower them if shots taken while moving miss.
 */
@Configurable
public class ShootGateConstants {
    // Off: only the wheel speed gates feeding (the competition behaviour).
    public static boolean ENABLED = true;
    // While the turret auto-aims, it must have been on target (within
    // TurretConstants.AIM_DEADBAND_DEG of where it should point) for this long. The
    // servos have no feedback, so this also covers their travel lag.
    public static double AIM_SETTLE_S = 0.15;
    // The robot must be moving slower than this (inches per second) and turning slower
    // than MAX_TURN_DEG_PER_S. PLACEHOLDERS: measure what still scores.
    public static double MAX_SPEED_IN_PER_S = 15;
    public static double MAX_TURN_DEG_PER_S = 60;
}
