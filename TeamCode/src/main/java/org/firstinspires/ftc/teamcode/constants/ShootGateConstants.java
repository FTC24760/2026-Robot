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

    // ---- Shooting zone and the driver's "ready to fire" signal (TeleOp). ----
    // The zone is the area the shot table is calibrated for: target distance within the
    // measured rows (+ ShotTableConstants.OUT_OF_RANGE_MARGIN_IN). Once in, the robot
    // stays "in zone" until this much further out, so it does not flicker at the edge.
    public static double ZONE_HYSTERESIS_IN = 2;
    // Spin the wheel up to the table speed while in the zone, so a shot goes the moment
    // the button is pressed. Off saves battery; "ready" then ignores the wheel.
    public static boolean PRESPIN_IN_ZONE = true;
    // Rumble the driver and operator gamepads for this long when the robot becomes ready.
    public static int READY_RUMBLE_MS = 250;
    public static double READY_RUMBLE_COOLDOWN_S = 2.0;
    // Gamepad LED (PS4/PS5-style pads only): green when ready, red when not.
    public static boolean READY_LED = true;
}
