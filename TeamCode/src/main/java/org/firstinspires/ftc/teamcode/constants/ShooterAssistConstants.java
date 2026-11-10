package org.firstinspires.ftc.teamcode.constants;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Rapid-fire consistency fixes. Only used when Robot.enableShooterAssist() is called
 * (the "TeleOp (Shooter Assist)" OpMode); the normal TeleOp ignores this file.
 * Each fix has its own switch so they can be compared one at a time from Panels.
 *
 * The hub reports flywheel speed in steps of 20 ticks/s = 42.86 RPM (..., 1500, 1543,
 * 1586, ...). Thresholds below are sized against that step, and a TARGET_RPM that is a
 * multiple of 42.86 lets the controller see zero error when it is actually on target.
 */
@Configurable
public class ShooterAssistConstants {

    // ---- 1. Recovery boost: full power while the wheel is well under target. ----
    public static boolean RECOVERY_BOOST_ENABLED = true;
    // Boost while measured RPM is more than this far below target. Must be bigger than
    // one 43 RPM step or the boost will chatter at steady state.
    public static double RECOVERY_BOOST_DEADBAND_RPM = 60;
    public static double RECOVERY_BOOST_POWER = 1.0;

    // ---- 2. Feed gating: hold the transfer until the wheel is back at speed. ----
    // Only applies while shooting (trigger held and blocker open).
    public static boolean FEED_GATING_ENABLED = true;
    // The wheel counts as ready between (target - LOW) and (target + HIGH).
    public static double FEED_READY_LOW_RPM = 50;
    public static double FEED_READY_HIGH_RPM = 100;

    // ---- 3. Integral that actually trims steady-state error. ----
    // Replaces ShooterConstants.kI / MAX_INTEGRAL. Largest power it can add or remove
    // is kI * MAX_INTEGRAL (0.15 with these values). Frozen while boosting.
    public static double kI = 0.001;
    public static double MAX_INTEGRAL = 150;

    // ---- 4. Idle spin: keep the wheel turning between bursts. ----
    // Starts after the first trigger pull; Y stops the wheel completely.
    public static boolean IDLE_SPIN_ENABLED = true;
    public static double IDLE_RPM = 1200;

    // ---- 5. Less filtering, so the controller sees each ball sooner. ----
    // Replaces ShooterConstants.VELOCITY_ALPHA.
    public static double VELOCITY_ALPHA = 0.4;
}
