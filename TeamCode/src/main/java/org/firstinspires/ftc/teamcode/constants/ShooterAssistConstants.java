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

    // ---- 1. Recovery boost: full power while the wheel is under target. ----
    public static boolean RECOVERY_BOOST_ENABLED = true;
    // While just holding speed: boost once the wheel reads this far below target. Keep
    // it above one encoder step (21 RPM). Below that, the normal one-step flicker of the
    // reading fires the boost several times a second and the motor audibly stutters.
    public static double RECOVERY_BOOST_DEADBAND_RPM = 30;
    // While a burst is being fed: boost on the first step down, for the fastest recovery
    // between balls. Stutter does not matter here; the wheel is being hit anyway.
    public static double FEED_BOOST_DEADBAND_RPM = 10;
    public static double RECOVERY_BOOST_POWER = 1.0;

    // ---- 2. Feed gating: hold the transfer until the wheel is back at speed. ----
    // Only applies while shooting (trigger held and blocker open).
    public static boolean FEED_GATING_ENABLED = true;
    // The wheel counts as ready between (target - LOW) and (target + HIGH).
    public static double FEED_READY_LOW_RPM = 25;
    // Allows up to two encoder steps high. 30 (one step) was tried and reverted along
    // with the other feed changes after doubles appeared; tighten it on its own if balls
    // go long.
    public static double FEED_READY_HIGH_RPM = 55;
    // Judge readiness on the middle of the last three speed readings instead of the
    // newest one. Single-sample encoder jumps (one step of dither, or a glitch as a
    // ball hits) then no longer switch the transfer on and off. Costs one loop of delay.
    public static boolean FEED_GATE_FILTER_ENABLED = true;

    // ---- 2b. Pulsed feed: one ball per pulse, at full transfer power. ----
    // Run continuously, the transfer delivers balls about 100 ms apart, faster than the
    // gate can react, so the second ball of a pair leaves slow. Pulsing spaces them
    // without slowing the transfer, so every ball (the last one included) gets a full
    // push into the wheel. A pulse starts only when the wheel is ready and then always
    // runs its full length.
    public static boolean FEED_PULSE_ENABLED = true;
    // Length of one pulse. Tune until exactly one ball goes per pulse: shorten it if
    // two go together, lengthen it if a pulse sometimes moves no ball.
    public static double FEED_PULSE_ON_MS = 140;
    // Shortest rest between pulses, even if the wheel already reads ready. Covers the
    // delay before a ball's hit shows up in the speed reading. Lower it for a faster
    // burst; raise it if a ball is fed before the previous one's dip has been seen.
    public static double FEED_PULSE_MIN_OFF_MS = 150;

    // ---- 2c. Rapid fire (only with Robot.enableRapidFire()). ----
    // The staggered feed above waits to see each ball's dip before sending the next,
    // which costs about 0.3 s per ball. Rapid fire does not wait: after the first
    // pulse it sends balls on a fixed beat. The beat (ON + OFF) should match how long
    // the wheel needs to win back one ball's worth of speed, about 200 ms.
    // Shorter is faster but later balls fall short; longer is slower but safer.
    public static double RAPID_PULSE_ON_MS = 100;
    public static double RAPID_PULSE_OFF_MS = 120;

    // ---- 2d. Final push (both feed styles). ----
    // After this many pulses in a burst the transfer stops pulsing and runs continuously
    // whenever the wheel is ready. The last ball has nothing behind it and short pulses
    // only nudge it, so it sat for up to a second before going in. Set it to the number
    // of balls minus one; 0 turns it off.
    // OFF by default: a pulse sometimes moves no ball, so two balls can be left when the
    // continuous push starts, and they then go in together and both miss.
    public static int FEED_FINAL_PUSH_AFTER_PULSES = 0;

    // Transfer power used while shooting, in place of IntakeConstants.TRANSFER_POWER.
    public static boolean SHOOT_TRANSFER_POWER_ENABLED = true;
    public static double SHOOT_TRANSFER_POWER = 0.8;

    // ---- 3. Integral that actually trims steady-state error. ----
    // Replaces ShooterConstants.kI / MAX_INTEGRAL. Largest power it can add or remove
    // is kI * MAX_INTEGRAL (0.15 with these values). Frozen while boosting.
    public static double kI = 0.001;
    public static double MAX_INTEGRAL = 150;

    // ---- 4. Idle spin: keep the wheel turning between bursts. ----
    // Starts after the first trigger pull; Y stops the wheel completely.
    public static boolean IDLE_SPIN_ENABLED = true;
    public static double IDLE_RPM = 0;

    // ---- 5. Less filtering, so the controller sees each ball sooner. ----
    // Replaces ShooterConstants.VELOCITY_ALPHA.
    public static double VELOCITY_ALPHA = 0.4;
}
