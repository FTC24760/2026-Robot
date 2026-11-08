package org.firstinspires.ftc.teamcode.constants;

/**
 * Tunable values for the checked-in JerryRiggedPidfTeleOp.
 *
 * Deliberately preserves the original main-branch values; these are NOT
 * the experimental 1410-RPM / 0.0015-kP diagnostic baseline.
 * Update values here when a tune has been validated on hardware.
 */
public final class JerryRiggedTeleOpConstants {
    private JerryRiggedTeleOpConstants() {}

    public static double INTAKE_POWER = 0.6;
    public static double TRANSFER_POWER = 0.8;

    public static double SHOOTER_TARGET_RPM = 1470.0;
    public static double SHOOTER_KF = 0.0006825;
    public static double SHOOTER_KS = 0.069;
    public static double SHOOTER_KP = 0.0051;
    public static double SHOOTER_KI = 0.00002;
    public static double SHOOTER_KD = 0.00005;
    public static double SHOOTER_KA = 0.0;
    public static double SHOOTER_MAX_INTEGRAL = 5000.0;
    public static double SHOOTER_NOMINAL_VOLTAGE = 12.0;
    public static double SHOOTER_MAX_ACCEL_TICKS_PER_SEC2 = 6000.0;
    public static double SHOOTER_VELOCITY_ALPHA = 0.7;
    public static double SHOOTER_KS_DEADBAND_TICKS_PER_SEC = 50.0;

    public static double TURRET_RATE_POSITION_PER_SECOND = 0.25;
    public static double TURRET_INITIAL_POSITION = 0.5;
    public static double BLOCKER_OPEN_POSITION = 0.032;
    public static double BLOCKER_CLOSED_POSITION = 0.0;
    public static double SHOOTER_TRIGGER_THRESHOLD = 0.2;
    public static double FEED_TRIGGER_THRESHOLD = 0.2;
    public static long VOLTAGE_SAMPLE_INTERVAL_NS = 250_000_000L;
}
