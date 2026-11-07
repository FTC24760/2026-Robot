package org.firstinspires.ftc.teamcode;

public class ShooterConstants {
    public static String shooterName = "shooter";

    public static double kA = 0; // what is this? - tony
    public static double kP = 0.0035;
    // Integral removes steady-state RPM error. Keep small; too high causes windup.
    public static double kI = 0.00001;
    // Derivative damps overshoot. Raise slightly if the shooter spikes past target.
    public static double kD = 0.00005;
    // Feedforward power per RPM. Tune first so steady RPM holds with little PID work.
    public static double kF = 0.0001625;
    // Static friction/start power added with feedforward.
    public static double kS = 0.060;
    // Max integral accumulator. Lower if the controller stays wound up.
    public static double MAX_INTEGRAL = 5000;
    public static double TARGET_RPM = 1520;
    public static double IDLE_RPM = 0;
    // Shooter is ready inside this RPM window around target.
    public static double RPM_THRESHOLD = 150;
    // RPM loss before transfer recovery starts adding extra commanded speed.
    public static double TRANSFER_RECOVERY_DEADBAND_RPM = 100;
    // Extra RPM command per RPM lost past the deadband while the transfer is active.
    public static double TRANSFER_RECOVERY_MULTIPLIER = 5.0;
    // Auto starts feeding with a tighter low-side RPM window than teleop.
    public static double AUTON_READY_LOW_RPM_THRESHOLD = 150;
    // Auto recovery starts earlier because feed timing is less forgiving than driver timing.
    public static double AUTON_TRANSFER_RECOVERY_DEADBAND_RPM = 75;
    public static double AUTON_TRANSFER_RECOVERY_MULTIPLIER = 5.0;
    // Voltage used as the feedforward reference.
    public static double nominalVoltage = 12.0;
    // Target RPM scale when waiting outside a shooting zone.
    public static double OUTSIDE_SHOOTING_ZONE_RPM_SCALE = 0.90;

    // Manual shooter tuning mode values.
    public static double tuningRPM = 3000.0;
    public static double tuningHoodPos = 0.75;
}