package org.firstinspires.ftc.teamcode.constants;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class ShooterConstants {
    public static String MOTOR_NAME = "shooter";

    // goBILDA 5000 series, no gearbox.
    public static final double TICKS_PER_REV = 28.0;

    public static double TARGET_RPM = 1550;
    public static double IDLE_RPM = 0;

    // Software velocity controller gains, in motor power per encoder tick/second
    // (not the hub PIDF units). Tuning order: kF, kS, kP, kD, kI, kA.
    // Feedforward power per tick/s. Tune first so steady speed holds with little PID work.
    public static double kF = 0.00089;
    // Static friction/start power added with feedforward.
    public static double kS = 0.069;
    public static double kP = 0.01;
    // Integral removes steady-state error. Keep small; too high causes windup.
    public static double kI = 0.00002;
    // Derivative damps overshoot. Raise slightly if the shooter spikes past target.
    public static double kD = 0.00005;
    // Feedforward power per tick/s^2 of profiled acceleration.
    public static double kA = 0.0;
    // Max integral accumulator. Lower if the controller stays wound up.
    public static double MAX_INTEGRAL = 5000;
    // Target ramp limit, ticks/s^2.
    public static double MAX_ACCEL = 6000.0;
    // Velocity low-pass filter: 0 = no filtering, closer to 1 = smoother but laggier.
    public static double VELOCITY_ALPHA = 0.7;
    // kS is only applied above this target speed, ticks/s.
    public static double KS_DEADBAND = 50.0;

    // ---- Not wired into any subsystem yet (carried over for upcoming features). ----
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
    // Target RPM scale when waiting outside a shooting zone.
    public static double OUTSIDE_SHOOTING_ZONE_RPM_SCALE = 0.90;
    // Manual shooter tuning mode values.
    public static double TUNING_RPM = 3000.0;
    public static double TUNING_HOOD_POSITION = 0.75;
}
