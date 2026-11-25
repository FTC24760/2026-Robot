package org.firstinspires.ftc.teamcode.constants;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Configurable
public class ShooterConstants {
    public static String MOTOR_NAME = "shooter";
    public static DcMotorSimple.Direction direction = DcMotorSimple.Direction.FORWARD;

    // goBILDA 5000 series, no gearbox.
    public static final double TICKS_PER_REV = 28.0;

    // The hub reports speed in steps of 20 ticks/s = 42.86 RPM (..., 1500, 1543, 1586, ...).
    // Keep this on a step so the controller reads zero error when it is on target.
    public static double MANUAL_RPM = 3350;

    // Software velocity controller gains, in motor power per encoder tick/second
    // (not the hub PIDF units). Tuning order: kF, kS, kP, kD, kI, kA.
    // Feedforward power per tick/s. Tune first so steady speed holds with little PID work.
    public static double kF = 0.00039;
    // Static friction/start power added with feedforward.

    public static double kS = 0.14;

    public static double kP = 0.012;
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
}
