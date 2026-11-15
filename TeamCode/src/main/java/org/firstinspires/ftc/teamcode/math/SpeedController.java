package org.firstinspires.ftc.teamcode.math;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

/** Software velocity PIDF. Velocities are ticks/s; acceleration is ticks/s^2. */
public class SpeedController {
    private static final double MAX_ACCEL = 6000.0;
    private static final double VELOCITY_ALPHA = 0.7;
    private static final double KS_DEADBAND = 50.0;
    private static final double MAX_INTEGRAL = 1000.0;
    private static final double NOMINAL_VOLTAGE = 12.0;

    private final VoltageSensor voltageSensor;
    private double kP, kI, kD, kF, kS, kA;
    private double integral, lastError, filteredVel, profiledTarget;
    private long lastTimeNs = -1L;

    public SpeedController(HardwareMap hardwareMap) {
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    public void setCoefficients(double p, double i, double d, double f, double s, double a) {
        // Do not retain integral accumulated while I was disabled or being tuned.
        if (kI != i) integral = 0.0;
        kP = p;
        kI = i;
        kD = d;
        kF = f;
        kS = s;
        kA = a;
    }

    public double calculate(double targetVel, double measuredVel) {
        if (targetVel == 0.0) {
            reset();
            return 0.0;
        }
        long nowNs = System.nanoTime();
        if (lastTimeNs < 0) {
            lastTimeNs = nowNs;
            filteredVel = measuredVel;
            profiledTarget = measuredVel;
            return 0.0;
        }
        double dt = (nowNs - lastTimeNs) * 1e-9;
        lastTimeNs = nowNs;
        if (dt <= 0) return 0.0;

        double previousVel = filteredVel;
        filteredVel = VELOCITY_ALPHA * filteredVel + (1.0 - VELOCITY_ALPHA) * measuredVel;
        double previousTarget = profiledTarget;
        double maxDelta = MAX_ACCEL * dt;
        profiledTarget += clamp(targetVel - profiledTarget, -maxDelta, maxDelta);
        double acceleration = (profiledTarget - previousTarget) / dt;
        lastError = profiledTarget - filteredVel;
        double derivative = -(filteredVel - previousVel) / dt;

        if (kI != 0.0 && Math.abs(kP * lastError) < 1.0) {
            integral = clamp(integral + lastError * dt, -MAX_INTEGRAL, MAX_INTEGRAL);
        }
        double staticTerm = Math.abs(profiledTarget) > KS_DEADBAND
                ? Math.copySign(kS, profiledTarget) : 0.0;
        double voltage = voltageSensor.getVoltage();
        double voltageScale = voltage > 0.1 ? NOMINAL_VOLTAGE / voltage : 1.0;
        double output = kF * profiledTarget + staticTerm + kA * acceleration
                + kP * lastError + kI * integral + kD * derivative;
        return clamp(output * voltageScale, -1.0, 1.0);
    }

    public void reset() {
        integral = lastError = filteredVel = profiledTarget = 0.0;
        lastTimeNs = -1L;
    }

    public double getFilteredVelocity() { return filteredVel; }
    public double getProfiledTarget() { return profiledTarget; }
    public double getError() { return lastError; }
    public double getIntegral() { return integral; }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
