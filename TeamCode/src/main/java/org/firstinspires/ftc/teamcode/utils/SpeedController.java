package org.firstinspires.ftc.teamcode.utils;

import static org.firstinspires.ftc.teamcode.utils.MathUtil.clamp;

/**
 * Velocity controller with motion profiling, PIDF, and voltage compensation.
 * Units are whatever the caller measures velocity in (encoder ticks/s for the shooter);
 * the output is motor power in [-1, 1].
 * Tuning order: kF, kS, kP, kD, kI, kA.
 */
public class SpeedController {

    /** Tunable values. The owner may rewrite these at any time (e.g. from live-tuned constants). */
    public static class Gains {
        public double kP, kI, kD;
        // Feedforward per unit of velocity, static friction offset, and per unit of acceleration.
        public double kF, kS, kA;
        public double maxIntegral = 5000;
        // Limit on how fast the profiled target may ramp, velocity units per second.
        public double maxAccel = 6000;
        // Measured-velocity low-pass filter: 0 = unfiltered, closer to 1 = smoother.
        public double velocityAlpha = 0.7;
        // kS is only applied when the profiled target is above this speed.
        public double kSDeadband = 50;
    }

    public final Gains gains = new Gains();

    private double integral;
    private double filteredVel;
    private double profiledTarget;
    private long lastTimeNs = -1;

    /**
     * @param target       desired velocity
     * @param measured     measured velocity
     * @param voltageScale nominal voltage / battery voltage
     * @param nowNs        timestamp of the measurement, from System.nanoTime()
     * @return motor power in [-1, 1]
     */
    public double calculate(double target, double measured, double voltageScale, long nowNs) {
        return calculate(target, measured, voltageScale, nowNs, false);
    }

    /**
     * @param holdIntegral true to leave the integral where it is this step, e.g. while the
     *                     caller is overriding the output and the error is not the
     *                     controller's to fix
     */
    public double calculate(
            double target,
            double measured,
            double voltageScale,
            long nowNs,
            boolean holdIntegral
    ) {
        if (lastTimeNs < 0) {
            lastTimeNs = nowNs;
            filteredVel = measured;
            profiledTarget = measured;
            return 0.0;
        }

        double dt = (nowNs - lastTimeNs) * 1e-9;
        lastTimeNs = nowNs;
        if (dt <= 0) {
            return 0.0;
        }

        double previousVel = filteredVel;
        double alpha = clamp(gains.velocityAlpha, 0, 1);
        filteredVel = alpha * filteredVel + (1 - alpha) * measured;

        double previousTarget = profiledTarget;
        double maxDelta = Math.max(0, gains.maxAccel) * dt;
        profiledTarget += clamp(target - profiledTarget, -maxDelta, maxDelta);
        double accel = (profiledTarget - previousTarget) / dt;

        double error = profiledTarget - filteredVel;
        double derivative = -(filteredVel - previousVel) / dt;

        // Only integrate while the proportional term is not already saturating the output.
        if (!holdIntegral && Math.abs(gains.kP * error) < 1.0) {
            double limit = Math.abs(gains.maxIntegral);
            integral = clamp(integral + error * dt, -limit, limit);
        }

        double staticTerm = Math.abs(profiledTarget) > gains.kSDeadband
                ? Math.copySign(gains.kS, profiledTarget)
                : 0.0;

        double output = gains.kF * profiledTarget
                + staticTerm
                + gains.kA * accel
                + gains.kP * error
                + gains.kI * integral
                + gains.kD * derivative;

        return clamp(output * voltageScale, -1, 1);
    }

    public void reset() {
        integral = 0;
        filteredVel = 0;
        profiledTarget = 0;
        lastTimeNs = -1;
    }

    public double getFilteredVelocity() {
        return filteredVel;
    }

    public double getProfiledTarget() {
        return profiledTarget;
    }

    public double getIntegral() {
        return integral;
    }
}
