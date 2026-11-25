package org.firstinspires.ftc.teamcode.utils;

import static org.firstinspires.ftc.teamcode.utils.MathUtil.clamp;

/**
 * Velocity controller with motion profiling, P + feedforward, and voltage compensation.
 * No integral, no derivative and no velocity filtering: the error uses the raw reading.
 * Units are whatever the caller measures velocity in (encoder ticks/s for the shooter);
 * the output is motor power in [-1, 1].
 * Tuning order: kF, kS, kP, kA.
 */
public class SpeedController {

    /** Tunable values. The owner may rewrite these at any time (e.g. from live-tuned constants). */
    public static class Gains {
        public double kP;
        // Feedforward per unit of velocity, static friction offset, and per unit of acceleration.
        public double kF, kS, kA;
        // Limit on how fast the profiled target may ramp, velocity units per second.
        public double maxAccel = 1000;
        // kS is only applied when the profiled target is above this speed.
        public double kSDeadband = 50;
    }

    public final Gains gains = new Gains();

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
        if (lastTimeNs < 0) {
            lastTimeNs = nowNs;
            profiledTarget = measured;
            return 0.0;
        }

        double dt = (nowNs - lastTimeNs) * 1e-9;
        lastTimeNs = nowNs;
        if (dt <= 0) {
            return 0.0;
        }

        double previousTarget = profiledTarget;
        double maxDelta = Math.max(0, gains.maxAccel) * dt;
        profiledTarget += clamp(target - profiledTarget, -maxDelta, maxDelta);
        double accel = (profiledTarget - previousTarget) / dt;

        double error = profiledTarget - measured;

        double staticTerm = Math.abs(profiledTarget) > gains.kSDeadband
                ? Math.copySign(gains.kS, profiledTarget)
                : 0.0;

        double output = gains.kF * profiledTarget
                + staticTerm
                + gains.kA * accel
                + gains.kP * error;

        return clamp(output * voltageScale, -1, 1);
    }

    public void reset() {
        profiledTarget = 0;
        lastTimeNs = -1;
    }

    public double getProfiledTarget() {
        return profiledTarget;
    }
}
