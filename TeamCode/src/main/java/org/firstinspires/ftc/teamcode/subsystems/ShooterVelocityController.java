package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.constants.JerryRiggedTeleOpConstants;

/**
 * Software PIDF in encoder ticks/second, not hub PIDF units.
 * The algorithm is intentionally identical to the original JerryRigged
 * TeleOp controller. No adaptive recovery or additional feed gating.
 */
public final class ShooterVelocityController {
    private double integral;
    private double filteredVelocity;
    private double profiledTarget;
    private long lastTimeNs = -1;

    public double calculate(double target, double measured, double voltageScale, long nowNs) {
        if (lastTimeNs < 0) {
            lastTimeNs = nowNs;
            filteredVelocity = measured;
            profiledTarget = measured;
            return 0.0;
        }
        double dt = (nowNs - lastTimeNs) * 1e-9;
        lastTimeNs = nowNs;
        if (dt <= 0) return 0.0;

        double previousVelocity = filteredVelocity;
        double alpha = clamp(JerryRiggedTeleOpConstants.SHOOTER_VELOCITY_ALPHA, 0, 1);
        filteredVelocity = alpha * filteredVelocity + (1 - alpha) * measured;

        double previousTarget = profiledTarget;
        double maxDelta = Math.max(0, JerryRiggedTeleOpConstants.SHOOTER_MAX_ACCEL_TICKS_PER_SEC2) * dt;
        profiledTarget += clamp(target - profiledTarget, -maxDelta, maxDelta);

        double acceleration = (profiledTarget - previousTarget) / dt;
        double error = profiledTarget - filteredVelocity;
        double derivative = -(filteredVelocity - previousVelocity) / dt;

        double kp = JerryRiggedTeleOpConstants.SHOOTER_KP;
        if (Math.abs(kp * error) < 1.0) {
            double limit = Math.abs(JerryRiggedTeleOpConstants.SHOOTER_MAX_INTEGRAL);
            integral = clamp(integral + error * dt, -limit, limit);
        }

        double staticTerm = Math.abs(profiledTarget) >
                JerryRiggedTeleOpConstants.SHOOTER_KS_DEADBAND_TICKS_PER_SEC
                ? Math.copySign(JerryRiggedTeleOpConstants.SHOOTER_KS, profiledTarget)
                : 0.0;

        return clamp((
                JerryRiggedTeleOpConstants.SHOOTER_KF * profiledTarget
                + staticTerm
                + JerryRiggedTeleOpConstants.SHOOTER_KA * acceleration
                + kp * error
                + JerryRiggedTeleOpConstants.SHOOTER_KI * integral
                + JerryRiggedTeleOpConstants.SHOOTER_KD * derivative
        ) * voltageScale, -1, 1);
    }

    public double getProfiledTarget() { return profiledTarget; }
    public double getFilteredVelocity() { return filteredVelocity; }

    public void reset() {
        integral = 0;
        filteredVelocity = 0;
        profiledTarget = 0;
        lastTimeNs = -1;
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
