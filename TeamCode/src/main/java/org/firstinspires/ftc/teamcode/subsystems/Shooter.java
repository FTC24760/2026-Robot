package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.utils.MathUtil.clamp;
import static org.firstinspires.ftc.teamcode.utils.MathUtil.rpmToTicksPerSecond;
import static org.firstinspires.ftc.teamcode.utils.MathUtil.ticksPerSecondToRpm;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.ShooterAssistConstants;
import org.firstinspires.ftc.teamcode.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.utils.SpeedController;

/**
 * Flywheel shooter run by a software velocity controller (not the hub's velocity loop).
 * With assist enabled it also applies the rapid-fire fixes in ShooterAssistConstants.
 */
public class Shooter implements Subsystem {

    public enum State {
        /** Hold ShooterConstants.TARGET_RPM. */
        SHOOT,
        /** Hold the idle RPM (ShooterAssistConstants.IDLE_RPM with assist, else ShooterConstants). */
        IDLE,
        /** Motor off and controller reset; the flywheel coasts. */
        DISABLED
    }

    private final DcMotorEx motor;
    private final SpeedController controller = new SpeedController();

    private State state = State.DISABLED;
    private double targetTicksPerSecond;
    private double actualTicksPerSecond;
    private double power;

    private boolean assistEnabled;
    private boolean boosting;

    public Shooter(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, ShooterConstants.MOTOR_NAME);
        // Power is controlled in software; avoid the hub velocity loop.
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    /** Turn on the rapid-fire fixes configured in ShooterAssistConstants. */
    public void setAssistEnabled(boolean assistEnabled) {
        this.assistEnabled = assistEnabled;
    }

    public void setState(State state) {
        this.state = state;
    }

    public State getState() {
        return state;
    }

    /**
     * Run one step of the velocity controller. Call exactly once per loop: it takes the
     * single fresh velocity read that the getters below then reuse.
     *
     * @param nowNs        loop timestamp from System.nanoTime()
     * @param voltageScale nominal voltage / battery voltage
     */
    public void update(long nowNs, double voltageScale) {
        actualTicksPerSecond = motor.getVelocity();

        if (state == State.DISABLED) {
            targetTicksPerSecond = 0.0;
            controller.reset();
            power = 0.0;
            boosting = false;
        } else {
            double targetRpm = getStateTargetRpm();

            targetTicksPerSecond = rpmToTicksPerSecond(targetRpm, ShooterConstants.TICKS_PER_REV);
            syncGains();

            // Decided on the raw reading, not the filtered one, to react as early as possible.
            boosting = assistEnabled
                    && ShooterAssistConstants.RECOVERY_BOOST_ENABLED
                    && state == State.SHOOT
                    && getActualRpm() < targetRpm - ShooterAssistConstants.RECOVERY_BOOST_DEADBAND_RPM;

            // The controller keeps running while boosting so its filter and ramp stay
            // current, but its integral is held: the dip is the boost's job to fix.
            power = controller.calculate(
                    targetTicksPerSecond,
                    actualTicksPerSecond,
                    voltageScale,
                    nowNs,
                    boosting
            );

            if (assistEnabled) {
                // Never drive the wheel backwards to slow it; let it coast down instead.
                power = Math.max(power, 0.0);
            }

            if (boosting) {
                power = Math.max(power, clamp(ShooterAssistConstants.RECOVERY_BOOST_POWER, 0, 1));
            }
        }

        motor.setPower(power);
    }

    public double getTargetRpm() {
        return ticksPerSecondToRpm(targetTicksPerSecond, ShooterConstants.TICKS_PER_REV);
    }

    public double getActualRpm() {
        return ticksPerSecondToRpm(actualTicksPerSecond, ShooterConstants.TICKS_PER_REV);
    }

    /** The ramped target the controller is actually chasing right now. */
    public double getProfiledRpm() {
        return ticksPerSecondToRpm(controller.getProfiledTarget(), ShooterConstants.TICKS_PER_REV);
    }

    public double getPower() {
        return power;
    }

    /** True while the recovery boost is overriding the controller with full power. */
    public boolean isBoosting() {
        return boosting;
    }

    /** True when the wheel is at shooting speed, close enough to feed the next ball. */
    public boolean isReadyToFeed() {
        if (state != State.SHOOT) {
            return false;
        }

        double error = getActualRpm() - ShooterConstants.TARGET_RPM;
        return error >= -ShooterAssistConstants.FEED_READY_LOW_RPM
                && error <= ShooterAssistConstants.FEED_READY_HIGH_RPM;
    }

    @Override
    public void stop() {
        state = State.DISABLED;
        boosting = false;
        motor.setPower(0);
        controller.reset();
    }

    @Override
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Target RPM", getTargetRpm());
        telemetry.addData("Actual RPM", "%.1f", getActualRpm());
        telemetry.addData("Profiled RPM", "%.1f", getProfiledRpm());
        telemetry.addData("Shooter power", "%.3f", power);
        telemetry.addData("Feedforward", ShooterConstants.kF);

        if (assistEnabled) {
            telemetry.addData("Shooter state", state);
            telemetry.addData("Ready to feed", isReadyToFeed() ? "yes" : "no");
            telemetry.addData("Boosting", boosting ? "yes" : "no");
            telemetry.addData("Integral power", "%.3f", controller.getIntegral() * controller.gains.kI);
        }
    }

    // Copied every loop so live edits to ShooterConstants take effect immediately.
    private void syncGains() {
        SpeedController.Gains gains = controller.gains;
        gains.kP = ShooterConstants.kP;
        gains.kI = ShooterConstants.kI;
        gains.kD = ShooterConstants.kD;
        gains.kF = ShooterConstants.kF;
        gains.kS = ShooterConstants.kS;
        gains.kA = ShooterConstants.kA;
        gains.maxIntegral = ShooterConstants.MAX_INTEGRAL;
        gains.maxAccel = ShooterConstants.MAX_ACCEL;
        gains.velocityAlpha = ShooterConstants.VELOCITY_ALPHA;
        gains.kSDeadband = ShooterConstants.KS_DEADBAND;

        if (assistEnabled) {
            gains.kI = ShooterAssistConstants.kI;
            gains.maxIntegral = ShooterAssistConstants.MAX_INTEGRAL;
            gains.velocityAlpha = ShooterAssistConstants.VELOCITY_ALPHA;
        }
    }

    private double getStateTargetRpm() {
        if (state == State.SHOOT) {
            return ShooterConstants.TARGET_RPM;
        }
        return assistEnabled ? ShooterAssistConstants.IDLE_RPM : ShooterConstants.IDLE_RPM;
    }
}
