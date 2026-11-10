package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.utils.MathUtil.rpmToTicksPerSecond;
import static org.firstinspires.ftc.teamcode.utils.MathUtil.ticksPerSecondToRpm;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.utils.SpeedController;

/** Flywheel shooter run by a software velocity controller (not the hub's velocity loop). */
public class Shooter implements Subsystem {

    public enum State {
        /** Hold ShooterConstants.TARGET_RPM. */
        SHOOT,
        /** Hold ShooterConstants.IDLE_RPM. */
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

    public Shooter(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, ShooterConstants.MOTOR_NAME);
        // Power is controlled in software; avoid the hub velocity loop.
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
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
        } else {
            double targetRpm = state == State.SHOOT
                    ? ShooterConstants.TARGET_RPM
                    : ShooterConstants.IDLE_RPM;

            targetTicksPerSecond = rpmToTicksPerSecond(targetRpm, ShooterConstants.TICKS_PER_REV);
            syncGains();
            power = controller.calculate(
                    targetTicksPerSecond,
                    actualTicksPerSecond,
                    voltageScale,
                    nowNs
            );
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

    @Override
    public void stop() {
        state = State.DISABLED;
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
    }
}
