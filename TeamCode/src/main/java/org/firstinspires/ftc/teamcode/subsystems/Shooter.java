package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.utils.MathUtil.rpmToTicksPerSecond;
import static org.firstinspires.ftc.teamcode.utils.MathUtil.ticksPerSecondToRpm;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.utils.SpeedController;

/**
 * Flywheel shooter run by a software velocity controller (not the hub's velocity loop).
 * Set state and shootRpmOverride, then call update() once per loop. Power is scaled by
 * Robot.voltageScale. ENABLED chases getShootRpm(); DISABLED lets the wheel coast.
 */
public class Shooter implements Subsystem {

    public enum State {
        ENABLED,
        DISABLED
    }

    private final DcMotorEx motor;
    private final SpeedController controller = new SpeedController();

    public State state = State.DISABLED;
    private double targetTicksPerSecond;
    private double actualTicksPerSecond;
    public double power;
    private final double[] recentRpm = new double[3];
    // Shooting speed to aim for; NaN means use ShooterConstants.MANUAL_RPM.
    // The aim command (ShotCommands.autoAim / manualAim) sets this every loop.
    public double shootRpmOverride = Double.NaN;

    public Shooter(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, ShooterConstants.MOTOR_NAME);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setDirection(ShooterConstants.direction);
    }

    /** Go back to shooting at ShooterConstants.MANUAL_RPM. */
    public void clearShootRpm() {
        shootRpmOverride = Double.NaN;
    }

    /** The speed the ENABLED state aims for right now. */
    public double getShootRpm() {
        if (!Double.isNaN(shootRpmOverride)) {
            return shootRpmOverride;
        }
        return ShooterConstants.MANUAL_RPM;
    }

    /**
     * Run one step of the velocity controller. Call exactly once per loop: it takes the
     * single fresh velocity read that the getters below then reuse.
     *
     * @param nowNs loop timestamp from System.nanoTime()
     */
    public void update(long nowNs) {
        actualTicksPerSecond = motor.getVelocity();
        recentRpm[2] = recentRpm[1];
        recentRpm[1] = recentRpm[0];
        recentRpm[0] = getActualRpm();

        if (state == State.DISABLED) {
            targetTicksPerSecond = 0.0;
            controller.reset();
            power = 0.0;
        } else {
            double targetRpm = getStateTargetRpm();

            targetTicksPerSecond = rpmToTicksPerSecond(targetRpm, ShooterConstants.TICKS_PER_REV);
            syncGains();

            power = controller.calculate(
                    targetTicksPerSecond,
                    actualTicksPerSecond,
                    Robot.voltageScale,
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
        return ticksPerSecondToRpm(controller.profiledTarget, ShooterConstants.TICKS_PER_REV);
    }

    /** True when the wheel is at shooting speed, close enough to feed the next ball. */
    public boolean isAtTargetRpm() {
        // TODO placeholder: always ready. Replace with a real check against getTargetRpm().
        return true;
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

    private double getStateTargetRpm() {
        if (state == State.ENABLED) {
            return getShootRpm();
        }
        else return 0;
    }
}
