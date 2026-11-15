package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.math.SpeedController;

/** Shared shooter setup; neither test initializes drive, intake, or turret hardware. */
public abstract class ShooterTestBase extends OpMode {
    // goBILDA 5000 series, direct drive. Change if the encoder/gearing changes.
    private static final double TICKS_PER_REV = 28.0;
    private DcMotorEx shooterMotor;
    protected SpeedController controller;

    protected abstract double targetRpm();
    protected abstract void updateCoefficients();

    @Override
    public void init() {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        shooterMotor.setPower(0.0);
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        controller = new SpeedController(hardwareMap);
    }

    @Override
    public void start() {
        controller.reset();
    }

    @Override
    public void loop() {
        updateCoefficients();
        double rpm = targetRpm();
        double velocity = shooterMotor.getVelocity();
        double power = controller.calculate(rpm * TICKS_PER_REV / 60.0, velocity);
        shooterMotor.setPower(power);
        telemetry.addData("Target RPM", rpm);
        telemetry.addData("Actual RPM", velocity * 60.0 / TICKS_PER_REV);
        telemetry.addData("Filtered RPM", controller.getFilteredVelocity() * 60.0 / TICKS_PER_REV);
        telemetry.addData("Profiled RPM", controller.getProfiledTarget() * 60.0 / TICKS_PER_REV);
        telemetry.addData("RPM error", rpm - velocity * 60.0 / TICKS_PER_REV);
        telemetry.addData("Shooter power", power);
        telemetry.addData("Integral (ticks)", controller.getIntegral());
        telemetry.update();
    }

    @Override
    public void stop() {
        if (shooterMotor != null) shooterMotor.setPower(0.0);
        if (controller != null) controller.reset();
    }
}
