package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
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
    protected TelemetryManager panelsTelemetry;

    protected abstract double targetRpm();
    protected abstract void updateCoefficients();

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
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
        panelsTelemetry.addData("Target RPM", rpm);
        panelsTelemetry.addData("Actual RPM", velocity * 60.0 / TICKS_PER_REV);
        panelsTelemetry.addData("Filtered RPM", controller.getFilteredVelocity() * 60.0 / TICKS_PER_REV);
        panelsTelemetry.addData("Profiled RPM", controller.getProfiledTarget() * 60.0 / TICKS_PER_REV);
        panelsTelemetry.addData("RPM error", rpm - velocity * 60.0 / TICKS_PER_REV);
        panelsTelemetry.addData("Shooter power", power);
        panelsTelemetry.addData("Integral (ticks)", controller.getIntegral());
        // Publish the same values to Panels and the Driver Station.
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void stop() {
        if (shooterMotor != null) shooterMotor.setPower(0.0);
        if (controller != null) controller.reset();
    }
}
