package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import java.util.List;

import org.firstinspires.ftc.teamcode.constants.JerryRiggedHardwareConstants;
import org.firstinspires.ftc.teamcode.constants.JerryRiggedTeleOpConstants;
import org.firstinspires.ftc.teamcode.subsystems.ButterflyController;
import org.firstinspires.ftc.teamcode.subsystems.JerryRiggedDrive;
import org.firstinspires.ftc.teamcode.subsystems.JerryRiggedMechanisms;
import org.firstinspires.ftc.teamcode.subsystems.ShooterVelocityController;

/**
 * Original JerryRigged PIDF controls, extracted into modular subsystems.
 * Tuning remains at the repository's original values, not the experimental
 * diagnostic values. Continuous transfer and the original gamepad mapping remain.
 */
@TeleOp(name = "Jerry Rigged PIDF Teleop", group = "Robot")
public class JerryRiggedPidfTeleOp extends OpMode {

    private JerryRiggedDrive drive;
    private JerryRiggedMechanisms mechanisms;
    private ButterflyController butterfly;
    private DcMotorEx shooterMotor;
    private ShooterVelocityController speedController;

    private List<LynxModule> hubs;
    private VoltageSensor voltageSensor;
    private long lastVoltageReadNs;
    private long lastLoopNs;
    private double voltageScale = 1.0;

    @Override
    public void init() {
        drive = new JerryRiggedDrive(hardwareMap);
        mechanisms = new JerryRiggedMechanisms(hardwareMap);
        butterfly = new ButterflyController(hardwareMap);
        speedController = new ShooterVelocityController();

        shooterMotor = hardwareMap.get(DcMotorEx.class, JerryRiggedHardwareConstants.SHOOTER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        voltageSensor = hardwareMap.voltageSensor.iterator().hasNext()
                ? hardwareMap.voltageSensor.iterator().next() : null;
    }

    @Override
    public void loop() {
        long now = System.nanoTime();
        double loopSeconds = lastLoopNs == 0 ? 0.0
                : Math.min((now - lastLoopNs) * 1e-9, 0.1);
        lastLoopNs = now;

        // Exactly one fresh hub bulk snapshot and shooter velocity read.
        for (LynxModule hub : hubs) hub.clearBulkCache();
        double actualTicksPerSecond = shooterMotor.getVelocity();

        boolean shooterEnabled =
                gamepad1.right_trigger > JerryRiggedTeleOpConstants.SHOOTER_TRIGGER_THRESHOLD;
        double targetTicksPerSecond = shooterEnabled
                ? JerryRiggedTeleOpConstants.SHOOTER_TARGET_RPM
                    * JerryRiggedHardwareConstants.SHOOTER_TICKS_PER_REV / 60.0
                : 0.0;

        if (lastVoltageReadNs == 0 ||
                now - lastVoltageReadNs >= JerryRiggedTeleOpConstants.VOLTAGE_SAMPLE_INTERVAL_NS) {
            double voltage = voltageSensor == null ? 0.0 : voltageSensor.getVoltage();
            voltageScale = voltage > 0.1
                    ? JerryRiggedTeleOpConstants.SHOOTER_NOMINAL_VOLTAGE / voltage
                    : 1.0;
            lastVoltageReadNs = now;
        }

        double shooterPower;
        if (shooterEnabled) {
            shooterPower = speedController.calculate(
                    targetTicksPerSecond, actualTicksPerSecond, voltageScale, now);
        } else {
            speedController.reset();
            shooterPower = 0.0;
        }
        shooterMotor.setPower(shooterPower);

        // Preserve the original driver controls and continuous feed behavior.
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;
        if (gamepad1.b) drive.resetHeading();

        butterfly.update(forward, strafe, rotate, gamepad1.right_bumper);
        drive.driveFieldRelative(forward, strafe, rotate);
        mechanisms.updateTurret(gamepad1.right_bumper, gamepad1.left_bumper, loopSeconds);
        mechanisms.updateFeed(gamepad1.left_trigger, gamepad1.x);
        mechanisms.updateBlocker(gamepad1.a);

        double ticksPerRev = JerryRiggedHardwareConstants.SHOOTER_TICKS_PER_REV;
        telemetry.addData("Target RPM", targetTicksPerSecond * 60.0 / ticksPerRev);
        telemetry.addData("Actual RPM", "%.1f", actualTicksPerSecond * 60.0 / ticksPerRev);
        telemetry.addData("Profiled RPM", "%.1f", speedController.getProfiledTarget() * 60.0 / ticksPerRev);
        telemetry.addData("Shooter power", "%.3f", shooterPower);
        telemetry.addData("Loop ms", "%.1f", loopSeconds * 1000.0);
        telemetry.addData("Blocker open", mechanisms.isBlockerOpen() ? "yes" : "no");
        telemetry.update();
    }

    @Override
    public void stop() {
        // Stop all mechanisms, not just shooter.
        if (shooterMotor != null) shooterMotor.setPower(0.0);
        if (mechanisms != null) mechanisms.stop();
        if (drive != null) drive.stop();
        if (speedController != null) speedController.reset();
        if (hubs != null) {
            for (LynxModule hub : hubs) {
                hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
            }
        }
    }
}
