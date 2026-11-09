package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.controls.DriverControls;
import org.firstinspires.ftc.teamcode.subsystems.Butterfly;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.utils.AsyncRpmLogger;
import org.firstinspires.ftc.teamcode.utils.HubManager;
import org.firstinspires.ftc.teamcode.utils.MathUtil;

/**
 * The whole robot. An OpMode only has to forward its lifecycle:
 *
 *   init()  -> robot = new Robot(this);
 *   start() -> robot.start();
 *   loop()  -> robot.update();
 *   stop()  -> robot.stop();
 *
 * Robot wires driver input (DriverControls) to the subsystems. Hardware details live in
 * the subsystems, tunable numbers in constants/, and button bindings in DriverControls.
 */
public class Robot {
    public final Drivetrain drivetrain;
    public final Butterfly butterfly;
    public final Intake intake;
    public final Turret turret;
    public final Shooter shooter;

    private final OpMode opMode;
    private final Telemetry telemetry;
    private final DriverControls controls;
    private final HubManager hubs;
    private final Subsystem[] subsystems;

    private long lastLoopNs;
    private double loopSeconds;

    // Shooter diagnostics (off unless enableShooterLogging() is called).
    private AsyncRpmLogger rpmLogger;
    private long startNs;
    private long lastRpmLoggedNs;
    private long lastTelemetryNs;

    /** Maps and configures all hardware. Construct this from OpMode.init(). */
    public Robot(OpMode opMode) {
        this.opMode = opMode;
        this.telemetry = opMode.telemetry;

        hubs = new HubManager(opMode.hardwareMap);
        controls = new DriverControls(opMode);

        drivetrain = new Drivetrain(opMode.hardwareMap);
        butterfly = new Butterfly(opMode.hardwareMap);
        intake = new Intake(opMode.hardwareMap);
        turret = new Turret(opMode.hardwareMap);
        shooter = new Shooter(opMode.hardwareMap);

        subsystems = new Subsystem[]{drivetrain, butterfly, intake, turret, shooter};
    }

    /**
     * Record shooter RPM to a CSV in the background while the OpMode runs, and throttle
     * telemetry to keep the loop fast. Call before start().
     */
    public Robot enableShooterLogging() {
        rpmLogger = new AsyncRpmLogger();
        return this;
    }

    public void start() {
        startNs = System.nanoTime();
        lastRpmLoggedNs = 0;
        lastTelemetryNs = 0;

        if (rpmLogger != null) {
            rpmLogger.start(opMode.hardwareMap.appContext.getExternalFilesDir(null));
        }
    }

    /** One iteration of teleop. Call from OpMode.loop(). */
    public void update() {
        long nowNs = System.nanoTime();
        loopSeconds = lastLoopNs == 0
                ? 0
                : Math.min((nowNs - lastLoopNs) * 1e-9, RobotConstants.MAX_LOOP_SECONDS);
        lastLoopNs = nowNs;

        hubs.clearBulkCache();
        controls.update();

        updateShooter(nowNs);
        updateDrive();
        updateTurret();
        updateIntake();

        logShooterRpm(nowNs);
        updateTelemetry(nowNs);
    }

    public void stop() {
        for (Subsystem subsystem : subsystems) {
            subsystem.stop();
        }

        if (rpmLogger != null) {
            rpmLogger.stopAsync();
        }

        hubs.stop();
    }

    private void updateShooter(long nowNs) {
        shooter.setState(controls.shoot() ? Shooter.State.SHOOT : Shooter.State.DISABLED);
        shooter.update(nowNs, hubs.getVoltageScale(nowNs));
    }

    private void updateDrive() {
        double forward = controls.forward();
        double strafe = controls.strafe();
        double rotate = controls.rotate();

        if (controls.resetYaw()) {
            drivetrain.resetYaw();
        }

        butterfly.update(forward, strafe, rotate, controls.butterflyInvert());
        drivetrain.driveFieldCentric(forward, strafe, rotate);
    }

    private void updateTurret() {
        turret.move(controls.turretDirection(), loopSeconds);
    }

    private void updateIntake() {
        if (controls.intake()) {
            intake.setMode(Intake.Mode.INTAKE);
        } else if (controls.transferOnly()) {
            intake.setMode(Intake.Mode.TRANSFER_ONLY);
        } else {
            intake.setMode(Intake.Mode.OFF);
        }

        if (controls.blockerTogglePressed()) {
            intake.toggleBlocker();
        }
    }

    private void logShooterRpm(long nowNs) {
        if (rpmLogger == null) {
            return;
        }

        long periodNs = MathUtil.msToNs(RobotConstants.RPM_LOG_PERIOD_MS);

        if (lastRpmLoggedNs == 0 || nowNs - lastRpmLoggedNs >= periodNs) {
            rpmLogger.record(
                    (nowNs - startNs) * 1e-9,
                    shooter.getActualRpm(),
                    shooter.getTargetRpm(),
                    intake.isFeeding()
            );
            lastRpmLoggedNs = nowNs;
        }
    }

    private void updateTelemetry(long nowNs) {
        // Telemetry is only throttled while logging, where loop time matters most.
        if (rpmLogger != null) {
            long periodNs = MathUtil.msToNs(RobotConstants.DIAGNOSTIC_TELEMETRY_PERIOD_MS);

            if (lastTelemetryNs != 0 && nowNs - lastTelemetryNs < periodNs) {
                return;
            }
            lastTelemetryNs = nowNs;
        }

        telemetry.addData("Loop ms", "%.1f", loopSeconds * 1000.0);

        for (Subsystem subsystem : subsystems) {
            subsystem.addTelemetry(telemetry);
        }

        if (rpmLogger != null) {
            telemetry.addData("RPM CSV", rpmLogger.getStatus());
            telemetry.addData("Dropped RPM samples", rpmLogger.getDroppedSamples());
        }

        telemetry.update();
    }
}
