package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.constants.ShooterAssistConstants;
import org.firstinspires.ftc.teamcode.constants.VisionConstants;
import org.firstinspires.ftc.teamcode.controls.DriverControls;
import org.firstinspires.ftc.teamcode.sequences.ShootSequence;
import org.firstinspires.ftc.teamcode.subsystems.Butterfly;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.FlowerArm;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.utils.Alliance;
import org.firstinspires.ftc.teamcode.utils.AsyncRpmLogger;
import org.firstinspires.ftc.teamcode.utils.HubManager;
import org.firstinspires.ftc.teamcode.utils.MathUtil;
import org.firstinspires.ftc.teamcode.utils.ShotSolver;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

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
 *
 * An autonomous OpMode drives the same robot without a gamepad:
 *
 *   loop() -> robot.beginLoop();
 *             robot.drivetrain.update();      // follow the current path
 *             robot.shootSequence.update(robot.nowSeconds());
 *             robot.runShooter(true);         // spin up / hold speed
 *             robot.runTurret(0);             // auto-align if the HIVE is in view
 *             robot.runIntake(mode);          // feed, held back until the wheel is ready
 *             robot.endLoop();
 */
public class Robot {
    public final Drivetrain drivetrain;
    public final Butterfly butterfly;
    public final Intake intake;
    public final Turret turret;
    public final Shooter shooter;
    public final FlowerArm flowerArm;
    /** One-click shooting: spin up, open the blocker, feed, close. */
    public final ShootSequence shootSequence;
    // Only present after enableAutoAim(); null otherwise, so robots without a Limelight
    // or hood servo configured can still run the plain TeleOp.
    public Vision vision;
    public Hood hood;

    private final OpMode opMode;
    private final Telemetry telemetry;
    private final DriverControls controls;
    private final HubManager hubs;
    private final List<Subsystem> subsystems;

    private long nowNs;
    private long lastLoopNs;
    private double loopSeconds;
    // Nominal voltage / battery voltage, refreshed by HubManager a few times a second.
    private double voltageScale = 1.0;

    // Rapid-fire fixes (off unless enableShooterAssist() is called).
    private boolean shooterAssist;
    // Set by the first trigger pull, cleared by the stop button; idle spin runs while set.
    private boolean shooterArmed;
    private boolean feedHeld;

    // Auto-aim (off unless enableAutoAim() is called). The latest solved shot, or null
    // when the HIVE is not in view.
    private ShotSolver.Solution shot;
    // The upward-facing CELL of our HIVE, or null if it is not in view.
    private Vision.HiveTarget aimTarget;
    // The driver can switch auto-aim off mid-match if the camera misbehaves.
    private boolean aimEnabled = true;
    private String aimNote = "";

    // Driver's intake on/off switch (teleop only).
    private boolean intakeOn;

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
        flowerArm = new FlowerArm(opMode.hardwareMap);
        shootSequence = new ShootSequence(shooter, intake);

        subsystems = new ArrayList<>(
                Arrays.asList(drivetrain, butterfly, intake, turret, shooter, flowerArm));
    }

    /**
     * Record shooter RPM to a CSV in the background while the OpMode runs, and throttle
     * telemetry to keep the loop fast. Call before start().
     */
    public Robot enableShooterLogging() {
        rpmLogger = new AsyncRpmLogger();
        return this;
    }

    /**
     * Turn on the rapid-fire consistency fixes: recovery boost, feed gating, a working
     * integral and idle spin. Each is configured in ShooterAssistConstants.
     */
    public Robot enableShooterAssist() {
        shooterAssist = true;
        shooter.setAssistEnabled(true);
        return this;
    }

    /**
     * Use the Limelight to find our HIVE, turn the turret to it, and set shooter speed
     * and hood angle from the measured distance.
     *
     * If there is no Limelight in the robot configuration, auto-aim stays off and the
     * robot still runs with manual aiming and the fixed ShooterConstants.TARGET_RPM; a
     * warning is shown on telemetry. A missing hood servo is tolerated the same way.
     * When the upward-facing CELL is not in view, the shooter also falls back to
     * TARGET_RPM and the turret and hood stay where they were.
     */
    public Robot enableAutoAim(Alliance alliance) {
        try {
            vision = new Vision(opMode.hardwareMap, alliance);
        } catch (RuntimeException e) {
            vision = null;
            aimNote = "AUTO-AIM OFF: no Limelight \"" + VisionConstants.LIMELIGHT_NAME + "\" configured";
            return this;
        }

        hood = new Hood(opMode.hardwareMap);
        subsystems.add(vision);
        subsystems.add(hood);
        return this;
    }

    /** True if auto-aim was enabled and the Limelight was found. */
    public boolean hasAutoAim() {
        return vision != null;
    }

    /** The shot auto-aim is using right now, or null if the HIVE is not in view. */
    public ShotSolver.Solution getShot() {
        return shot;
    }

    /** True when auto-aim can see the upward-facing CELL and has a reachable shot for it. */
    public boolean hasShot() {
        return shot != null && shot.reachable;
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
        beginLoop();
        controls.update();

        // Switches.
        if (controls.aimTogglePressed()) {
            aimEnabled = !aimEnabled;
        }
        if (controls.intakeTogglePressed()) {
            intakeOn = !intakeOn;
        }
        if (controls.flowerTogglePressed()) {
            flowerArm.toggle();
        }
        // The blocker belongs to the shoot sequence while a shot is in progress.
        if (controls.blockerTogglePressed() && !shootSequence.isActive()) {
            intake.toggleBlocker();
        }

        // Shooting.
        if (controls.shooterStopPressed()) {
            shootSequence.cancel();
            shooterArmed = false;
        }
        if (controls.shootPressed()) {
            shootSequence.toggle(nowSeconds());
        }
        shootSequence.update(nowSeconds());
        runShooter(shootSequence.wantsShooter() || controls.manualSpin());

        updateDrive();
        runTurret(controls.turretDirection());

        // Intake and transfer, highest priority first.
        if (controls.reverse()) {
            runIntake(Intake.Mode.REVERSE);
        } else if (shootSequence.isFeeding()) {
            // Intake roller and transfer together, so every ball reaches the wheel.
            runIntake(Intake.Mode.INTAKE);
        } else if (controls.transferOnly()) {
            runIntake(Intake.Mode.TRANSFER_ONLY);
        } else if (intakeOn) {
            runIntake(Intake.Mode.INTAKE);
        } else {
            runIntake(Intake.Mode.OFF);
        }

        endLoop();
    }

    // ---- Building blocks, also used directly by autonomous OpModes ----

    /** Start of every loop: timing, sensor cache, battery voltage and auto-aim. */
    public void beginLoop() {
        nowNs = System.nanoTime();
        loopSeconds = lastLoopNs == 0
                ? 0
                : Math.min((nowNs - lastLoopNs) * 1e-9, RobotConstants.MAX_LOOP_SECONDS);
        lastLoopNs = nowNs;

        hubs.clearBulkCache();
        voltageScale = hubs.getVoltageScale(nowNs);
        updateAim();
    }

    /**
     * Run the shooter for this loop.
     *
     * @param shoot true to hold shooting speed; false to idle (with assist, once it has
     *              been spun up) or stop
     */
    public void runShooter(boolean shoot) {
        if (shoot) {
            shooterArmed = true;
            shooter.setState(Shooter.State.SHOOT);
        } else if (shooterAssist && ShooterAssistConstants.IDLE_SPIN_ENABLED && shooterArmed) {
            shooter.setState(Shooter.State.IDLE);
        } else {
            shooter.setState(Shooter.State.DISABLED);
        }

        shooter.update(nowNs, voltageScale);
    }

    /**
     * Run the intake and transfer for this loop. Call after runShooter() so feed gating
     * uses this loop's wheel speed.
     */
    public void runIntake(Intake.Mode mode) {
        // Feed gating: while shooting, do not push the next ball into a wheel that has
        // not recovered from the last one.
        boolean shooting = shooter.getState() == Shooter.State.SHOOT && intake.isBlockerOpen();
        boolean feeding = mode == Intake.Mode.INTAKE || mode == Intake.Mode.TRANSFER_ONLY;
        feedHeld = shooterAssist
                && ShooterAssistConstants.FEED_GATING_ENABLED
                && shooting
                && feeding
                && !shooter.isReadyToFeed();

        intake.setVoltageScale(voltageScale);
        intake.setMode(feedHeld ? Intake.Mode.OFF : mode);
    }

    /**
     * Aim the turret for this loop. With auto-aim on and the upward-facing CELL in
     * view, the turret follows it; otherwise, or while the driver is steering it by
     * hand, it moves only as directed.
     *
     * @param manualDirection +1, -1 or 0 from the driver; pass 0 in autonomous
     */
    public void runTurret(double manualDirection) {
        if (manualDirection != 0 || aimTarget == null) {
            turret.move(manualDirection, loopSeconds);
        } else if (VisionConstants.CAMERA_ON_TURRET) {
            turret.track(aimTarget.bearingDeg, loopSeconds);
        } else {
            turret.pointAt(aimTarget.bearingDeg, loopSeconds);
        }
    }

    /** Loop timestamp in seconds, for timing sequences. */
    public double nowSeconds() {
        return nowNs * 1e-9;
    }

    /** End of every loop: logging and telemetry. */
    public void endLoop() {
        logShooterRpm(nowNs);
        updateTelemetry(nowNs);
    }

    /** Seconds since the previous loop, capped so a stall cannot cause a jump. */
    public double getLoopSeconds() {
        return loopSeconds;
    }

    // Auto-aim: turn what the camera sees into a shooter speed and hood position.
    private void updateAim() {
        if (vision == null) {
            return;
        }

        vision.update(nowNs);
        Vision.HiveTarget target = vision.getTarget(nowNs);

        // Only aim at a CELL that is facing up; the other one cannot be scored in.
        aimTarget = aimEnabled && target != null && target.cellUp ? target : null;
        shot = aimTarget != null
                ? ShotSolver.solveForCell(aimTarget.distanceM, aimTarget.tagHeightM)
                : null;

        if (hasShot()) {
            shooter.setShootRpm(shot.rpm);
            hood.setPosition(shot.hoodPosition);
        } else {
            shooter.clearShootRpm();
        }
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

    private void updateDrive() {
        if (controls.resetPose()) {
            drivetrain.resetPose();
        }

        butterfly.setDown();
        drivetrain.driveFieldCentric(controls.forward(), controls.strafe(), controls.rotate());
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
                    shooter.getPower(),
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

        if (shooterAssist) {
            telemetry.addData("Feed held for RPM", feedHeld ? "yes" : "no");
        }

        telemetry.addData("Shoot", shootSequence.getStage());
        telemetry.addData("Intake switch", intakeOn ? "ON" : "off");
        if (!aimNote.isEmpty()) {
            telemetry.addLine(aimNote);
        }

        if (vision != null) {
            if (!aimEnabled) {
                telemetry.addData("Auto-aim", "SWITCHED OFF by driver");
            } else if (shot == null) {
                telemetry.addData("Auto-aim", "no upward CELL in view, using fixed RPM");
            } else if (!shot.reachable) {
                telemetry.addData("Auto-aim", "OUT OF RANGE");
            } else {
                telemetry.addData("Auto-aim", "%.0f RPM at %.1f deg", shot.rpm, shot.launchAngleDeg);
            }
        }

        if (rpmLogger != null) {
            telemetry.addData("RPM CSV", rpmLogger.getStatus());
            telemetry.addData("Dropped RPM samples", rpmLogger.getDroppedSamples());
        }

        telemetry.update();
    }
}
