package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.aim.AimController;
import org.firstinspires.ftc.teamcode.constants.Configurables;
import org.firstinspires.ftc.teamcode.constants.OperatorConstants;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.ShootGateConstants;
import org.firstinspires.ftc.teamcode.constants.ShotConstants;
import org.firstinspires.ftc.teamcode.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.constants.ShooterAssistConstants;
import org.firstinspires.ftc.teamcode.constants.TurretConstants;
import org.firstinspires.ftc.teamcode.constants.VisionConstants;
import org.firstinspires.ftc.teamcode.controls.DriverControls;
import org.firstinspires.ftc.teamcode.controls.OperatorControls;
import org.firstinspires.ftc.teamcode.sequences.FeedPulser;
import org.firstinspires.ftc.teamcode.sequences.ShootSequence;
import org.firstinspires.ftc.teamcode.sequences.ShootZone;
import org.firstinspires.ftc.teamcode.utils.CellSide;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.utils.Alliance;
import org.firstinspires.ftc.teamcode.utils.AsyncRpmLogger;
import org.firstinspires.ftc.teamcode.utils.HiveTracker;
import org.firstinspires.ftc.teamcode.utils.HubManager;
import org.firstinspires.ftc.teamcode.utils.MathUtil;
import org.firstinspires.ftc.teamcode.utils.ShotTable;

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
 * Robot wires driver input (DriverControls, gamepad 1) and operator input
 * (OperatorControls, gamepad 2) to the subsystems. Hardware details live in the
 * subsystems, tunable numbers in constants/, and button bindings in the controls classes.
 *
 * An autonomous OpMode drives the same robot without a gamepad:
 *
 *   loop() -> robot.beginLoop();
 *             robot.drivetrain.update();      // follow the current path
 *             robot.runShooter(true);         // spin up / hold speed
 *             robot.shootSequence.update(robot.nowSeconds());
 *             robot.runTurret(0);             // auto-align if the HIVE is in view
 *             robot.runIntake(mode);          // feed, held back until the wheel is ready
 *             robot.endLoop();
 */
public class Robot {
    public final Drivetrain drivetrain;
    public final Intake intake;
    public final Turret turret;
    public final Shooter shooter;
    public final FlowerArm flowerArm;
    /** Shooting for TeleOp and Auto: spin up, open the blocker, feed through the fire gate, close. */
    public final ShootSequence shootSequence;
    // Only present after enableAutoAim(); null otherwise, so robots without a Limelight
    // configured can still run.
    public Vision vision;
    // Always present; does nothing if the hood servo is not configured.
    public final Hood hood;

    private final OpMode opMode;
    private final Telemetry telemetry;
    private final DriverControls controls;
    private final OperatorControls operator;
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
    private final FeedPulser feedPulser = new FeedPulser();
    // Off unless enableRapidFire() is called: feed on a fixed beat instead of waiting
    // for the wheel to recover after each ball.
    private boolean rapidFire;

    // Pose + camera -> turret, wheel speed, hood, and the aim half of the fire gate.
    private AimController aim;
    // TeleOp shooting zone and the "ready to fire" signal.
    private ShootZone zone;
    private boolean wasReady;
    /** Feed in timed push/retract pulses while shooting. Operator X in teleop. */
    public boolean pulseShoot = true;
    private double pulseStartS = -1;

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
        operator = new OperatorControls(opMode);

        drivetrain = new Drivetrain(opMode.hardwareMap);
        intake = new Intake(opMode.hardwareMap);
        turret = new Turret(opMode.hardwareMap);
        shooter = new Shooter(opMode.hardwareMap);
        shooter.alwaysFullSpeed = true;

        flowerArm = new FlowerArm(opMode.hardwareMap);
        hood = new Hood(opMode.hardwareMap);
        shootSequence = new ShootSequence(shooter, intake, this::aimReady);
        zone = new ShootZone(opMode.gamepad1, opMode.gamepad2);
        aim = new AimController(drivetrain, shooter, hood, turret);

        subsystems = new ArrayList<>(
                Arrays.asList(drivetrain, intake, turret, shooter, flowerArm, hood));
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
     * robot still runs with manual aiming and the fixed Configurables.SHOOTER_RPM; a
     * warning is shown on telemetry. A missing hood servo is tolerated the same way.
     * When the upward-facing CELL is not in view, the shooter also falls back to
     * TARGET_RPM and the turret and hood stay where they were.
     */
    public Robot enableAutoAim(Alliance alliance) {
        vision = aim.enable(opMode.hardwareMap, alliance);
        if (vision != null) {
            subsystems.add(vision);
        }
        return this;
    }

    /** True if auto-aim was enabled and the Limelight was found. */
    /** TeleOp: remember the CELL for good and switch to the other after a down-read. */
    public Robot enableTeleOpMemory() {
        aim.enableTeleOpMemory();
        return this;
    }

    public boolean hasAutoAim() {
        return aim.hasAutoAim();
    }

    /**
     * Tell odometry where the robot is (e.g. at the start of an auto). Use this rather
     * than drivetrain.setPose() so auto-aim's remembered HIVE position moves with it.
     */
    public void setPose(Pose pose) {
        Pose old = drivetrain.getPose();
        drivetrain.setPose(pose);
        aim.onPoseReset(old, pose);
    }

    /** Make the robot's current position and heading the field origin. */
    public void resetPose() {
        Pose old = drivetrain.getPose();
        drivetrain.resetPose();
        aim.onPoseReset(old, new Pose(0, 0, 0));
    }

    /** True while auto-aim has an up-facing CELL to aim at (seen, or remembered). */
    public boolean isAiming() {
        return aim.isAiming();
    }

    /** The table shot in use right now, or null if there is no target distance. */
    public ShotTable.Shot getShot() {
        return aim.getShot();
    }

    /** True when RPM and hood come from the shot table (a target distance is known). */
    public boolean hasShot() {
        return aim.getShot() != null;
    }

    /**
     * The aim half of the fire gate (the wheel-speed half is in Shooter): true when a ball
     * may be fed now. While the turret auto-aims it must have settled on the target; the
     * robot must be slow enough. When the turret is aimed by hand, that is the driver's
     * call and only the speed limit applies. See ShootGateConstants.
     */
    public boolean aimReady() {
        return aim.aimReady();
    }

    /** True while the target distance is inside the calibrated shooting zone (TeleOp). */
    public boolean isInZone() {
        return zone.isInZone();
    }

    /** True while in the zone with the aim ready and (when pre-spinning) the wheel at speed. */
    public boolean isReadyToFire() {
        return zone.isReadyToFire();
    }

    /**
     * Auto: run the same shooting-zone and "ready to fire" decision as TeleOp, without the
     * driver feedback (the auto fires by itself). Call once per loop after the turret is aimed.
     *
     * @param distanceIn   distance to the up CELL (camera, else odometry)
     * @param offAxisDeg   degrees off the CELL's opening axis
     * @param upStateKnown a recent camera read of which CELL is up, since the last burst
     * @param intaking     the intake is running (counts as reloading)
     */
    public void updateAutoZone(double distanceIn, double offAxisDeg, boolean upStateKnown,
                               boolean intaking) {
        zone.setFeedback(false);
        zone.updateReloadArming(shootSequence.isActive(), shootSequence.isFeeding(), false,
                intaking, loopSeconds);
        zone.update(distanceIn, offAxisDeg, aimReady(), shooter.isReadyToFeed(),
                shootSequence.isActive(), upStateKnown, nowSeconds());
        if (zone.isReadyToFire() && !wasReady) {
            RobotLog.ii("ShootZone", "AUTO READY, %.0f in, %.0f deg off axis", distanceIn,
                    offAxisDeg);
        }
        wasReady = zone.isReadyToFire();
    }

    /** True if the wheel should be held at the table speed (in the zone, not reloading). */
    public boolean zoneWantsPrespin() {
        return zone.wantsPrespin();
    }

    /** Seconds since the camera or driver last said which CELL is up; NaN if never. */
    public double upStateAgeS() {
        return aim.upStateAgeS(nowNs);
    }

    /** Aim at this CELL until the camera says otherwise (start of TeleOp, or the auto's belief). */
    public void syncUpSide(CellSide side) {
        aim.syncUpSide(side, nowNs);
    }

    /** Why aimReady() is false, for telemetry; null when ready. */
    public String aimBlockReason() {
        return aim.aimBlockReason();
    }

    /** Degrees the shot to the up CELL is off the CELL's opening axis, NaN without a target. */
    public double targetOffAxisDeg() {
        return aim.targetOffAxisDeg();
    }

    /**
     * Distance from the robot centre to the up CELL auto-aim is tracking, inches, or NaN
     * without a target.
     */
    public double targetDistanceIn() {
        return aim.targetDistanceIn();
    }

    /** Set wheel speed and hood for a shot from this distance; see AimController. */
    public void aimShotAt(double distanceIn) {
        aim.aimShotAt(distanceIn);
    }

    /**
     * Shoot on a fixed beat instead of waiting for the wheel to recover after each ball
     * (the default, "staggered").
     * Faster, but later balls can fall short if the beat is too quick for the wheel.
     * Needs enableShooterAssist(); timings are the RAPID_ values in
     * ShooterAssistConstants.
     */
    public Robot enableRapidFire() {
        rapidFire = true;
        return this;
    }

    public void start() {
        // Put the hood somewhere known; without a Limelight nothing else ever moves it.
        hood.setPosition(Configurables.HOOD_POSITION);

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
        operator.update();
        int rpmDirection = (controls.rpmUpPressed() || operator.rpmUpPressed() ? 1 : 0)
                - (controls.rpmDownPressed() || operator.rpmDownPressed() ? 1 : 0);
        if (rpmDirection != 0) {
            // A trim on top of the shot table (or the fixed speed): takes effect next loop.
            aim.adjustRpmTrim(rpmDirection);
        }

        // Switches (driver or operator).
        if (controls.aimTogglePressed() || operator.aimTogglePressed()) {
            aim.toggleEnabled();
        }
        intakeOn = controls.intake() || operator.intake();
        if (operator.pulseTogglePressed()) {
            pulseShoot = !pulseShoot;
        }
        if (controls.flipCellPressed() || operator.flipCellPressed()) {
            aim.flipUpCell(nowNs);
        }
        if (controls.flowerTogglePressed() || operator.flowerTogglePressed()) {
            flowerArm.toggle();
        }
        flowerArm.refresh();
        // A reverse request cancels shooting; the gate remains closed during collection.
        boolean reversing = controls.reverse() || operator.reverse();
        boolean stopShot = controls.shooterStopPressed() || reversing;
        if (stopShot) {
            shootSequence.cancel();
            shooterArmed = false;
            // The stop button also stops the in-zone pre-spin until the robot leaves the zone.
            zone.onStopPressed();
        } else if (controls.shootPressed() || operator.shootPressed()) {
            shootSequence.toggle(nowSeconds());
        }
        // RPM and hood were set by auto-aim in beginLoop() (shot table, or fixed fallback).
        // In the shooting zone the wheel is kept at the table speed, ready for the button.
        runShooter(shootSequence.wantsShooter() || zone.wantsPrespin());
        if (!controls.shootHeld && !operator.shootHeld) {
            shootSequence.cancel();
        }
        shootSequence.update(nowSeconds());
        zone.updateReloadArming(shootSequence.isActive(), shootSequence.isFeeding(), stopShot,
                intakeOn && !reversing, loopSeconds);

        updateDrive();
        runTeleopTurret();
        if (zone.takeShotEnded()) {
            aim.noteShotFed(nowNs);
        }
        zone.update(targetDistanceIn(), targetOffAxisDeg(), aimReady(), shooter.isReadyToFeed(),
                shootSequence.isActive(), aim.upStateConfirmed(nowNs), nowSeconds());
        if (zone.isReadyToFire() && !wasReady) {
            // To set UPSIDE_MAX_AGE_S: how old was the up/down read each time we signalled?
            RobotLog.ii("ShootZone", "READY, up/down read age %.1f s", aim.upStateAgeS(nowNs));
        }
        wasReady = zone.isReadyToFire();

        // Intake and transfer, highest priority first.
        boolean pulsing = pulseActive(shootSequence.inFeedingStage() && aimReady());
        if (reversing) {
            runIntake(Intake.Mode.REVERSE);
        } else if (pulsing) {
            runIntake(pulseFeedMode());
        } else if (shootSequence.isFeeding()) {
            runIntake(Intake.Mode.INTAKE);
        } else if (shootSequence.isActive()) {
            runIntake(Intake.Mode.OFF);
        } else if (intakeOn) {
            runIntake(Intake.Mode.INTAKE);
        } else {
            runIntake(Intake.Mode.OFF);
        }

        endLoop();
    }

    /**
     * True while pulse shoot should drive the transfer: it is switched on and a shoot
     * sequence is in its feeding stage. Call once per loop; restarts the timer otherwise.
     */
    public boolean pulseActive(boolean feedingStage) {
        boolean on = pulseShoot && feedingStage;
        if (!on) {
            pulseStartS = -1;
        }
        return on;
    }

    /**
     * Pulse shoot, on a timer: push one ball in, pull the transfer back a little so the
     * next ball waits off the wheel, repeat for PULSE_BALLS balls, then keep pushing.
     */
    public Intake.Mode pulseFeedMode() {
        if (pulseStartS < 0) {
            pulseStartS = nowSeconds();
        }
        double cycle = Configurables.PULSE_PUSH_SECONDS + Configurables.PULSE_RETRACT_SECONDS;
        double t = nowSeconds() - pulseStartS;
        if (t >= cycle * Configurables.PULSE_BALLS || t % cycle < Configurables.PULSE_PUSH_SECONDS) {
            return Intake.Mode.INTAKE;
        }
        return Intake.Mode.RETRACT;
    }

    // Operator right stick aims the turret; driver bumpers are exclusively intake/outtake.
    private void runTeleopTurret() {
        runTurret(operator.turret());
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
        aim.update(nowNs);
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
        } else if (shooterArmed) {
            shooter.setState(Shooter.State.IDLE);
        } else {
            shooter.setState(Shooter.State.DISABLED);
        }

        shooter.update(nowNs, voltageScale);
    }

    /**
     * Run the shooter for this loop with no idle: shooting speed or fully off. The auto
     * uses this so the wheel does not draw battery while driving between shots.
     */
    public void runShooterOrStop(boolean shoot) {
        shooter.setState(shoot ? Shooter.State.SHOOT : Shooter.State.DISABLED);
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
        boolean gated = shooterAssist
                && ShooterAssistConstants.FEED_GATING_ENABLED
                && shooting
                && feeding;

        // Once the earlier balls have gone, stop pulsing: the last ball needs a steady push.
        int finalPushAfter = ShooterAssistConstants.FEED_FINAL_PUSH_AFTER_PULSES;
        boolean finalPush = finalPushAfter > 0
                && feedPulser.getPulseCount() >= finalPushAfter
                && !feedPulser.isPulsing();

        if (!gated) {
            feedPulser.reset();
            feedHeld = false;
        } else if (finalPush) {
            feedHeld = !shooter.isReadyToFeed();
        } else if (rapidFire) {
            // Fixed beat: the wheel must be ready for the first ball, then no waiting.
            boolean mayStart = feedPulser.getPulseCount() > 0 || shooter.isReadyToFeed();
            feedHeld = !feedPulser.update(
                    nowSeconds(),
                    mayStart,
                    ShooterAssistConstants.RAPID_PULSE_ON_MS,
                    ShooterAssistConstants.RAPID_PULSE_OFF_MS);
        } else if (ShooterAssistConstants.FEED_PULSE_ENABLED) {
            // Staggered: one ball per pulse, and a pulse only starts when the wheel is ready.
            boolean onSpeed = shooter.getActualRpm()
                    >= shooter.getShootRpm() - ShooterAssistConstants.FEED_PULSE_START_LOW_RPM;
            feedHeld = !feedPulser.update(
                    nowSeconds(),
                    shooter.isReadyToFeed() && onSpeed,
                    ShooterAssistConstants.FEED_PULSE_ON_MS,
                    ShooterAssistConstants.FEED_PULSE_MIN_OFF_MS);
        } else {
            feedHeld = !shooter.isReadyToFeed();
        }

        // Takes effect on the next shooter update, one loop later.
        shooter.setFeedingBurst(gated);

        // After a burst (shoot released, auto shot done) the slow blocker is still part
        // open for a moment: do not push balls toward it until it has closed. Backwards
        // modes (REVERSE, RETRACT) pull away from it and are allowed.
        boolean closingHold = feeding && intake.isBlockerClosing();

        intake.setVoltageScale(voltageScale);
        intake.setMode(feedHeld || closingHold ? Intake.Mode.OFF : mode);
    }

    /**
     * Aim the turret for this loop. With auto-aim on and a target (seen now, or
     * remembered and followed with odometry), the turret points at it from the latest
     * pose; otherwise, or while the driver is steering it by hand, it moves only as
     * directed. Call after the drivetrain has updated this loop.
     *
     * @param manualDirection +1, -1 or 0 from the driver; pass 0 in autonomous
     */
    public void runTurret(double manualDirection) {
        aim.runTurret(manualDirection, loopSeconds);
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

    public void stop() {
        shootSequence.cancel();
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
            resetPose();
        }

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

        // TeleOp only (Auto shoots by itself): first, so the driver sees it at a glance.
        if (zone.isFeedbackShown()) {
            if (zone.isReadyToFire()) {
                telemetry.addLine(">>> READY TO FIRE <<<");
            } else if (!zone.isInZone()) {
                telemetry.addLine(Double.isNaN(targetDistanceIn())
                        ? "Not ready: no target"
                        : targetOffAxisDeg() > ShootGateConstants.ZONE_MAX_OFF_AXIS_DEG
                        ? String.format("Not ready: bad angle (%.0f deg off the opening)", targetOffAxisDeg())
                        : "Not ready: outside shooting zone");
 } else if (zone.isUpStateUnconfirmed()) {
                telemetry.addLine("UP CELL UNCONFIRMED (no recent camera read): D-pad right flips"
                        + " the cell, or shoot anyway");
            } else {
                String blocked = aimBlockReason();
                telemetry.addLine("In zone, not ready: " + (zone.isReloading() ? "reloading (run intake or leave the zone)"
                        : blocked != null ? blocked : "wheel spinning up"));
            }
        }

        telemetry.addData("Loop ms", "%.1f", loopSeconds * 1000.0);

        for (Subsystem subsystem : subsystems) {
            subsystem.addTelemetry(telemetry);
        }

        if (shooterAssist) {
            telemetry.addData("Feed held for RPM", feedHeld ? "yes" : "no");
            telemetry.addData("Feed pulses", feedPulser.getPulseCount());
        }

        telemetry.addData("Shoot", shootSequence.getStage());
        String blocked = aimBlockReason();
        if (shootSequence.isActive() && blocked != null) {
            telemetry.addData("Fire gate", "holding: " + blocked);
        }
        telemetry.addData("Pulse shoot (op X)", pulseShoot ? "ON" : "off");
        telemetry.addData("Intake held", intakeOn ? "yes" : "no");
        aim.addTelemetry(telemetry, nowNs);

        if (rpmLogger != null) {
            telemetry.addData("RPM CSV", rpmLogger.getStatus());
            telemetry.addData("Dropped RPM samples", rpmLogger.getDroppedSamples());
        }

        telemetry.update();
    }
}