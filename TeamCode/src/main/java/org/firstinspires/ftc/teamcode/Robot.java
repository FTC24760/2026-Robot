package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.constants.Configurables;
import org.firstinspires.ftc.teamcode.constants.OperatorConstants;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.constants.ShotConstants;
import org.firstinspires.ftc.teamcode.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.constants.ShooterAssistConstants;
import org.firstinspires.ftc.teamcode.constants.TurretConstants;
import org.firstinspires.ftc.teamcode.constants.VisionConstants;
import org.firstinspires.ftc.teamcode.controls.DriverControls;
import org.firstinspires.ftc.teamcode.controls.OperatorControls;
import org.firstinspires.ftc.teamcode.sequences.FeedPulser;
import org.firstinspires.ftc.teamcode.sequences.ShootSequence;
import org.firstinspires.ftc.teamcode.sequences.AutoShootSequence;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.utils.Alliance;
import org.firstinspires.ftc.teamcode.utils.AsyncRpmLogger;
import org.firstinspires.ftc.teamcode.utils.HiveTracker;
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
    /** One-click shooting: spin up, open the blocker, feed, close. */
    public final ShootSequence shootSequence;
    public final AutoShootSequence autoShootSequence;
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

    // Auto-aim (off unless enableAutoAim() is called). The latest solved shot, or null
    // when there is no target.
    private ShotSolver.Solution shot;
    // Where the upward-facing CELL of our HIVE is on the field, from the camera and
    // odometry; null without auto-aim.
    private HiveTracker hiveTracker;
    // True while the turret should follow hiveTracker.
    private boolean aiming;
    // The driver can switch auto-aim off mid-match if the camera misbehaves.
    private boolean aimEnabled = true;
    private String aimNote = "";

    // Driver's intake on/off switch (teleop only).
    private boolean intakeOn;
    // Flower intake side-to-side shift (teleop only); null when not shifting.
    private Pose shiftCenter;

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
        shootSequence = new ShootSequence(shooter, intake);
        autoShootSequence = new AutoShootSequence(shooter, intake);

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
        try {
            vision = new Vision(opMode.hardwareMap, alliance);
        } catch (RuntimeException e) {
            vision = null;
            aimNote = "AUTO-AIM OFF: no Limelight \"" + VisionConstants.LIMELIGHT_NAME + "\" configured";
            return this;
        }

        hiveTracker = new HiveTracker();
        subsystems.add(vision);
        return this;
    }

    /** True if auto-aim was enabled and the Limelight was found. */
    public boolean hasAutoAim() {
        return vision != null;
    }

    /**
     * Tell odometry where the robot is (e.g. at the start of an auto). Use this rather
     * than drivetrain.setPose() so auto-aim's remembered HIVE position moves with it.
     */
    public void setPose(Pose pose) {
        Pose old = drivetrain.getPose();
        drivetrain.setPose(pose);
        if (hiveTracker != null) {
            hiveTracker.onPoseReset(old, pose);
        }
    }

    /** Make the robot's current position and heading the field origin. */
    public void resetPose() {
        Pose old = drivetrain.getPose();
        drivetrain.resetPose();
        if (hiveTracker != null) {
            hiveTracker.onPoseReset(old, new Pose(0, 0, 0));
        }
    }

    /** True while auto-aim has an up-facing CELL to aim at (seen, or remembered). */
    public boolean isAiming() {
        return aiming;
    }

    /** The shot auto-aim is using right now, or null if there is no target. */
    public ShotSolver.Solution getShot() {
        return shot;
    }

    /** True when auto-aim can see the upward-facing CELL and has a reachable shot for it. */
    public boolean hasShot() {
        return shot != null && shot.reachable;
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
            Configurables.SHOOTER_RPM = MathUtil.clamp(
                    Configurables.SHOOTER_RPM + rpmDirection * OperatorConstants.RPM_STEP,
                    OperatorConstants.MIN_RPM, OperatorConstants.MAX_RPM);
        }

        // Switches (driver or operator).
        if (controls.aimTogglePressed() || operator.aimTogglePressed()) {
            aimEnabled = !aimEnabled;
        }
        intakeOn = controls.intake() || operator.intake();
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
        } else if (controls.shootPressed() || operator.shootPressed()) {
            shootSequence.toggle(nowSeconds());
        }
        hood.setPosition(Configurables.HOOD_POSITION);
        shooter.clearShootRpm();
        runShooter(shootSequence.wantsShooter());
        if (!controls.shootHeld && !operator.shootHeld) {
            shootSequence.cancel();
        }
        shootSequence.update(nowSeconds());

        updateDrive();
        runTeleopTurret();

        // Intake and transfer, highest priority first.
        if (reversing) {
            runIntake(Intake.Mode.REVERSE);
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
        } else if (shooterArmed) {
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

        intake.setVoltageScale(voltageScale);
        intake.setMode(feedHeld ? Intake.Mode.OFF : mode);
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
        if (manualDirection != 0 || !aiming) {
            turret.move(manualDirection, loopSeconds);
            return;
        }

        double bearing = hiveTracker.turretBearingDeg(
                drivetrain.getPose(), drivetrain.getVelocity(), TurretConstants.AIM_LEAD_SECONDS);
        turret.pointAt(bearing, loopSeconds);
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

        hiveTracker.recordPose(drivetrain.getPoseTimeNs(), drivetrain.getPose());
        vision.update(nowNs);

        // Only aim at a CELL that is facing up; the other one cannot be scored in.
        Vision.Fix fix = vision.getFix();
        if (fix != null) {
            if (fix.cellUp) {
                hiveTracker.addSighting(fix, turret.getAngleDeg());
            } else {
                hiveTracker.sawCellDown(fix.sighting.cell);
            }
        }

        aiming = aimEnabled && hiveTracker.hasTarget(nowNs);
        shot = aiming
                ? ShotSolver.solveForCell(
                hiveTracker.distanceM(drivetrain.getPose()), hiveTracker.getTagHeightM())
                : null;

        if (hasShot()) {
            shooter.setShootRpm(shot.rpm);
            //hood.setPosition(shot.hoodPosition);
        } else {
            // No usable shot: fixed TARGET_RPM with the hood at its matching position.
            shooter.clearShootRpm();
            hood.setPosition(Configurables.HOOD_POSITION);
        }
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

        // Flower intake: with the arm down and the sticks still, shift side to side to
        // work the POLLEN out. Any stick input takes over at once.
        boolean sticksStill = Math.abs(controls.forward()) < 0.05
                && Math.abs(controls.strafe()) < 0.05
                && Math.abs(controls.rotate()) < 0.05;
        if (IntakeConstants.FLOWER_SHIFT_IN_TELEOP && flowerArm.isDown() && sticksStill) {
            if (shiftCenter == null) {
                shiftCenter = drivetrain.getPose();
            }
            drivetrain.shiftAround(shiftCenter, IntakeConstants.FLOWER_SHIFT_IN,
                    IntakeConstants.FLOWER_SHIFT_POWER, IntakeConstants.FLOWER_SHIFT_PERIOD_S,
                    nowSeconds());
            drivetrain.updateHold();
            return;
        }

        if (shiftCenter != null) {
            drivetrain.endShift();
            shiftCenter = null;
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

        telemetry.addData("Loop ms", "%.1f", loopSeconds * 1000.0);

        for (Subsystem subsystem : subsystems) {
            subsystem.addTelemetry(telemetry);
        }

        if (shooterAssist) {
            telemetry.addData("Feed held for RPM", feedHeld ? "yes" : "no");
            telemetry.addData("Feed pulses", feedPulser.getPulseCount());
        }

        telemetry.addData("Shoot", shootSequence.getStage());
        telemetry.addData("Intake held", intakeOn ? "yes" : "no");
        if (!aimNote.isEmpty()) {
            telemetry.addLine(aimNote);
        }

        if (vision != null) {
            if (!aimEnabled) {
                telemetry.addData("Auto-aim", "SWITCHED OFF");
            } else if (shot == null) {
                telemetry.addData("Auto-aim", "no upward CELL known, using fixed RPM");
            } else if (!shot.reachable) {
                telemetry.addData("Auto-aim", "OUT OF RANGE");
            } else {
                telemetry.addData("Auto-aim", "%.0f RPM at %.1f deg", shot.rpm, shot.launchAngleDeg);
            }
            if (aiming) {
                telemetry.addData("Aim source", hiveTracker.seesTarget(nowNs)
                        ? "camera" : "odometry (camera lost it)");
            }
        }

        if (rpmLogger != null) {
            telemetry.addData("RPM CSV", rpmLogger.getStatus());
            telemetry.addData("Dropped RPM samples", rpmLogger.getDroppedSamples());
        }

        telemetry.update();
    }
}