package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.constants.AutoConstants;
import org.firstinspires.ftc.teamcode.constants.TurretConstants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.utils.Alliance;
import org.firstinspires.ftc.teamcode.utils.CellSide;
import org.firstinspires.ftc.teamcode.utils.EdgeDetector;
import org.firstinspires.ftc.teamcode.utils.PoseHandoff;

/**
 * Base for every autonomous OpMode. A subclass only names its alliance and its routine:
 *
 *   @Autonomous(name = "Auto: Rapid Fire Red")
 *   public class RapidFireRedAuto extends AutoOpMode {
 *       public RapidFireRedAuto() { super(Alliance.RED); }
 *       protected AutoRoutine buildRoutine(AutoContext ctx) { return RapidFire.build(ctx); }
 *   }
 *
 * During init, before the match countdown starts (gamepad 1):
 *   d-pad left / right  start preset (where the robot is placed; see StartPreset)
 *   A                   our slot: SOLO / FIRST / SECOND (agreed with the partner)
 *   bumpers             start delay for the SECOND slot, down / up
 *   left stick press    if FIRST's pre-loads do not tip the HIVE: LEAVE or STAY (agree with partner)
 *   d-pad up / down     more / fewer collection cycles after the pre-loads
 *   X / B               our LEFT / RIGHT CELL is up (override)
 *   Y                   back to automatic (Limelight if it sees the HIVE, else RIGHT,
 *                       which is how the manual sets up every match)
 * The alliance is the OpMode (Auto: Red / Auto: Blue), so the right TeleOp is preselected.
 * Choices are kept for the next match (AutoSelection).
 * The rules (G401) forbid touching the controls once the countdown begins.
 */
public abstract class AutoOpMode extends OpMode {
    private String lastLoggedStep = "";

    private final Alliance alliance;

    private Robot robot;
    private AutoContext ctx;
    private AutoRoutine routine;
    private boolean routineStarted;

    // Driver's choice during init; null means "let the Limelight decide".
    private CellSide chosenSide;
    private CellSide seenSide;

    private final EdgeDetector moreCycles = new EdgeDetector();
    private final EdgeDetector fewerCycles = new EdgeDetector();
    private final EdgeDetector nextPreset = new EdgeDetector();
    private final EdgeDetector previousPreset = new EdgeDetector();
    private final EdgeDetector nextSlot = new EdgeDetector();
    private final EdgeDetector moreDelay = new EdgeDetector();
    private final EdgeDetector lessDelay = new EdgeDetector();

    protected AutoOpMode(Alliance alliance) {
        this.alliance = alliance;
    }

    /** Build this OpMode's routine. Called once from init(). */
    protected abstract AutoRoutine buildRoutine(AutoContext ctx);

    @Override
    public void init() {
        robot = new Robot(this).enableShooterAssist();
        if (AutoConstants.USE_AUTO_AIM) {
            robot.enableAutoAim(alliance);
        }
        robot.pulseShoot = AutoConstants.PULSE_SHOOT;

        ctx = new AutoContext(robot, alliance, defaultSide());
        routine = buildRoutine(ctx);
        robot.setPose(routine.startPose(ctx));
        ctx.checkRouteConstants();
    }

    @Override
    public void init_loop() {
        if (moreCycles.rose(gamepad1.dpad_up)) {
            ctx.cycles = Math.min(ctx.cycles + 1, AutoConstants.MAX_CYCLES);
        }
        if (fewerCycles.rose(gamepad1.dpad_down)) {
            ctx.cycles = Math.max(ctx.cycles - 1, 0);
        }
        AutoSelection.cycles = ctx.cycles;

        boolean presetChanged = false;
        if (nextPreset.rose(gamepad1.dpad_right)) {
            AutoSelection.preset = AutoSelection.preset.next();
            presetChanged = true;
        }
        if (previousPreset.rose(gamepad1.dpad_left)) {
            AutoSelection.preset = AutoSelection.preset.previous();
            presetChanged = true;
        }
        if (presetChanged) {
            robot.setPose(routine.startPose(ctx));
        }

        if (nextSlot.rose(gamepad1.a)) {
            AutoSelection.slot = AutoSelection.slot.next();
            // The stronger robot takes FIRST and starts at A; the weaker SECOND starts at B.
            StartPreset forSlot = AutoSelection.presetForSlot();
            if (forSlot != AutoSelection.preset) {
                AutoSelection.preset = forSlot;
                robot.setPose(routine.startPose(ctx));
            }
        }
        if (moreDelay.rose(gamepad1.right_bumper)) {
            AutoSelection.startDelayS = Math.min(
                    AutoSelection.startDelayS + AutoConstants.START_DELAY_STEP_S,
                    AutoConstants.MAX_START_DELAY_S);
        }
        if (lessDelay.rose(gamepad1.left_bumper)) {
            AutoSelection.startDelayS = Math.max(
                    AutoSelection.startDelayS - AutoConstants.START_DELAY_STEP_S, 0);
        }

        if (gamepad1.x) {
            chosenSide = CellSide.LEFT;
        } else if (gamepad1.b) {
            chosenSide = CellSide.RIGHT;
        } else if (gamepad1.y) {
            chosenSide = null;
        }

        // Only read the camera here: nothing may move until the match starts.
        if (robot.hasAutoAim()) {
            robot.vision.update(System.nanoTime());
            CellSide seen = seenUpSide(System.nanoTime());
            if (seen != null) {
                seenSide = seen;
            }
        }

        telemetry.addLine(routine.getName() + " (" + alliance + ")");
        Pose start = AutoSelection.preset.pose(ctx);
        telemetry.addData("Start", "%s, %s: x %.1f  y %.1f  heading %.0f   (d-pad left / right)",
                AutoSelection.preset, AutoSelection.preset.where,
                start.x(), start.y(), Math.toDegrees(start.heading()));
        telemetry.addData("Slot", "%s   (A)", AutoSelection.slot);
        telemetry.addData("Start delay", AutoSelection.slot == AutoSelection.Slot.SECOND
                ? String.format("%.1f s   (bumpers)", AutoSelection.startDelayS)
                : "none (only for SECOND)");
        telemetry.addData("Cycles after pre-loads", "%d   (d-pad up / down)", ctx.cycles);
        addWarnings();
        telemetry.addData("Up CELL", "%s  [%s]", startSide(), sideSource());
        telemetry.addLine("X = LEFT CELL up, B = RIGHT CELL up, Y = automatic");
        telemetry.addLine("Hands off the controller once the countdown starts (G401)");
        for (String w : ctx.zoneWarnings(AutoSelection.preset, startSide())) {
            telemetry.addLine("!! Outside the shooting zone: " + w);
        }
        if (!ctx.sideWarnings().isEmpty()) {
            telemetry.addLine("!! Route goes past our side of the field (G402): " + ctx.sideWarnings());
        }
        telemetry.addData("Limelight sees", seenSide == null ? "nothing" : seenSide + " up");
        telemetry.addLine(robot.hasAutoAim() ? "Auto-aim on" : "Auto-aim OFF: fixed RPM, turret still");
        telemetry.update();
    }

    @Override
    public void start() {
        robot.start();
        ctx.setUpSide(startSide());
        robot.setPose(routine.startPose(ctx));
        robot.intake.setBlockerOpen(false);
        robot.flowerArm.setDown(false);
    }

    @Override
    public void loop() {
        robot.beginLoop();
        // Auto-aim set RPM and hood from the camera's distance in beginLoop(). Without a
        // camera target, use the distance from odometry to the CELL we believe is up, so
        // every auto shot still comes from the shot table.
        if (!robot.isAiming()) {
            robot.aimShotAt(odometryDistanceToUpCellIn());
        }
        robot.drivetrain.update();
        trackUpSide();

        if (!routineStarted) {
            ctx.markStart(robot.nowSeconds());
            routine.start(ctx);
            routineStarted = true;
        }

        routine.update(ctx);

        // Background jobs. Shooter before intake, so feed gating uses this loop's speed.
        robot.flowerArm.setDown(ctx.flowerDown);
        // Off (not idling) whenever we are not shooting, e.g. while driving, to save battery.
        // In the shooting zone it pre-spins to the table speed, as in TeleOp.
        robot.runShooterOrStop(ctx.spinUp || robot.shootSequence.wantsShooter()
                || robot.zoneWantsPrespin());
        robot.shootSequence.update(robot.nowSeconds());
        aimTurret();
        // The TeleOp "ready to fire" decision; Steps.toZoneAndReady fires on it.
        double camDist = robot.targetDistanceIn();
        double camOff = robot.targetOffAxisDeg();
        boolean camera = !Double.isNaN(camDist) && !Double.isNaN(camOff);
        robot.updateAutoZone(camera ? camDist : ctx.upCellDistanceIn(),
                camera ? camOff : ctx.upCellOffAxisDeg(), ctx.upStateKnown(),
                ctx.intakeMode == Intake.Mode.INTAKE);
        boolean pulsing = robot.pulseActive(
                robot.shootSequence.inFeedingStage() && robot.aimReady());
        robot.runIntake(pulsing ? robot.pulseFeedMode()
                : robot.shootSequence.isFeeding() ? Intake.Mode.INTAKE
                : robot.shootSequence.isActive() ? Intake.Mode.OFF : ctx.intakeMode);

        // For TeleOp to start from (the auto can be stopped at any loop).
        PoseHandoff.save(robot.drivetrain.getPose(), alliance, ctx.upSide);

        String stepName = routine.currentStep();
        if (!stepName.equals(lastLoggedStep)) {
            // Times to fill in the planned durations in AutoConstants (AVG_SPEED_IN_PER_S, BURST_S ...).
            RobotLog.ii("AutoStep", "%5.1f s left: %s", ctx.timeLeft(), stepName);
            lastLoggedStep = stepName;
        }
        telemetry.addData("Auto step", stepName);
        double readAge = robot.upStateAgeS();
        telemetry.addData("Up CELL", "%s (camera read %s)", ctx.upSide,
                Double.isNaN(readAge) ? "never" : String.format("%.0f s ago", readAge));
        telemetry.addData("Zone", robot.isReadyToFire() ? "READY" : robot.isInZone() ? "in zone"
                : "out of zone");
        telemetry.addData("Time left s", "%.1f", ctx.timeLeft());
        robot.endLoop();
    }

    @Override
    public void stop() {
        if (routine != null && routineStarted) {
            routine.stop(ctx);
        }
        if (robot != null && routineStarted) {
            PoseHandoff.save(robot.drivetrain.getPose(), alliance, ctx.upSide);
        }
        if (robot != null) {
            robot.stop();
        }
    }

    // Selections that look like a mistake.
    private void addWarnings() {
        StartPreset forSlot = AutoSelection.presetForSlot();
        if (forSlot != AutoSelection.preset) {
            telemetry.addLine("!! " + AutoSelection.slot + " normally starts at " + forSlot
                    + ", not " + AutoSelection.preset);
        }
        if (AutoSelection.slot == AutoSelection.Slot.SECOND && AutoSelection.startDelayS <= 0) {
            telemetry.addLine("!! SECOND slot with no delay: we will shoot at the same time as the partner");
        }
        // Rough budget: delay, one pre-load burst, the wait before parking, and the park
        // drive itself. If it does not fit, FullAuto skips the park drive.
        double burstS = AutoConstants.SHOOT_TIMEOUT_S / 2 + AutoConstants.SHOOT_FEED_S;
        double needS = AutoSelection.effectiveDelayS() + burstS
                + AutoConstants.PARK_DELAY_S + AutoConstants.PARK_MIN_TIME_LEFT_S;
        if (AutoConstants.PARK_ENABLED && needS > AutoConstants.AUTO_LENGTH_S) {
            telemetry.addLine(String.format(
                    "!! Delay too long: about %.0f s needed of %.0f, PARK may be skipped",
                    needS, AutoConstants.AUTO_LENGTH_S));
        }
    }

    private CellSide defaultSide() {
        return AutoConstants.START_UP_SIDE;
    }

    private CellSide startSide() {
        if (chosenSide != null) {
            return chosenSide;
        }
        if (AutoConstants.DETECT_UP_SIDE && seenSide != null) {
            return seenSide;
        }
        return defaultSide();
    }

    private String sideSource() {
        if (chosenSide != null) {
            return "driver";
        }
        if (AutoConstants.DETECT_UP_SIDE && seenSide != null) {
            return "Limelight";
        }
        return "default";
    }

    // Follow the HIVE during the auto, e.g. after we or a partner tip it.
    private void trackUpSide() {
        if (!AutoConstants.DETECT_UP_SIDE || !robot.hasAutoAim()) {
            return;
        }
        // A fresh frame of one of our CELLS, up or down, counts as a read. The two CELLS
        // are joined, so seeing one facing down also says the other is up.
        Vision.Fix fix = robot.vision.getFix();
        if (fix != null && fix.sighting.cell.alliance == alliance) {
            ctx.lastCellReadS = ctx.now();
            if (!fix.cellUp) {
                ctx.setUpSide(CellSide.of(alliance, fix.sighting.cell.side).other());
            }
        }
        CellSide seen = seenUpSide(System.nanoTime());
        if (seen != null) {
            ctx.setUpSide(seen);
        }
    }

    // The turret follows the Limelight's target when it has one. Without one it points
    // at the CELL we believe is up, from odometry and that CELL's aim point, so a shot
    // never waits on the camera. LOCK_TURRET_FORWARD holds it straight ahead instead.
    private void aimTurret() {
        if (AutoConstants.LOCK_TURRET_FORWARD) {
            robot.turret.setPosition(TurretConstants.FORWARD_POSITION);
            return;
        }
        if (robot.isAiming()) {
            robot.runTurret(0);
            return;
        }
        Pose cell = upCellPose();
        Pose me = robot.drivetrain.getPose();
        double bearing = Math.atan2(cell.y() - me.y(), cell.x() - me.x()) - me.heading();
        bearing = Math.atan2(Math.sin(bearing), Math.cos(bearing));
        robot.turret.pointAt(Math.toDegrees(bearing), robot.getLoopSeconds());
    }

    // The aim point (middle) of the CELL we believe is up.
    private Pose upCellPose() {
        return ctx.upSide == CellSide.LEFT
                ? ctx.pose(AutoConstants.LEFT_CELL_X, AutoConstants.LEFT_CELL_Y, 0)
                : ctx.pose(AutoConstants.RIGHT_CELL_X, AutoConstants.RIGHT_CELL_Y, 0);
    }

    private double odometryDistanceToUpCellIn() {
        Pose cell = upCellPose();
        Pose me = robot.drivetrain.getPose();
        return Math.hypot(cell.x() - me.x(), cell.y() - me.y());
    }

    private CellSide seenUpSide(long nowNs) {
        Vision.HiveTarget target = robot.vision.getTarget(nowNs);
        if (target == null || !target.cellUp || target.cell.alliance != alliance) {
            return null;
        }
        return CellSide.of(alliance, target.cell.side);
    }
}
