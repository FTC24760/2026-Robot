package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.constants.AutoConstants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.utils.Alliance;
import org.firstinspires.ftc.teamcode.utils.CellSide;
import org.firstinspires.ftc.teamcode.utils.EdgeDetector;

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
 *   d-pad up / down   more / fewer collection cycles after the pre-loads
 *   X / B             our LEFT / RIGHT CELL is up (override)
 *   Y                 back to automatic (Limelight if it sees the HIVE, else RIGHT,
 *                     which is how the manual sets up every match)
 * The rules (G401) forbid touching the controls once the countdown begins.
 */
public abstract class AutoOpMode extends OpMode {
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

        ctx = new AutoContext(robot, alliance, defaultSide());
        routine = buildRoutine(ctx);
        robot.setPose(routine.startPose(ctx));
    }

    @Override
    public void init_loop() {
        if (moreCycles.rose(gamepad1.dpad_up)) {
            ctx.cycles = Math.min(ctx.cycles + 1, AutoConstants.MAX_CYCLES);
        }
        if (fewerCycles.rose(gamepad1.dpad_down)) {
            ctx.cycles = Math.max(ctx.cycles - 1, 0);
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
        telemetry.addData("Cycles after pre-loads", "%d   (d-pad up / down)", ctx.cycles);
        telemetry.addData("Up CELL", "%s  [%s]", startSide(), sideSource());
        telemetry.addLine("X = LEFT CELL up, B = RIGHT CELL up, Y = automatic");
        telemetry.addLine("Hands off the controller once the countdown starts (G401)");
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
        robot.butterfly.setDown();
        robot.drivetrain.update();
        trackUpSide();

        if (!routineStarted) {
            ctx.markStart(robot.nowSeconds());
            routine.start(ctx);
            routineStarted = true;
        }

        robot.shootSequence.update(robot.nowSeconds());
        routine.update(ctx);

        // Background jobs. Shooter before intake, so feed gating uses this loop's speed.
        robot.flowerArm.setDown(ctx.flowerDown);
        robot.runShooter(ctx.spinUp || robot.shootSequence.wantsShooter());
        aimTurret();
        robot.runIntake(robot.shootSequence.isFeeding() ? Intake.Mode.INTAKE : ctx.intakeMode);

        telemetry.addData("Auto step", routine.currentStep());
        telemetry.addData("Up CELL", "%s%s", ctx.upSide, robot.isAiming() ? " (Limelight)" : " (odometry)");
        telemetry.addData("Time left s", "%.1f", ctx.timeLeft());
        robot.endLoop();
    }

    @Override
    public void stop() {
        if (routine != null && routineStarted) {
            routine.stop(ctx);
        }
        if (robot != null) {
            robot.stop();
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
        CellSide seen = seenUpSide(System.nanoTime());
        if (seen != null) {
            ctx.setUpSide(seen);
        }
    }

    // With a Limelight target the turret follows it. Without one, point it at the CELL we
    // believe is up, from odometry and its known position, so a shot never waits on the
    // camera and never goes to the CELL facing down.
    private void aimTurret() {
        if (robot.isAiming()) {
            robot.runTurret(0);
            return;
        }
        Pose me = robot.drivetrain.getPose();
        Pose cell = ctx.cellPose(ctx.upSide);
        double bearing = Math.atan2(cell.y() - me.y(), cell.x() - me.x()) - me.heading();
        bearing = Math.atan2(Math.sin(bearing), Math.cos(bearing));
        robot.turret.pointAt(Math.toDegrees(bearing), robot.getLoopSeconds());
    }

    private CellSide seenUpSide(long nowNs) {
        Vision.HiveTarget target = robot.vision.getTarget(nowNs);
        if (target == null || !target.cellUp || target.cell.alliance != alliance) {
            return null;
        }
        return CellSide.of(alliance, target.cell.side);
    }
}
