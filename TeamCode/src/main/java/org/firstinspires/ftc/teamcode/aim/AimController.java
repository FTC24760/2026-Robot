package org.firstinspires.ftc.teamcode.aim;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.AutoConstants;
import org.firstinspires.ftc.teamcode.constants.Configurables;
import org.firstinspires.ftc.teamcode.constants.OperatorConstants;
import org.firstinspires.ftc.teamcode.constants.ShootGateConstants;
import org.firstinspires.ftc.teamcode.constants.TurretConstants;
import org.firstinspires.ftc.teamcode.constants.VisionConstants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.utils.Alliance;
import org.firstinspires.ftc.teamcode.utils.CellSide;
import org.firstinspires.ftc.teamcode.utils.HiveCell;
import org.firstinspires.ftc.teamcode.utils.HiveSide;
import org.firstinspires.ftc.teamcode.utils.HiveTracker;
import org.firstinspires.ftc.teamcode.utils.MathUtil;
import org.firstinspires.ftc.teamcode.utils.ShotTable;

/**
 * Pose + camera -> where the up CELL is -> turret bearing, wheel speed and hood, and the
 * aim half of the fire gate. Shared by TeleOp and Auto; owns no hardware of its own (it
 * drives the turret, hood and shooter it is given).
 *
 * Without a Limelight, auto-aim stays off: the turret moves only by hand and the shot is
 * the fixed Configurables values (an auto may still call aimShotAt()).
 */
public class AimController {
    private final Drivetrain drivetrain;
    private final Shooter shooter;
    private final Hood hood;
    private final Turret turret;

    // Null until enable() finds a Limelight.
    private Vision vision;
    // Where the upward-facing CELL of our HIVE is on the field, from the camera and
    // odometry; null without auto-aim.
    private HiveTracker hiveTracker;
    private Alliance alliance;
    // True while the turret should follow hiveTracker.
    private boolean aiming;
    // The driver can switch auto-aim off mid-match if the camera misbehaves.
    private boolean aimEnabled = true;
    private String aimNote = "";
    // The shot (RPM, hood) from the shot table for the current target distance, or null
    // when there is no target distance (then the fixed Configurables values are used).
    private ShotTable.Shot shot;
    // Operator's RPM trim (rpm up/down buttons), added to whatever speed is used.
    private double rpmTrim;

    public AimController(Drivetrain drivetrain, Shooter shooter, Hood hood, Turret turret) {
        this.drivetrain = drivetrain;
        this.shooter = shooter;
        this.hood = hood;
        this.turret = turret;
    }

    /** Look for the Limelight. Returns the Vision to register as a subsystem, or null. */
    public Vision enable(HardwareMap hardwareMap, Alliance alliance) {
        try {
            vision = new Vision(hardwareMap, alliance);
        } catch (RuntimeException e) {
            vision = null;
            aimNote = "AUTO-AIM OFF: no Limelight \"" + VisionConstants.LIMELIGHT_NAME + "\" configured";
            return null;
        }
        hiveTracker = new HiveTracker();
        this.alliance = alliance;
        return vision;
    }

    public boolean hasAutoAim() {
        return vision != null;
    }

    public Vision getVision() {
        return vision;
    }

    /** True if the up/down state is current (see ShootGateConstants.UPSIDE_CHECK). */
    public boolean upStateConfirmed(long nowNs) {
        return hiveTracker == null || !ShootGateConstants.UPSIDE_CHECK
                || hiveTracker.upStateConfirmed(nowNs);
    }

    /** Seconds since the camera or driver last said which CELL is up; NaN if never. */
    public double upStateAgeS(long nowNs) {
        return hiveTracker == null ? Double.NaN : hiveTracker.upStateAgeS(nowNs);
    }

    /** A shot that fed balls ended: the HIVE may have tipped, so wait for a new read. */
    public void noteShotFed(long nowNs) {
        if (hiveTracker != null) {
            hiveTracker.noteShot(nowNs);
        }
    }

    /**
     * Driver override while the camera cannot say: aim at the other CELL (or the right one,
     * which starts up, if none is tracked). Counts as a read until the camera disagrees.
     */
    public void flipUpCell(long nowNs) {
        if (hiveTracker == null) {
            return;
        }
        HiveCell now = hiveTracker.getCell();
        HiveCell next = now != null ? otherCell(now)
                : cellAt(CellSide.RIGHT);
        assumeUp(next, nowNs);
    }

    // Our CELL on the other end of the HIVE.
    private HiveCell otherCell(HiveCell cell) {
        return cellOf(cell.alliance, cell.side.other());
    }

    private HiveCell cellAt(CellSide side) {
        for (HiveCell c : HiveCell.values()) {
            if (c.alliance == alliance && CellSide.of(alliance, c.side) == side) {
                return c;
            }
        }
        return null;
    }

    private static HiveCell cellOf(Alliance alliance, HiveSide side) {
        for (HiveCell c : HiveCell.values()) {
            if (c.alliance == alliance && c.side == side) {
                return c;
            }
        }
        return null;
    }

    // Aim at this CELL from the field points in AutoConstants (red frame, flipped for blue)
    // and take it as read now.
    private void assumeUp(HiveCell cell, long nowNs) {
        if (cell == null || cell.alliance != alliance) {
            return;
        }
        boolean left = CellSide.of(alliance, cell.side) == CellSide.LEFT;
        double x = left ? AutoConstants.LEFT_CELL_X : AutoConstants.RIGHT_CELL_X;
        double y = left ? AutoConstants.LEFT_CELL_Y : AutoConstants.RIGHT_CELL_Y;
        if (alliance == Alliance.BLUE) {
            x = AutoConstants.FIELD_SIZE_IN - x;
            y = AutoConstants.FIELD_SIZE_IN - y;
        }
        hiveTracker.assume(cell, x, y, nowNs);
        hiveTracker.noteRead(nowNs);
    }

    /** The robot's pose was reset: move the remembered CELL position with it. */
    public void onPoseReset(Pose old, Pose now) {
        if (hiveTracker != null) {
            hiveTracker.onPoseReset(old, now);
        }
    }

    /** True while auto-aim has an up-facing CELL to aim at (seen, or remembered). */
    public boolean isAiming() {
        return aiming;
    }

    public void toggleEnabled() {
        aimEnabled = !aimEnabled;
    }

    /** Operator RPM buttons: +1 / -1 steps the trim, taking effect next loop. */
    public void adjustRpmTrim(int direction) {
        rpmTrim = MathUtil.clamp(rpmTrim + direction * OperatorConstants.RPM_STEP,
                -OperatorConstants.MAX_RPM_TRIM, OperatorConstants.MAX_RPM_TRIM);
    }

    /** The table shot in use right now, or null if there is no target distance. */
    public ShotTable.Shot getShot() {
        return shot;
    }

    /** Auto-aim for this loop: camera -> remembered CELL -> speed and hood. */
    public void update(long nowNs) {
        if (vision == null) {
            // No Limelight: fixed speed and hood (an auto may still call aimShotAt()).
            aimShotAt(Double.NaN);
            return;
        }

        hiveTracker.recordPose(drivetrain.getPoseTimeNs(), drivetrain.getPose());
        vision.update(nowNs);

        // Only aim at a CELL that is facing up; the other one cannot be scored in.
        Vision.Fix fix = vision.getFix();
        if (fix != null) {
            hiveTracker.noteRead(fix.captureNs);
            if (fix.cellUp) {
                hiveTracker.addSighting(fix, turret.getAngleDeg());
            } else {
                // The two CELLS are joined: this one down means the other is up. Keep aiming
                // at it from its known position until the camera sees it.
                hiveTracker.sawCellDown(fix.sighting.cell);
                assumeUp(otherCell(fix.sighting.cell), nowNs);
            }
        }

        aiming = aimEnabled && hiveTracker.hasTarget(nowNs);
        // Speed and hood from the shot table for the target's distance; without a target,
        // the fixed Configurables values.
        aimShotAt(targetDistanceIn());
    }

    /**
     * Aim the turret for this loop. With auto-aim on and a target (seen now, or remembered
     * and followed with odometry), the turret points at it from the latest pose; otherwise,
     * or while the driver is steering it by hand, it moves only as directed.
     */
    public void runTurret(double manualDirection, double loopSeconds) {
        if (manualDirection != 0 || !aiming) {
            turret.move(manualDirection, loopSeconds);
            return;
        }

        double bearing = hiveTracker.turretBearingDeg(
                drivetrain.getPose(), drivetrain.getVelocity(), TurretConstants.AIM_LEAD_SECONDS,
                flightS(), ShootGateConstants.MAX_LEAD_IN);
        turret.pointAt(bearing, loopSeconds);
    }

    /**
     * Set wheel speed and hood for a shot from this distance (inches, robot centre to the
     * CELL), from the shot table; NaN or no table falls back to the fixed Configurables
     * values. Auto-aim calls this every loop; the auto also calls it with an odometry
     * distance when the camera has no target.
     */
    public void aimShotAt(double distanceIn) {
        shot = ShotTable.lookup(distanceIn);
        if (shot != null) {
            shooter.setShootRpm(shot.rpm + rpmTrim);
            hood.setPosition(shot.hoodPosition);
        } else {
            shooter.setShootRpm(Configurables.SHOOTER_RPM + rpmTrim);
            hood.setPosition(Configurables.HOOD_POSITION);
        }
    }

    /**
     * Distance from the robot centre to the up CELL auto-aim is tracking, inches, or NaN
     * without a target.
     */
    public double targetDistanceIn() {
        return aiming ? hiveTracker.distanceM(drivetrain.getPose(), drivetrain.getVelocity(),
                flightS(), ShootGateConstants.MAX_LEAD_IN) / 0.0254 : Double.NaN;
    }

    /** Degrees the shot to the up CELL is off the CELL's opening axis, NaN without a target. */
    public double targetOffAxisDeg() {
        return aiming ? hiveTracker.offAxisDeg(drivetrain.getPose()) : Double.NaN;
    }

    // Ball flight time for the virtual target; 0 (aim at the CELL itself) unless shoot on
    // the move is on.
    private double flightS() {
        return ShootGateConstants.SHOOT_ON_MOVE ? ShootGateConstants.FLIGHT_TIME_S : 0;
    }

    /**
     * The aim half of the fire gate (the wheel-speed half is in Shooter): true when a ball
     * may be fed now. See aimBlockReason().
     */
    public boolean aimReady() {
        return aimBlockReason() == null;
    }

    /** Why aimReady() is false, for telemetry; null when ready. */
    public String aimBlockReason() {
        if (!ShootGateConstants.ENABLED) {
            return null;
        }
        if (turret.isAutoAiming() && !turret.isSettledOnTarget(ShootGateConstants.AIM_SETTLE_S)) {
            return "turret settling";
        }
        if (!hood.isSettled(ShootGateConstants.HOOD_SETTLE_S)) {
            return "hood settling";
        }
        Velocity v = drivetrain.getVelocity();
        if (v != null) {
            boolean onMove = ShootGateConstants.SHOOT_ON_MOVE;
            double maxSpeed = onMove ? ShootGateConstants.MOVING_MAX_SPEED_IN_PER_S
                    : ShootGateConstants.MAX_SPEED_IN_PER_S;
            double maxTurn = onMove ? ShootGateConstants.MOVING_MAX_TURN_DEG_PER_S
                    : ShootGateConstants.MAX_TURN_DEG_PER_S;
            if (Math.hypot(v.vx, v.vy) > maxSpeed) {
                return "moving too fast";
            }
            if (Math.abs(Math.toDegrees(v.omega)) > maxTurn) {
                return "turning too fast";
            }
        }
        return null;
    }

    /** The auto-aim lines of the telemetry screen. */
    public void addTelemetry(Telemetry telemetry, long nowNs) {
        if (!aimNote.isEmpty()) {
            telemetry.addLine(aimNote);
        }
        if (vision == null) {
            return;
        }
        if (!aimEnabled) {
            telemetry.addData("Auto-aim", "SWITCHED OFF");
        } else if (shot == null) {
            telemetry.addData("Auto-aim", "no target distance, fixed %.0f RPM",
                    Configurables.SHOOTER_RPM + rpmTrim);
        } else {
            telemetry.addData("Auto-aim", "%.0f in: %.0f RPM, hood %.3f%s",
                    shot.distanceIn, shot.rpm + rpmTrim, shot.hoodPosition,
                    shot.inRange ? "" : "  (OUTSIDE MEASURED TABLE)");
        }
        if (rpmTrim != 0) {
            telemetry.addData("RPM trim", "%+.0f", rpmTrim);
        }
        double readAge = upStateAgeS(nowNs);
        telemetry.addData("Up/down read age", Double.isNaN(readAge) ? "never"
                : String.format("%.1f s%s", readAge, upStateConfirmed(nowNs) ? "" : " (stale)"));
        if (aiming) {
            telemetry.addData("Aim source", hiveTracker.seesTarget(nowNs)
                    ? "camera" : "odometry (camera lost it)");
        }
    }
}
