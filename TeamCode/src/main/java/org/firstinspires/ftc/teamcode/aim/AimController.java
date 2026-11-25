package org.firstinspires.ftc.teamcode.aim;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.function.DoubleSupplier;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
    // Distance to use when auto-aim has no target (Auto: odometry to the CELL believed up).
    // Returns NaN for "none"; null = always NaN (TeleOp: fixed Configurables values).
    private DoubleSupplier fallbackDistanceIn;
    // A test OpMode's own RPM / hood, used instead of the table while set (NaN = off).
    private double overrideRpm = Double.NaN;
    private double overrideHood = Double.NaN;

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
        return vision;
    }

    public boolean hasAutoAim() {
        return vision != null;
    }

    public Vision getVision() {
        return vision;
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

    /**
     * Distance (inches) to look the shot up for when auto-aim has no target, e.g. Auto's
     * odometry distance to the CELL it believes is up. The hood and RPM are then set once
     * per loop, here, so nothing else has to call aimShotAt() (two different hood
     * commands in one loop would make the hood never count as settled).
     */
    public void setFallbackDistance(DoubleSupplier distanceIn) {
        fallbackDistanceIn = distanceIn;
    }

    /** Use these instead of the table (a calibration OpMode trying values); NaN clears. */
    public void overrideShot(double rpm, double hoodPosition) {
        overrideRpm = rpm;
        overrideHood = hoodPosition;
    }

    /** The table shot in use right now, or null if there is no target distance. */
    public ShotTable.Shot getShot() {
        return shot;
    }

    /** Auto-aim for this loop: camera -> remembered CELL -> speed and hood. */
    public void update(long nowNs) {
        if (vision == null) {
            // No Limelight: the fallback distance (Auto), else fixed speed and hood.
            aimShotAt(fallbackDistance());
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
        // Speed and hood from the shot table for the target's distance; without a target,
        // the fallback distance (Auto), else the fixed Configurables values.
        aimShotAt(aiming ? targetDistanceIn() : fallbackDistance());
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
        if (!Double.isNaN(overrideRpm)) {
            shooter.setShootRpm(overrideRpm);
            hood.setPosition(Double.isNaN(overrideHood) ? Configurables.HOOD_POSITION : overrideHood);
        } else if (shot != null) {
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

    private double fallbackDistance() {
        return fallbackDistanceIn == null ? Double.NaN : fallbackDistanceIn.getAsDouble();
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
        if (aiming) {
            telemetry.addData("Aim source", hiveTracker.seesTarget(nowNs)
                    ? "camera" : "odometry (camera lost it)");
        }
    }
}
