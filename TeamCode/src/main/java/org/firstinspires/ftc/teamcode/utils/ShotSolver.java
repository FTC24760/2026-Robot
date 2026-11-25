package org.firstinspires.ftc.teamcode.utils;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.constants.AutoConstants;
import org.firstinspires.ftc.teamcode.constants.HoodConstants;
import org.firstinspires.ftc.teamcode.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.constants.TurretConstants;

/**
 * Works out how to hit the HIVE from the odometry pose: which CELL, turret bearing, wheel
 * speed and hood. Robot calls update() each loop; the aim commands read the public fields.
 *
 *   CELL     the nearer of our alliance's two CELLS (AutoConstants.RIGHT_CELL / LEFT_CELL,
 *            mirrored for blue). Vision is off in TeleOp, so it can't tell which is up.
 *   turret   bearing from the turret pivot to that CELL, from the pose pushed ahead by
 *            TurretConstants.AIM_LEAD_SECONDS of the current velocity
 *   RPM      linear fit from Desmos: RPM_SLOPE * distance + RPM_INTERCEPT
 *   hood     linear fit from Desmos: HOOD_INTERCEPT + HOOD_DISTANCE_COEF * distance
 *            + HOOD_RPM_COEF * RPM
 *
 * The data for both fits comes from the "Test: Shot Tuning" OpMode. Distance is inches
 * from the turret pivot to the CELL centre, on the floor.
 *
 * The pose must be in the field frame described in AutoConstants. Robot sets it at the
 * start (from auto, or the start pose) and the operator's reset-pose buttons fix drift.
 */
public class ShotSolver {
    private static final double IN_PER_M = 1 / 0.0254;

    // Outputs, set by update().
    // Hood servo position.
    public double hoodAngle;
    public double shooterTargetRPM;
    // Turret bearing to aim at, radians to the left of robot forward.
    public double turretAngle;
    // Inches from the turret pivot to the CELL being aimed at.
    public double distanceIn;
    public HiveCell cell;

    // Tuning only: when not NaN, used instead of the fits. Set by the Shot Tuning OpMode.
    public double rpmOverride = Double.NaN;
    public double hoodOverride = Double.NaN;

    Follower follower;
    // Read the alliance as robot.alliance each loop rather than copying it here.
    Robot robot;

    public ShotSolver(Robot robot) {
        this.robot = robot;
        follower = robot.follower;
    }

    /** Robot turn rate from odometry, rad/s counter-clockwise; used as turret feedforward. */
    public double getAngularVelocity() {
        return follower.velocity().omega;
    }

    public void update() {
        Pose pose = follower.pose();
        Velocity velocity = follower.velocity();

        double lead = TurretConstants.AIM_LEAD_SECONDS;
        double x = pose.x() + velocity.vx * lead;
        double y = pose.y() + velocity.vy * lead;
        double h = pose.heading() + velocity.omega * lead;

        // Turret pivot on the field.
        double pf = TurretConstants.PIVOT_FORWARD_M * IN_PER_M;
        double pl = TurretConstants.PIVOT_LEFT_M * IN_PER_M;
        double px = x + pf * Math.cos(h) - pl * Math.sin(h);
        double py = y + pf * Math.sin(h) + pl * Math.cos(h);

        Pose right = FieldPoses.forAlliance(robot.alliance,
                AutoConstants.RIGHT_CELL_X, AutoConstants.RIGHT_CELL_Y, 0);
        Pose left = FieldPoses.forAlliance(robot.alliance,
                AutoConstants.LEFT_CELL_X, AutoConstants.LEFT_CELL_Y, 0);
        double rightDistance = Math.hypot(right.x() - px, right.y() - py);
        double leftDistance = Math.hypot(left.x() - px, left.y() - py);
        boolean aimRight = rightDistance <= leftDistance;
        Pose target = aimRight ? right : left;
        cell = FieldPoses.cell(robot.alliance, aimRight ? CellSide.RIGHT : CellSide.LEFT);
        distanceIn = aimRight ? rightDistance : leftDistance;

        turretAngle = normalize(Math.atan2(target.y() - py, target.x() - px) - h);

        shooterTargetRPM = Double.isNaN(rpmOverride)
                ? MathUtil.clamp(
                        ShooterConstants.RPM_SLOPE * distanceIn + ShooterConstants.RPM_INTERCEPT,
                        0, ShooterConstants.MAX_RPM)
                : rpmOverride;

        // Hood.setPosition clamps this to the hood's range.
        hoodAngle = Double.isNaN(hoodOverride)
                ? HoodConstants.HOOD_INTERCEPT
                        + HoodConstants.HOOD_DISTANCE_COEF * distanceIn
                        + HoodConstants.HOOD_RPM_COEF * shooterTargetRPM
                : hoodOverride;
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Target CELL", "%s, %.1f in, bearing %.1f deg",
                cell, distanceIn, Math.toDegrees(turretAngle));
    }

    private static double normalize(double rad) {
        return Math.atan2(Math.sin(rad), Math.cos(rad));
    }
}
