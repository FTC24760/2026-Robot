package org.firstinspires.ftc.teamcode.utils;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;

import org.firstinspires.ftc.teamcode.constants.TurretConstants;
import org.firstinspires.ftc.teamcode.constants.VisionConstants;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

/**
 * Remembers where the HIVE CELL is on the field, so the turret can aim from odometry.
 *
 * Each camera sighting is placed on the field using the odometry pose from the moment
 * the frame was captured (not when it arrived), so camera lag and the robot turning in
 * the meantime do not throw the aim off. The turret then aims from the current pose
 * every loop. That keeps it on target while the robot spins, and keeps it aimed when the
 * camera loses the tags.
 *
 * Field frame is the Pedro odometry frame: inches, heading in radians, CCW positive.
 */
public class HiveTracker {
    private static final double M_PER_IN = 0.0254;
    private static final int HISTORY = 64;

    // Pose history, oldest overwritten first.
    private final long[] historyNs = new long[HISTORY];
    private final Pose[] historyPose = new Pose[HISTORY];
    private int historyCount;
    private int historyNext;

    private boolean has;
    private HiveCell cell;
    private double fieldX, fieldY; // inches
    private double tagHeightM;
    private long lastVisionNs;

    /** Record the odometry pose and when it was read. Call once per loop. */
    public void recordPose(long ns, Pose pose) {
        if (historyCount > 0 && historyNs[(historyNext + HISTORY - 1) % HISTORY] == ns) {
            return; // same reading as last time
        }
        historyNs[historyNext] = ns;
        historyPose[historyNext] = pose;
        historyNext = (historyNext + 1) % HISTORY;
        historyCount = Math.min(historyCount + 1, HISTORY);
    }

    /**
     * Add one camera sighting of an upward-facing CELL.
     *
     * @param turretAngleDeg where the turret pointed when the frame was captured; only
     *                       used when the camera rides on the turret
     */
    public void addSighting(Vision.Fix fix, double turretAngleDeg) {
        Pose pose = poseAt(fix.captureNs);
        if (pose == null) {
            return;
        }

        // Robot-frame position of the tags, inches.
        double fwd = fix.sighting.forwardM / M_PER_IN;
        double left = fix.sighting.leftM / M_PER_IN;
        if (VisionConstants.CAMERA_ON_TURRET) {
            // The camera turned with the turret, so its "forward" was the turret's.
            double t = Math.toRadians(turretAngleDeg);
            double f2 = fwd * Math.cos(t) - left * Math.sin(t);
            left = fwd * Math.sin(t) + left * Math.cos(t);
            fwd = f2;
        }

        double h = pose.heading();
        double x = pose.x() + fwd * Math.cos(h) - left * Math.sin(h);
        double y = pose.y() + fwd * Math.sin(h) + left * Math.cos(h);

        if (has && cell == fix.sighting.cell) {
            // The HIVE does not move, so smoothing here adds no lag when the robot turns.
            double a = MathUtil.clamp(VisionConstants.FIELD_SMOOTHING, 0, 0.95);
            fieldX = a * fieldX + (1 - a) * x;
            fieldY = a * fieldY + (1 - a) * y;
            tagHeightM = a * tagHeightM + (1 - a) * fix.sighting.heightM;
        } else {
            fieldX = x;
            fieldY = y;
            tagHeightM = fix.sighting.heightM;
        }

        has = true;
        cell = fix.sighting.cell;
        lastVisionNs = fix.captureNs;
    }

    /** The camera saw this CELL facing down: the remembered target is no longer valid. */
    public void sawCellDown(HiveCell downCell) {
        if (has && cell == downCell) {
            forget();
        }
    }

    public void forget() {
        has = false;
        cell = null;
    }

    /**
     * The odometry origin moved (pose reset): re-express the remembered CELL in the new
     * coordinates so it stays where it really is.
     *
     * @param oldPose the robot's pose just before the reset, in the old frame
     * @param newPose the same physical pose, in the new frame
     */
    public void onPoseReset(Pose oldPose, Pose newPose) {
        historyCount = 0;
        historyNext = 0;
        if (!has) {
            return;
        }

        // Target relative to the robot, then back out in the new frame.
        double dx = fieldX - oldPose.x();
        double dy = fieldY - oldPose.y();
        double ho = oldPose.heading();
        double fwd = dx * Math.cos(ho) + dy * Math.sin(ho);
        double left = -dx * Math.sin(ho) + dy * Math.cos(ho);

        double hn = newPose.heading();
        fieldX = newPose.x() + fwd * Math.cos(hn) - left * Math.sin(hn);
        fieldY = newPose.y() + fwd * Math.sin(hn) + left * Math.cos(hn);
    }

    /** True if there is a target to aim at, from the camera or from memory. */
    public boolean hasTarget(long nowNs) {
        if (!has) {
            return false;
        }
        if (seesTarget(nowNs)) {
            return true;
        }
        if (!VisionConstants.ODOMETRY_FALLBACK) {
            return false;
        }
        double maxS = VisionConstants.ODOMETRY_FALLBACK_MAX_S;
        return maxS <= 0 || (nowNs - lastVisionNs) * 1e-9 <= maxS;
    }

    /** True if the camera has seen the target recently (not just remembered). */
    public boolean seesTarget(long nowNs) {
        return has && nowNs - lastVisionNs <= MathUtil.msToNs(VisionConstants.TARGET_TIMEOUT_MS);
    }

    public HiveCell getCell() {
        return cell;
    }

    public double getTagHeightM() {
        return tagHeightM;
    }

    /**
     * How far the shot from {@code pose} to the CELL is off the CELL's opening axis, in
     * degrees, 0 (straight into the opening) to 180. The up CELL opens along the field's
     * y axis, away from the HIVE's pivot: the AUDIENCE cell toward the audience wall (-y),
     * the FAR cell toward the far wall (+y) (manual figs 9-9, 9-10; field frame has the
     * audience wall at the bottom). A shot perpendicular to that (90) grazes the opening.
     * NaN with no target.
     */
    public double offAxisDeg(Pose pose) {
        if (!has || cell == null) {
            return Double.NaN;
        }
        double openY = cell.side == HiveSide.AUDIENCE ? -1 : 1;
        double rx = pose.x() - fieldX; // CELL -> robot
        double ry = pose.y() - fieldY;
        double len = Math.hypot(rx, ry);
        if (len < 1e-6) {
            return 0;
        }
        return Math.toDegrees(Math.acos(Math.max(-1, Math.min(1, ry * openY / len))));
    }

    /** Horizontal distance from the robot centre to the CELL, metres. */
    public double distanceM(Pose pose) {
        return Math.hypot(fieldX - pose.x(), fieldY - pose.y()) * M_PER_IN;
    }

    /**
     * Direction from the turret pivot to the CELL, degrees left of robot forward, for a
     * pose pushed ahead by {@code leadS} seconds of the current velocity. The lead makes
     * up for the servo's own lag when the robot is moving or turning.
     */
    public double turretBearingDeg(Pose pose, Velocity velocity, double leadS) {
        double x = pose.x();
        double y = pose.y();
        double h = pose.heading();
        if (velocity != null && leadS > 0) {
            x += velocity.vx * leadS;
            y += velocity.vy * leadS;
            h += velocity.omega * leadS;
        }

        // Turret pivot on the field.
        double pf = TurretConstants.PIVOT_FORWARD_M / M_PER_IN;
        double pl = TurretConstants.PIVOT_LEFT_M / M_PER_IN;
        double px = x + pf * Math.cos(h) - pl * Math.sin(h);
        double py = y + pf * Math.sin(h) + pl * Math.cos(h);

        double fieldAngle = Math.atan2(fieldY - py, fieldX - px);
        return Math.toDegrees(normalize(fieldAngle - h));
    }

    // Odometry pose at a past moment, interpolated; the nearest end if out of range.
    private Pose poseAt(long ns) {
        if (historyCount == 0) {
            return null;
        }

        int newest = (historyNext + HISTORY - 1) % HISTORY;
        int oldest = (historyNext + HISTORY - historyCount) % HISTORY;
        if (ns >= historyNs[newest]) {
            return historyPose[newest];
        }
        if (ns <= historyNs[oldest]) {
            return historyPose[oldest];
        }

        for (int i = 1; i < historyCount; i++) {
            int a = (newest + HISTORY - i) % HISTORY;
            int b = (a + 1) % HISTORY;
            if (historyNs[a] <= ns) {
                double t = (double) (ns - historyNs[a]) / (historyNs[b] - historyNs[a]);
                Pose pa = historyPose[a];
                Pose pb = historyPose[b];
                double dh = normalize(pb.heading() - pa.heading());
                return new Pose(
                        pa.x() + (pb.x() - pa.x()) * t,
                        pa.y() + (pb.y() - pa.y()) * t,
                        pa.heading() + dh * t);
            }
        }
        return historyPose[oldest];
    }

    private static double normalize(double rad) {
        return Math.atan2(Math.sin(rad), Math.cos(rad));
    }
}
