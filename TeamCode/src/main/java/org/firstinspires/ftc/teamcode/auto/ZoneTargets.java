package org.firstinspires.ftc.teamcode.auto;

import org.firstinspires.ftc.teamcode.constants.AutoConstants;
import org.firstinspires.ftc.teamcode.constants.ShootGateConstants;
import org.firstinspires.ftc.teamcode.constants.VisionConstants;
import org.firstinspires.ftc.teamcode.utils.CellSide;
import org.firstinspires.ftc.teamcode.utils.HiveSide;
import org.firstinspires.ftc.teamcode.utils.ShotTable;
import org.firstinspires.ftc.teamcode.utils.ShotZoneGeometry;

/**
 * Where the auto shoots from: not a fixed spot but the nearest point of the shooting zone.
 * The zone is the TeleOp one (ShotZoneGeometry: distance inside the shot table's calibrated
 * range, shot within ZONE_MAX_OFF_AXIS_DEG of the CELL's opening axis). A point is used only
 * if it is ZONE_INSET_IN inside the zone's edge, on our side of the middle line (G402), clear
 * of the walls enough to turn (WALL_TURN_CLEAR_IN), clear of the FLOWERS, and, for the
 * partner's CELL in a pair, clear of the partner's start (where it shoots from). The heading
 * there faces the CELL, so the fixed camera sees the HIVE while the robot waits and shoots.
 *
 * Red frame (our RIGHT CELL is the AUDIENCE one); pass results through ctx.pose().
 */
public final class ZoneTargets {
    private ZoneTargets() {
    }

    // FLOWER positions in the red frame (doc/auto-design-spec.md section 2).
    private static final double[][] FLOWERS = {{0, 48}, {48, 144}, {95.5, 0}, {144, 95.5}};

    /** The aim point of this CELL, red frame. */
    public static double[] cellXY(CellSide cell) {
        return cell == CellSide.RIGHT
                ? new double[] {AutoConstants.RIGHT_CELL_X, AutoConstants.RIGHT_CELL_Y}
                : new double[] {AutoConstants.LEFT_CELL_X, AutoConstants.LEFT_CELL_Y};
    }

    private static HiveSide hiveSide(CellSide cell) {
        return cell == CellSide.RIGHT ? HiveSide.AUDIENCE : HiveSide.FAR;
    }

    public static double distanceIn(double x, double y, CellSide cell) {
        double[] c = cellXY(cell);
        return ShotZoneGeometry.distanceIn(x, y, c[0], c[1]);
    }

    public static double offAxisDeg(double x, double y, CellSide cell) {
        double[] c = cellXY(cell);
        return ShotZoneGeometry.offAxisDeg(x, y, c[0], c[1], hiveSide(cell));
    }

    /** The zone test itself (no hysteresis), same as TeleOp's. */
    public static boolean inZone(double x, double y, CellSide cell) {
        return ShotZoneGeometry.inZone(distanceIn(x, y, cell), offAxisDeg(x, y, cell), false);
    }

    /** Heading (degrees, red frame) that points the camera at this CELL from (x, y). */
    public static double headingToCellDeg(double x, double y, CellSide cell) {
        double[] c = cellXY(cell);
        return Math.toDegrees(Math.atan2(c[1] - y, c[0] - x)) - VisionConstants.CAMERA_YAW_DEG;
    }

    /**
     * The usable zone point for this CELL nearest to (fromX, fromY), as {x, y, headingDeg},
     * or null if there is none (e.g. the shot table is empty).
     */
    public static double[] target(CellSide cell, double fromX, double fromY, boolean pair,
                                  CellSide ownSide) {
        // In a pair the partner shoots its own CELL (the one nearest its start) from near its
        // start; keep away from there when we shoot that CELL.
        double[] partner = null;
        if (pair && cell != ownSide) {
            partner = AutoSelection.preset == StartPreset.A
                    ? new double[] {AutoConstants.START_B_X, AutoConstants.START_B_Y}
                    : new double[] {AutoConstants.START_A_X, AutoConstants.START_A_Y};
        }

        double step = Math.max(0.5, AutoConstants.ZONE_GRID_IN);
        double maxX = AutoConstants.OWN_SIDE_MAX_X_IN - AutoConstants.OWN_SIDE_MARGIN_IN;
        double clear = AutoConstants.WALL_TURN_CLEAR_IN;
        double[] best = null;
        double bestD = Double.MAX_VALUE;
        for (double x = clear; x <= maxX; x += step) {
            for (double y = clear; y <= AutoConstants.FIELD_SIZE_IN - clear; y += step) {
                if (!usable(x, y, cell, partner)) {
                    continue;
                }
                double d = Math.hypot(x - fromX, y - fromY);
                if (d < bestD) {
                    bestD = d;
                    best = new double[] {x, y, headingToCellDeg(x, y, cell)};
                }
            }
        }
        return best;
    }

    private static boolean usable(double x, double y, CellSide cell, double[] partner) {
        for (double[] f : FLOWERS) {
            if (Math.hypot(x - f[0], y - f[1]) < AutoConstants.SPOT_KEEP_OUT_IN) {
                return false;
            }
        }
        if (partner != null
                && Math.hypot(x - partner[0], y - partner[1]) < AutoConstants.PARTNER_CLEAR_IN) {
            return false;
        }
        // ZONE_INSET_IN inside the edge: inside the distance ring by that much, and inside
        // the angle limit by the angle that distance makes at this range.
        double d = distanceIn(x, y, cell);
        double inset = AutoConstants.ZONE_INSET_IN;
        double insetDeg = d > inset ? Math.toDegrees(inset / d) : 90;
        return ShotTable.inCalibratedRange(d, -inset)
                && offAxisDeg(x, y, cell) <= ShootGateConstants.ZONE_MAX_OFF_AXIS_DEG - insetDeg;
    }
}
