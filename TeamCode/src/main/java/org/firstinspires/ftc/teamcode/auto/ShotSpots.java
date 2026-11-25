package org.firstinspires.ftc.teamcode.auto;

import org.firstinspires.ftc.teamcode.constants.AutoConstants;
import org.firstinspires.ftc.teamcode.utils.CellSide;
import org.firstinspires.ftc.teamcode.utils.HiveSide;
import org.firstinspires.ftc.teamcode.utils.ShotZoneGeometry;

/**
 * Where Auto shoots from. Candidates are the fixed shot spot for a CELL turned about that
 * CELL in steps (same distance, so the same shot-table row), and a candidate is allowed only
 * if it is in the shooting zone (ShotZoneGeometry, same test as TeleOp), on our side of the
 * field (G402), inside the field, and clear of the FLOWERS. SOLO and FIRST use the fixed
 * spot; SECOND uses the allowed candidate farthest from it, because the partner is expected
 * to be at the fixed spot. If nothing else is allowed (for example the shot table is empty),
 * the fixed spot is used. Everything is in the red frame; pass it through ctx.pose().
 */
public final class ShotSpots {
    private ShotSpots() {
    }

    // FLOWER positions in the red frame (doc/auto-design-spec.md section 2).
    private static final double[][] FLOWERS = {{0, 48}, {48, 144}, {95.5, 0}, {144, 95.5}};

    /** {x, y, headingDeg} to shoot from into this CELL, red frame. */
    public static double[] spot(CellSide cell, AutoSelection.Slot slot) {
        boolean right = cell == CellSide.RIGHT;
        double fx = right ? AutoConstants.SHOOT_X : AutoConstants.SHOOT_LEFT_UP_X;
        double fy = right ? AutoConstants.SHOOT_Y : AutoConstants.SHOOT_LEFT_UP_Y;
        double fh = right ? AutoConstants.SHOOT_H : AutoConstants.SHOOT_LEFT_UP_H;
        double[] fixed = {fx, fy, fh};
        if (slot != AutoSelection.Slot.SECOND || !AutoConstants.SECOND_USES_OFFSET_SPOT) {
            return fixed;
        }

        double cx = right ? AutoConstants.RIGHT_CELL_X : AutoConstants.LEFT_CELL_X;
        double cy = right ? AutoConstants.RIGHT_CELL_Y : AutoConstants.LEFT_CELL_Y;
        HiveSide side = right ? HiveSide.AUDIENCE : HiveSide.FAR;
        double[] best = fixed;
        double bestAway = 0;
        for (int k = -AutoConstants.SPOT_ARC_STEPS; k <= AutoConstants.SPOT_ARC_STEPS; k++) {
            if (k == 0) {
                continue;
            }
            double a = Math.toRadians(k * AutoConstants.SPOT_ARC_STEP_DEG);
            double vx = fx - cx;
            double vy = fy - cy;
            double x = cx + vx * Math.cos(a) - vy * Math.sin(a);
            double y = cy + vx * Math.sin(a) + vy * Math.cos(a);
            if (!allowed(x, y, cx, cy, side)) {
                continue;
            }
            double away = Math.hypot(x - fx, y - fy);
            if (away > bestAway) {
                bestAway = away;
                best = new double[] {x, y, fh};
            }
        }
        return best;
    }

    private static boolean allowed(double x, double y, double cx, double cy, HiveSide side) {
        double m = AutoConstants.OWN_SIDE_MARGIN_IN;
        if (x > AutoConstants.OWN_SIDE_MAX_X_IN - m || x < m
                || y < m || y > AutoConstants.FIELD_SIZE_IN - m) {
            return false;
        }
        for (double[] f : FLOWERS) {
            if (Math.hypot(x - f[0], y - f[1]) < AutoConstants.SPOT_KEEP_OUT_IN) {
                return false;
            }
        }
        return ShotZoneGeometry.inZone(ShotZoneGeometry.distanceIn(x, y, cx, cy),
                ShotZoneGeometry.offAxisDeg(x, y, cx, cy, side), false);
    }
}
