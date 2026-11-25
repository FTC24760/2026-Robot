package org.firstinspires.ftc.teamcode.utils;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.constants.AutoConstants;

/**
 * Turns the red-side poses in the constants into poses for either alliance. The field is
 * the same for blue turned half a circle about its centre (see AutoConstants for the
 * frame: inches, origin at the red/audience corner, heading 0 toward blue, CCW positive).
 */
public final class FieldPoses {
    private FieldPoses() {
    }

    /**
     * @param x        red-side x, inches
     * @param y        red-side y, inches
     * @param headingDeg red-side heading, degrees
     * @return the pose for this alliance, heading in radians
     */
    public static Pose forAlliance(Alliance alliance, double x, double y, double headingDeg) {
        double h = Math.toRadians(headingDeg);
        if (alliance == Alliance.BLUE) {
            double size = AutoConstants.FIELD_SIZE_IN;
            double blueH = h + Math.PI;
            return new Pose(size - x, size - y, Math.atan2(Math.sin(blueH), Math.cos(blueH)));
        }
        return new Pose(x, y, h);
    }

    /** The CELL of this alliance's HIVE that its drivers call left or right. */
    public static HiveCell cell(Alliance alliance, CellSide side) {
        // Red drivers have the audience on their right, blue drivers on their left.
        boolean audience = (side == CellSide.RIGHT) == (alliance == Alliance.RED);
        HiveSide fieldSide = audience ? HiveSide.AUDIENCE : HiveSide.FAR;
        for (HiveCell cell : HiveCell.values()) {
            if (cell.alliance == alliance && cell.side == fieldSide) {
                return cell;
            }
        }
        throw new IllegalStateException("no CELL for " + alliance + " " + side);
    }

    /**
     * Driver's forward relative to field +X: 0 for red (drivers look toward blue), half a
     * turn for blue. Used to make field-centric drive relative to our drivers.
     */
    public static double driverHeadingOffset(Alliance alliance) {
        return alliance == Alliance.BLUE ? Math.PI : 0;
    }
}
