package org.firstinspires.ftc.teamcode.auto;

import org.firstinspires.ftc.teamcode.constants.AutoConstants;
import org.firstinspires.ftc.teamcode.constants.AutoConstants.CycleSource;

/**
 * Rough trip times worked out from the poses in AutoConstants, to decide which cycle is
 * cheapest and whether it still fits in the time left. Red frame; every number it uses is a
 * placeholder until measured (AutoConstants AVG_SPEED_IN_PER_S ...).
 */
public final class TripTimes {
    private TripTimes() {
    }

    /** Seconds to drive from (x0, y0) to (x1, y1). */
    public static double driveS(double x0, double y0, double x1, double y1) {
        double d = Math.hypot(x1 - x0, y1 - y0);
        if (d < AutoConstants.ALREADY_THERE_IN) {
            return 0;
        }
        double mid = AutoConstants.FIELD_SIZE_IN / 2;
        if ((y0 < mid) != (y1 < mid)) {
            d *= AutoConstants.CROSS_DETOUR; // bulges around the HIVE
        }
        return d / AutoConstants.AVG_SPEED_IN_PER_S + AutoConstants.TURN_S;
    }

    /** Where the robot goes to collect from this source, red frame. */
    public static double[] sourceXY(CycleSource source) {
        switch (source) {
            case WALL_FLOWER:
                return new double[] {AutoConstants.WALL_FLOWER_X, AutoConstants.WALL_FLOWER_Y};
            case BACK_FLOWER:
                return new double[] {AutoConstants.BACK_FLOWER_X, AutoConstants.BACK_FLOWER_Y};
            case GARDEN:
            default:
                return new double[] {AutoConstants.GARDEN_X, AutoConstants.GARDEN_Y};
        }
    }

    /**
     * Drive from {@code from}, collect, drive to {@code next}: the cycle without its burst.
     * The GARDEN is collected while driving; a FLOWER needs the arm and the shake.
     */
    public static double cycleS(CycleSource source, double[] from, double[] next) {
        double[] at = sourceXY(source);
        if (source == CycleSource.GARDEN) {
            // Line up against the wall first, then along it.
            double lx = AutoConstants.GARDEN_LEAD_IN_X;
            double ly = AutoConstants.GARDEN_Y;
            return driveS(from[0], from[1], lx, ly) + driveS(lx, ly, at[0], at[1])
                    + driveS(at[0], at[1], next[0], next[1]);
        }
        double collect = AutoConstants.FLOWER_ARM_S + AutoConstants.FLOWER_COLLECT_S;
        return driveS(from[0], from[1], at[0], at[1]) + collect
                + driveS(at[0], at[1], next[0], next[1]);
    }
}
