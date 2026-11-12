package org.firstinspires.ftc.teamcode.utils;

import org.firstinspires.ftc.teamcode.constants.ShotConstants;

/**
 * Works out how to shoot into a CELL from a measured distance.
 *
 * The CELL faces upwards, so the ball must arrive falling. The solver:
 *   1. picks the launch angle that makes the ball arrive at ENTRY_ANGLE_DEG below
 *      horizontal (drag-free geometry), limited to what the hood can reach;
 *   2. finds the exit speed that passes through the target at that angle, including
 *      air drag, by simulating the flight;
 *   3. converts exit speed to wheel RPM and launch angle to a hood servo position.
 */
public final class ShotSolver {
    private static final double G = 9.81;
    private static final double SIM_STEP_S = 0.002;
    private static final double SIM_MAX_S = 3.0;

    private ShotSolver() {
    }

    public static class Solution {
        /** False if the target cannot be reached within the hood and RPM limits. */
        public final boolean reachable;
        public final double launchAngleDeg;
        public final double exitSpeedMps;
        public final double rpm;
        public final double hoodPosition;

        Solution(boolean reachable, double launchAngleDeg, double exitSpeedMps,
                 double rpm, double hoodPosition) {
            this.reachable = reachable;
            this.launchAngleDeg = launchAngleDeg;
            this.exitSpeedMps = exitSpeedMps;
            this.rpm = rpm;
            this.hoodPosition = hoodPosition;
        }
    }

    /**
     * @param distanceM  horizontal distance from the robot centre to the CELL
     * @param tagHeightM height of the CELL's AprilTag cluster above the tiles
     */
    public static Solution solveForCell(double distanceM, double tagHeightM) {
        double range = distanceM - ShotConstants.EXIT_FORWARD_M;
        double rise = tagHeightM + ShotConstants.TARGET_ABOVE_TAG_M - ShotConstants.EXIT_HEIGHT_M;
        return solve(range, rise);
    }

    /**
     * @param rangeM horizontal distance from the shooter exit to the target
     * @param riseM  height of the target above the shooter exit
     */
    public static Solution solve(double rangeM, double riseM) {
        double minAngle = Math.min(ShotConstants.HOOD_DOWN_ANGLE_DEG, ShotConstants.HOOD_UP_ANGLE_DEG);
        double maxAngle = Math.max(ShotConstants.HOOD_DOWN_ANGLE_DEG, ShotConstants.HOOD_UP_ANGLE_DEG);

        if (rangeM <= 0.05) {
            return unreachable(maxAngle);
        }

        // Drag-free parabola through (range, rise): the slope on arrival is
        // 2*rise/range - tan(launch). Setting it to -tan(entry) gives the launch angle.
        double entry = Math.toRadians(ShotConstants.ENTRY_ANGLE_DEG);
        double wanted = Math.toDegrees(Math.atan(2 * riseM / rangeM + Math.tan(entry)));
        double angleDeg = MathUtil.clamp(wanted, minAngle, maxAngle);
        double angle = Math.toRadians(angleDeg);

        // The target must be below the launch line, or no speed gets there.
        double drop = rangeM * Math.tan(angle) - riseM;
        if (drop <= 0) {
            return unreachable(angleDeg);
        }

        // If the hood limit flattened the shot so much that the ball would still be
        // rising at the CELL, it cannot drop in.
        if (2 * riseM / rangeM - Math.tan(angle) >= 0) {
            return unreachable(angleDeg);
        }

        double dragFree = Math.sqrt(G * rangeM * rangeM / (2 * Math.cos(angle) * Math.cos(angle) * drop));
        double speed = solveSpeedWithDrag(rangeM, riseM, angle, dragFree);

        double rpm = speed / ShotConstants.EXIT_SPEED_PER_RPM;
        boolean reachable = !Double.isNaN(speed)
                && rpm >= ShotConstants.MIN_RPM
                && rpm <= ShotConstants.MAX_RPM;

        return new Solution(
                reachable,
                angleDeg,
                speed,
                MathUtil.clamp(rpm, ShotConstants.MIN_RPM, ShotConstants.MAX_RPM),
                hoodPositionFor(angleDeg)
        );
    }

    /** Hood servo position for a launch angle, by straight-line fit between the two calibrated points. */
    public static double hoodPositionFor(double launchAngleDeg) {
        double span = ShotConstants.HOOD_UP_ANGLE_DEG - ShotConstants.HOOD_DOWN_ANGLE_DEG;
        if (Math.abs(span) < 1e-6) {
            return ShotConstants.HOOD_DOWN_POSITION;
        }

        double t = MathUtil.clamp((launchAngleDeg - ShotConstants.HOOD_DOWN_ANGLE_DEG) / span, 0, 1);
        return ShotConstants.HOOD_DOWN_POSITION
                + t * (ShotConstants.HOOD_UP_POSITION - ShotConstants.HOOD_DOWN_POSITION);
    }

    // With drag the ball needs more speed than the drag-free answer. Height at the
    // target's range rises with speed, so bisect between the drag-free speed and 3x it.
    private static double solveSpeedWithDrag(double range, double rise, double angle, double dragFree) {
        if (ShotConstants.DRAG_PER_M <= 0) {
            return dragFree;
        }

        double low = dragFree;
        double high = dragFree * 3;
        if (heightAtRange(high, angle, range) < rise) {
            return Double.NaN;
        }

        for (int i = 0; i < 24; i++) {
            double mid = 0.5 * (low + high);
            if (heightAtRange(mid, angle, range) < rise) {
                low = mid;
            } else {
                high = mid;
            }
        }
        return 0.5 * (low + high);
    }

    // Simulate the flight with quadratic drag and return the ball's height when it has
    // travelled `range` horizontally, or a very low value if it never gets there.
    private static double heightAtRange(double speed, double angle, double range) {
        double k = ShotConstants.DRAG_PER_M;
        double x = 0, y = 0;
        double vx = speed * Math.cos(angle);
        double vy = speed * Math.sin(angle);

        for (double t = 0; t < SIM_MAX_S; t += SIM_STEP_S) {
            double v = Math.hypot(vx, vy);
            double ax = -k * v * vx;
            double ay = -G - k * v * vy;

            double nextX = x + vx * SIM_STEP_S;
            double nextY = y + vy * SIM_STEP_S;

            if (nextX >= range) {
                double f = (range - x) / (nextX - x);
                return y + f * (nextY - y);
            }

            x = nextX;
            y = nextY;
            vx += ax * SIM_STEP_S;
            vy += ay * SIM_STEP_S;

            if (vx <= 1e-3) {
                break;
            }
        }
        return -1e9;
    }

    private static Solution unreachable(double angleDeg) {
        return new Solution(false, angleDeg, Double.NaN, ShotConstants.MAX_RPM, hoodPositionFor(angleDeg));
    }
}
