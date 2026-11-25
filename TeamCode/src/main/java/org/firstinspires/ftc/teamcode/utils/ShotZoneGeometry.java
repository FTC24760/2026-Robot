package org.firstinspires.ftc.teamcode.utils;

import org.firstinspires.ftc.teamcode.constants.ShootGateConstants;

/**
 * The one definition of "a shot from here can score", shared by TeleOp (ShootZone) and the
 * Auto init checks, so the two never disagree. Pure functions of positions, no hardware.
 *
 * A spot is in the shooting zone when (1) its distance to the up CELL is inside the shot
 * table's measured range (+ margin) and (2) the shot is within ZONE_MAX_OFF_AXIS_DEG of the
 * CELL's opening axis. The up CELL opens along field y: the AUDIENCE cell toward the
 * audience wall (-y), the FAR cell toward the far wall (+y), tilted 30 deg up (manual TU10
 * figs 9-9, 9-10). Field frame = the repo frame, same for both alliances.
 */
public final class ShotZoneGeometry {
    private ShotZoneGeometry() {
    }

    /** Distance from a spot to the CELL, inches. */
    public static double distanceIn(double x, double y, double cellX, double cellY) {
        return Math.hypot(cellX - x, cellY - y);
    }

    /**
     * Degrees the shot from a spot into the CELL is off the CELL's opening axis: 0 straight
     * in, 90 grazing, 180 from behind.
     */
    public static double offAxisDeg(double x, double y, double cellX, double cellY, HiveSide side) {
        double openY = side == HiveSide.AUDIENCE ? -1 : 1;
        double rx = x - cellX; // CELL -> spot
        double ry = y - cellY;
        double len = Math.hypot(rx, ry);
        if (len < 1e-6) {
            return 0;
        }
        return Math.toDegrees(Math.acos(Math.max(-1, Math.min(1, ry * openY / len))));
    }

    /**
     * The zone test, with hysteresis: easier to stay in than to get in. NaN distance or
     * angle (no target) is out of the zone.
     */
    public static boolean inZone(double distanceIn, double offAxisDeg, boolean wasIn) {
        double angleLimit = ShootGateConstants.ZONE_MAX_OFF_AXIS_DEG
                + (wasIn ? ShootGateConstants.ZONE_ANGLE_HYSTERESIS_DEG : 0);
        return ShotTable.inCalibratedRange(distanceIn,
                wasIn ? ShootGateConstants.ZONE_HYSTERESIS_IN : 0)
                && offAxisDeg <= angleLimit;
    }
}
