package org.firstinspires.ftc.teamcode.constants;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Measured shots: at this distance from the robot centre to the up CELL, this wheel speed
 * and hood position score. ShotTable interpolates between rows. Fill it with
 * "Test: Shot Calibration": park at a distance, adjust RPM and hood until shots go in,
 * press Y, and copy the printed row here. 5-6 rows across the shooting area is plenty.
 *
 * A row with distance 0 is unused. Rows may be in any order.
 *
 * The one row filled in is the competition setting (Configurables SHOOTER_RPM 3350, HOOD
 * 0.95) from start A, about 52.5 in from the RIGHT CELL. With only that row every distance
 * gets the same shot, i.e. the table behaves exactly like the fixed competition values
 * until more rows are measured.
 */
@Configurable
public class ShotTableConstants {
    // Off: always shoot at the fixed Configurables.SHOOTER_RPM / HOOD_POSITION.
    public static boolean ENABLED = true;

    public static double DIST_IN_1 = 52.5, RPM_1 = 3350, HOOD_1 = 0.95;
    public static double DIST_IN_2 = 0, RPM_2 = 0, HOOD_2 = 0;
    public static double DIST_IN_3 = 0, RPM_3 = 0, HOOD_3 = 0;
    public static double DIST_IN_4 = 0, RPM_4 = 0, HOOD_4 = 0;
    public static double DIST_IN_5 = 0, RPM_5 = 0, HOOD_5 = 0;
    public static double DIST_IN_6 = 0, RPM_6 = 0, HOOD_6 = 0;

    // Beyond the nearest / farthest row the end row's shot is used (never extrapolated).
    // Further than this outside the measured range, the shot is flagged out of range on
    // telemetry: it is a guess, measure a row there.
    public static double OUT_OF_RANGE_MARGIN_IN = 6;
}
