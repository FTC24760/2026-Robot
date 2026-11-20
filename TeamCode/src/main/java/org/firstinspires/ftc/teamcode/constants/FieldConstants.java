package org.firstinspires.ftc.teamcode.constants;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Where things are on the RED side of the field, in inches, in the frame described in
 * AutoConstants (origin at the red/audience corner, +X toward blue, +Y away from the
 * audience). Blue positions are worked out by turning these half a circle.
 *
 * Read off Figure 10-2 of the manual, about +/- 2 in. Measure on a real field and
 * correct them here; every routine uses these.
 */
@Configurable
public class FieldConstants {
    // ---- Our HIVE: the centre of each CELL, as our drivers see them. ----
    // The RIGHT CELL is up at the start of every match.
    public static double RIGHT_CELL_X = 58, RIGHT_CELL_Y = 62;
    public static double LEFT_CELL_X = 58, LEFT_CELL_Y = 91;

    // ---- The two FLOWERS on our side. ----
    // FACING_DEG is the heading the robot faces while collecting from the Retrieval
    // Opening at the bottom; the robot stops FLOWER_STANDOFF_IN short of the FLOWER.
    // On our wall, toward the audience from our ALLIANCE AREA:
    public static double WALL_FLOWER_X = 3, WALL_FLOWER_Y = 49, WALL_FLOWER_FACING_DEG = 180;
    // On the far wall (away from the audience):
    public static double FAR_FLOWER_X = 48, FAR_FLOWER_Y = 141, FAR_FLOWER_FACING_DEG = 90;
    // Distance from the robot centre to the FLOWER centre when collecting.
    public static double FLOWER_STANDOFF_IN = 14;

    // ---- Our GARDEN: 4 POLLEN along the audience wall, starting at our corner. ----
    public static double GARDEN_START_X = 1, GARDEN_END_X = 13, GARDEN_Y = 2;

    // ---- Our LOADING ZONE. ----
    public static double LOADING_ZONE_MIN_X = 0, LOADING_ZONE_MAX_X = 12;
    public static double LOADING_ZONE_MIN_Y = 97, LOADING_ZONE_MAX_Y = 119;
}
