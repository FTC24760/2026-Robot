package org.firstinspires.ftc.teamcode.utils;

/**
 * Which CELL of our own HIVE, as seen by our drivers standing in our ALLIANCE AREA.
 *
 * The manual puts the red ALLIANCE AREA on the left as the audience sees the field, so
 * red drivers have the audience on their right and blue drivers have it on their left.
 * The HIVE starts with each alliance's RIGHT CELL up (Figure 10-2).
 */
public enum CellSide {
    LEFT,
    RIGHT;

    public CellSide other() {
        return this == LEFT ? RIGHT : LEFT;
    }

    /** Our drivers' view of a CELL given by which end of the field it is at. */
    public static CellSide of(Alliance alliance, HiveSide fieldSide) {
        boolean audience = fieldSide == HiveSide.AUDIENCE;
        if (alliance == Alliance.BLUE) {
            return audience ? LEFT : RIGHT;
        }
        return audience ? RIGHT : LEFT;
    }
}
