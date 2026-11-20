package org.firstinspires.ftc.teamcode.utils;

/**
 * The four CELLS on the HIVE structure. Each alliance's HIVE has two CELLS on a pivot;
 * one faces up (the one to launch into) and one faces down at any time.
 *
 * Every CELL has a cluster of four 36h11 AprilTags on its bottom face, pointing down at
 * the tiles. IDs are from the BIOBUZZ Competition Manual, section 9.9 (TU08).
 */
public enum HiveCell {
    RED_FAR(Alliance.RED, HiveSide.FAR, 30),
    RED_AUDIENCE(Alliance.RED, HiveSide.AUDIENCE, 34),
    BLUE_AUDIENCE(Alliance.BLUE, HiveSide.AUDIENCE, 38),
    BLUE_FAR(Alliance.BLUE, HiveSide.FAR, 42);

    public static final int TAGS_PER_CELL = 4;

    public final Alliance alliance;
    public final HiveSide side;
    public final int firstTagId;

    HiveCell(Alliance alliance, HiveSide side, int firstTagId) {
        this.alliance = alliance;
        this.side = side;
        this.firstTagId = firstTagId;
    }

    /** @return the CELL this tag is on, or null if the tag is not a HIVE tag. */
    public static HiveCell fromTagId(int tagId) {
        for (HiveCell cell : values()) {
            if (tagId >= cell.firstTagId && tagId < cell.firstTagId + TAGS_PER_CELL) {
                return cell;
            }
        }
        return null;
    }
}
