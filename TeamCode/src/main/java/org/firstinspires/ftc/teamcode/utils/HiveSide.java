package org.firstinspires.ftc.teamcode.utils;

/**
 * Which CELL of an alliance's HIVE is meant: the one at the audience end of the field or
 * the one at the far end. Field-fixed, so it means the same thing from either alliance
 * station, unlike "left" and "right".
 */
public enum HiveSide {
    AUDIENCE,
    FAR;

    public HiveSide other() {
        return this == AUDIENCE ? FAR : AUDIENCE;
    }
}
