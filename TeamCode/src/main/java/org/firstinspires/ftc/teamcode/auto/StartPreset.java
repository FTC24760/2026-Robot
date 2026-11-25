package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.constants.AutoConstants;

/**
 * Where the robot is placed before the match, picked during init. The poses are in
 * AutoConstants, written for red; ctx.pose() turns them half a circle for blue, so A and B
 * mean the same place relative to our drivers on either alliance.
 */
public enum StartPreset {
    A("audience wall"),
    B("far wall");

    /** Shown on telemetry so the drive team knows where to put the robot. */
    public final String where;

    StartPreset(String where) {
        this.where = where;
    }

    /** This preset's robot pose for our alliance. */
    public Pose pose(AutoContext ctx) {
        return this == A
                ? ctx.pose(AutoConstants.START_A_X, AutoConstants.START_A_Y, AutoConstants.START_A_H)
                : ctx.pose(AutoConstants.START_B_X, AutoConstants.START_B_Y, AutoConstants.START_B_H);
    }

    public StartPreset next() {
        StartPreset[] all = values();
        return all[(ordinal() + 1) % all.length];
    }

    public StartPreset previous() {
        StartPreset[] all = values();
        return all[(ordinal() + all.length - 1) % all.length];
    }
}
