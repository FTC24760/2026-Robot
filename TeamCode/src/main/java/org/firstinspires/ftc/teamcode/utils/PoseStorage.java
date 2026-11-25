package org.firstinspires.ftc.teamcode.utils;

import com.pedropathing.math.Pose;

/**
 * Hands the robot's pose from the autonomous to the TeleOp that follows it. The auto sets
 * {@link #pose} when it ends; the next Robot takes it (and clears it) instead of assuming
 * the robot is at the start pose. Static, so it lasts between OpModes while the app runs.
 */
public final class PoseStorage {
    private PoseStorage() {
    }

    // Field-frame pose at the end of the last auto, or null if there isn't one to use.
    public static Pose pose;
}
