package org.firstinspires.ftc.teamcode.utils;

import com.pedropathing.math.Pose;

/**
 * Carries the robot's pose from the end of AUTO into TeleOp, so odometry (and aiming from
 * it) does not jump back to a fixed start pose. Static: it lives as long as the Robot
 * Controller app, which stays up between AUTO and TeleOp.
 *
 * TeleOp takes it once, and only if it is from the same alliance and recent, so a stale
 * pose from an earlier match or practice run is never used.
 */
public final class PoseHandoff {
    // Longer than the 8 s AUTO-to-TeleOp transition plus time for a slow INIT press.
    public static double MAX_AGE_S = 60;

    private static Pose pose;
    private static Alliance alliance;
    private static long savedNs;
    // Which CELL AUTO believed was up, and what the last take() found (null: none).
    private static CellSide upSide;
    private static CellSide takenUpSide;

    private PoseHandoff() {
    }

    /** Record the latest pose. AUTO calls this every loop and when it stops. */
    public static void save(Pose latest, Alliance from, CellSide believedUp) {
        pose = latest;
        alliance = from;
        upSide = believedUp;
        savedNs = System.nanoTime();
    }

    /**
     * The saved pose if it is from this alliance and at most MAX_AGE_S old, else null.
     * Either way the saved pose is cleared, so it is used at most once.
     */
    public static Pose take(Alliance forAlliance) {
        Pose result = null;
        takenUpSide = null;
        if (pose != null && alliance == forAlliance
                && (System.nanoTime() - savedNs) * 1e-9 <= MAX_AGE_S) {
            result = pose;
            takenUpSide = upSide;
        }
        pose = null;
        upSide = null;
        return result;
    }

    /** The CELL AUTO believed was up, from the last take() that returned a pose; else null. */
    public static CellSide takenUpSide() {
        return takenUpSide;
    }
}
