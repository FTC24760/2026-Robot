package org.firstinspires.ftc.teamcode.constants;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.utils.CellSide;

/**
 * Every autonomous setting: shared options, and the poses and options for Auto: Red /
 * Blue (the FullAuto routine). Field positions are in FieldConstants.
 *
 * Coordinates, in inches, as in Figure 10-2 of the manual (red on the left, audience at
 * the bottom):
 *   origin  the corner between the red ALLIANCE AREA wall and the audience wall
 *   +X      across the field toward blue
 *   +Y      away from the audience
 *   heading degrees, 0 = facing +X (toward blue), counter-clockwise positive
 * Every routine is written for RED. The field is the same for blue turned half a circle
 * about its centre, so blue poses are worked out automatically.
 */
@Configurable
public class AutoConstants {
    // Aim (turret, wheel speed, hood) with the Limelight when it is configured. Without
    // it the auto shoots at ShooterConstants.TARGET_RPM and the turret stays put.
    public static boolean USE_AUTO_AIM = true;

    // ---- Which CELL of our HIVE is up at the start, as seen by our drivers. ----
    // By the manual (10.3.1 and Figure 10-2, TU08) every match starts with each
    // alliance's RIGHT CELL up. The driver can override it during init if the field was
    // set up differently.
    public static CellSide START_UP_SIDE = CellSide.RIGHT;
    // During init and the auto, believe the Limelight over the defaults above when it
    // sees which CELL is up (for example after a partner robot tips the HIVE).
    public static boolean DETECT_UP_SIDE = true;

    public static double FIELD_SIZE_IN = 144;

    // ==== Poses for Auto: Red / Blue (written for red; see the frame above). ====
    // Starting guesses read off Figure 10-2, not measured on a field. Field features
    // (CELLS, FLOWERS, GARDEN, LOADING ZONE) are in FieldConstants.

    // ---- Start: touching our wall, just toward the audience from the LOADING ZONE. ----
    public static double START_X = 9, START_Y = 84, START_H = 0;

    // ---- Shooting: straight on to the up CELL. ----
    // The robot lines up level with the up CELL (same Y as it in FieldConstants), faces it
    // squarely (heading 0), and keeps the turret centred, so every shot is taken from the
    // same place along the same line. SHOOT_X is how far from our wall that spot is.
    public static double SHOOT_X = 12;
    // Keep the turret straight ahead while shooting in auto, so only the robot's heading
    // aims. Set false to let the turret aim at the CELL as in TeleOp.
    public static boolean LOCK_TURRET_FORWARD = true;

    // ---- Collection cycles after the pre-loads. ----
    public enum CycleSource {
        FAR_FLOWER,
        WALL_FLOWER,
        GARDEN
    }

    // How many the auto runs; the driver can change it during init with the d-pad.
    public static int DEFAULT_CYCLES = 2;
    public static int MAX_CYCLES = 3;
    // The order they run in. Default: nearest first.
    public static CycleSource CYCLE_1 = CycleSource.WALL_FLOWER;
    public static CycleSource CYCLE_2 = CycleSource.GARDEN;
    public static CycleSource CYCLE_3 = CycleSource.FAR_FLOWER;
    // Skip a cycle that starts with less than this much of the auto left.
    public static double CYCLE_MIN_TIME_LEFT_S = 9;

    // ---- GARDEN: drive along the audience wall toward our corner, intake first. ----
    public static double GARDEN_APPROACH_X = 32, GARDEN_APPROACH_Y = 10;
    public static double GARDEN_END_X = 10, GARDEN_END_Y = 10;
    public static double GARDEN_H = 180;

    // ---- Park partly in the LOADING ZONE (worth PARK points in AUTO). ----
    public static boolean PARK_ENABLED = true;
    public static double PARK_MIN_TIME_LEFT_S = 3;
    public static double PARK_X = 12, PARK_Y = 108, PARK_H = 0;

    // ---- Following the HIVE when it tips. ----
    // The Limelight is the main way the auto knows which CELL is up. As a backup when it
    // cannot see the HIVE, assume our own shots tipped it after this many balls into the
    // up CELL. 0 = do not guess (the manual only says "enough" POLLEN; set it once known).
    public static int BALLS_TO_TIP = 0;
    public static int BALLS_PER_SHOT = 4;
    // If the HIVE tips during a shot, stop feeding, let the turret swing to the new up
    // CELL for this long, then carry on.
    public static double SHOOT_RETARGET_PAUSE_S = 0.35;

    // ---- FLOWER collection. ----
    // How long to shift side to side with the arm down at each FLOWER.
    public static double FLOWER_COLLECT_S = 1.5;

    // ---- Step defaults. ----
    // Give up on a path that has not finished after this long (stuck or pushed).
    public static double DRIVE_TIMEOUT_S = 5.0;
    // How long a shot step feeds once the wheel is at speed. Enough for 4 balls at the
    // staggered feed rate.
    public static double SHOOT_FEED_S = 1.6;
    // Give up on a shot whose wheel never reaches speed after this long.
    public static double SHOOT_TIMEOUT_S = 4.0;

    public static double AUTO_LENGTH_S = 30.0;
}
