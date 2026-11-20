package org.firstinspires.ftc.teamcode.constants;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.utils.CellSide;

/**
 * Settings shared by every autonomous routine. Each routine's own poses and options live
 * in its class under auto/routines.
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

    // ---- Collection cycles after the pre-loads. ----
    // How many the auto runs; the driver can change it during init with the d-pad.
    public static int DEFAULT_CYCLES = 2;
    public static int MAX_CYCLES = 3;

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
