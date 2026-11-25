package org.firstinspires.ftc.teamcode.constants;


import org.firstinspires.ftc.teamcode.utils.CellSide;

/**
 * Every autonomous setting, including every robot pose the auto drives to.
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
public class AutoConstants {
    // Aim (turret, wheel speed, hood) with the Limelight when it is configured. Without
    // it the auto shoots at Configurables.SHOOTER_RPM and the turret stays put.
    public static boolean USE_AUTO_AIM = false;

    // ---- Which CELL of our HIVE is up at the start, as seen by our drivers. ----
    // By the manual (10.3.1 and Figure 10-2, TU08) every match starts with each
    // alliance's RIGHT CELL up. The driver can override it during init if the field was
    // set up differently.
    public static CellSide START_UP_SIDE = CellSide.RIGHT;
    // During init and the auto, believe the Limelight over the defaults above when it
    // sees which CELL is up (for example after a partner robot tips the HIVE).
    public static boolean DETECT_UP_SIDE = true;

    public static double FIELD_SIZE_IN = 144;

    // ==== ROBOT POSES for Auto: Red / Blue ====
    // Each is where the ROBOT CENTRE goes and which way the robot FRONT faces
    // (x, y in inches, heading in degrees; frame described at the top of this file).
    // Written for red; blue is worked out automatically. The shooter fires forward.
    // These are the only positions the auto uses. Defaults are read off the field
    // drawing and still need checking on a real field.

    // The robot is 17 in wide and 18 in deep, so against a wall its centre is 8.5 in
    // (side on) or 9 in (front or back on) from it.

    // Start: LEFT side against the audience wall, front facing our alliance wall (square
    // to it; heading 180). The turret aims at the CELL, so the pre-loads go straight away.
    public static double START_X = 59, START_Y = 8.5, START_H = 180;

    // Shooting position while our RIGHT CELL is up (the normal case). Same as the start:
    // the GARDEN run is straight there and straight back, and the turret aims.
    public static double SHOOT_X = 59, SHOOT_Y = 8.5, SHOOT_H = 180;

    // Shooting position if our LEFT CELL is up instead (HIVE tipped): against the far
    // wall. Same heading as everywhere else, so getting there needs no turn; the turret
    // aims.
    public static double SHOOT_LEFT_UP_X = 59, SHOOT_LEFT_UP_Y = 135.5, SHOOT_LEFT_UP_H = 180;

    // FLOWER poses are where the robot STOPS to collect. It lines up FLOWER_LINE_UP_IN
    // further back first, lowers the arm there, then creeps in. If it still goes in too
    // far, move these back (away from the FLOWER).

    // In front of the FLOWER on our wall (the left one from the start), ready to collect.
    public static double WALL_FLOWER_X = 19, WALL_FLOWER_Y = 48, WALL_FLOWER_H = 180;

    // In front of the FLOWER on the far (back) wall, ready to collect.
    public static double BACK_FLOWER_X = 48, BACK_FLOWER_Y = 125, BACK_FLOWER_H = 90;

    // GARDEN: from the shooting position, drive straight along the audience wall with the
    // intake running until the front reaches our wall, then straight back.
    public static double GARDEN_X = 9, GARDEN_Y = 8.5, GARDEN_H = 180;

    // Park partly in the LOADING ZONE (worth PARK points in AUTO), only in its lower
    // (audience) half so the upper half stays free. The zone is x 0..12, y 97..119
    // (middle at y 108). Side on (heading 180) the robot is 17 in along Y, so with its
    // centre at y 98 it covers y 89.5..106.5: 9.5 in inside the zone and 1.5 in short of
    // the middle. Keep PARK_Y <= 99 or it reaches into the upper half.
    public static boolean PARK_ENABLED = true;
    public static double PARK_MIN_TIME_LEFT_S = 3;
    // Wait this long after the last shot/cycle before driving to park.
    public static double PARK_DELAY_S = 8;
    public static double PARK_X = 10, PARK_Y = 98, PARK_H = 180;

    // Getting past the HIVE: a drive that crosses from the audience half to the far half
    // (or back) follows one smooth curve that bulges out toward our wall instead of going
    // straight through the HIVE. AROUND_HIVE_X sets how far it bulges: smaller = wider
    // berth. Checked clear of the HIVE for every drive in the auto at 20.
    public static double AROUND_HIVE_X = 20;

    // The turret aims in auto: at the Limelight's target when it has one, otherwise at
    // the up CELL's aim point below, from odometry. Set true to hold it straight ahead
    // and aim with the robot's heading instead.
    public static boolean LOCK_TURRET_FORWARD = false;
    // Where the turret aims when the Limelight has no target: the middle of each CELL of
    // our HIVE (as our drivers see them; RIGHT is up at the start).
    public static double RIGHT_CELL_X = 59, RIGHT_CELL_Y = 61;
    public static double LEFT_CELL_X = 59, LEFT_CELL_Y = 88;

    // ---- Collection cycles after the pre-loads. ----
    public enum CycleSource {
        WALL_FLOWER,
        GARDEN,
        BACK_FLOWER
    }

    // How many the auto runs; the driver can change it during init with the d-pad.
    public static int DEFAULT_CYCLES = 0;
    public static int MAX_CYCLES = 3;
    // The order they run in. Default: nearest first.
    public static CycleSource CYCLE_1 = CycleSource.GARDEN;
    public static CycleSource CYCLE_2 = CycleSource.WALL_FLOWER;
    public static CycleSource CYCLE_3 = CycleSource.BACK_FLOWER;
    // Skip a cycle that starts with less than this much of the auto left.
    public static double CYCLE_MIN_TIME_LEFT_S = 9;

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
    // How far back from the FLOWER pose the robot lines up and lowers the arm, before
    // creeping the rest of the way in.
    public static double FLOWER_LINE_UP_IN = 10;
    // How long to shift side to side with the arm down at each FLOWER.
    public static double FLOWER_COLLECT_S = 2.5;

    // ---- Step defaults. ----
    // Feed in timed push/retract pulses (timings in Configurables.PULSE_*), like
    // operator X in teleop. Read at INIT.
    public static boolean PULSE_SHOOT = true;
    // A drive whose target is closer than this is skipped, so the robot stays still
    // (the pre-load shot starts at its shooting pose: only the turret moves).
    public static double ALREADY_THERE_IN = 3.0;
    public static double ALREADY_THERE_DEG = 8.0;
    // Give up on a path that has not finished after this long (stuck or pushed).
    public static double DRIVE_TIMEOUT_S = 5.0;
    // How long a shot step feeds once the wheel is at speed. Pulse feeding puts 4 balls
    // in in about 1 s (PULSE_BALLS x (PULSE_PUSH + PULSE_RETRACT)); the rest is margin.
    // Raise it if the last ball is left behind.
    public static double SHOOT_FEED_S = 2;
    // Give up on a shot whose wheel never reaches speed after this long.
    public static double SHOOT_TIMEOUT_S = 4.0;

    public static double AUTO_LENGTH_S = 30.0;
}
