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
    // it the turret aims from odometry at the CELL aim points below, and RPM and hood
    // come from the shot table for the odometry distance.
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
    // In AUTO we may not go onto the opposing side (G402): red is field columns A-C, i.e.
    // x <= 72 in the red frame. A pose centre must stay this far inside, about half the
    // robot's longest side, so the robot's body stays on our side too.
    public static double OWN_SIDE_MAX_X_IN = 72;
    public static double OWN_SIDE_MARGIN_IN = 9;

    // ==== ROBOT POSES for Auto: Red / Blue ====
    // Each is where the ROBOT CENTRE goes and which way the robot FRONT faces
    // (x, y in inches, heading in degrees; frame described at the top of this file).
    // Written for red; blue is worked out automatically. The shooter fires forward.
    // These are the only positions the auto uses. Defaults are read off the field
    // drawing and still need checking on a real field.

    // The robot is 17 in wide and 18 in deep, so against a wall its centre is 8.5 in
    // (side on) or 9 in (front or back on) from it.

    // Start presets, picked during init (see auto/StartPreset). Both are back against a
    // wall, front (and the fixed camera) facing the HIVE, so the camera can watch it from
    // the start and the pre-loads go from here (both are inside the shooting zone). Legal
    // starts (G304): touching the wall, inside our columns (red x <= 72), clear of the
    // GARDEN, LOADING ZONE and FLOWER volumes.
    // A: audience wall, facing the AUDIENCE (RIGHT) CELL. Footprint x 50.5..67.5, y 0..18.
    public static double START_A_X = 59, START_A_Y = 9, START_A_H = 90;
    // B: far wall, facing the FAR (LEFT) CELL. The FLOWER there spans x 45..51 and column C
    // ends at x 72, so the centre must be 59.5..63.5 (footprint 53..70 at 61.5).
    public static double START_B_X = 61.5, START_B_Y = 135, START_B_H = 270;

    // ---- Where to shoot from: anywhere in the shooting zone (auto/ZoneTargets). ----
    // The zone is the TeleOp one (ShotZoneGeometry: distance ring + off-axis angle). The
    // auto drives to the in-zone point nearest to it that is ZONE_INSET_IN inside the edge,
    // faces the up CELL there (so the fixed camera sees it), and fires when the TeleOp
    // "ready to fire" test passes. Extra auto limits: not past the middle line (G402,
    // OWN_SIDE_*), WALL_TURN_CLEAR_IN from every wall (so it can turn there), SPOT_KEEP_OUT_IN
    // from FLOWERS, and in a pair PARTNER_CLEAR_IN from the partner's start (its auto is
    // unknown; it likely shoots from there). Searched on a ZONE_GRID_IN grid.
    public static double ZONE_INSET_IN = 3;
    public static double ZONE_GRID_IN = 2;
    public static double PARTNER_CLEAR_IN = 24;
    // Once at the point, wait up to this long for "ready to fire" (in zone, aim and hood
    // settled, wheel at speed, a recent camera read of the up CELL). Then fire anyway: the
    // shot step still waits for the wheel and aim (SHOOT_TIMEOUT_S).
    public static double READY_TIMEOUT_S = 1.5;
    // If the up CELL changes before firing (camera saw a tip), go to the other CELL's point;
    // at most this many times per shot.
    public static int MAX_REDIRECTS = 2;

    // ---- Turning near walls. ----
    // The robot (17 x 18 in) sweeps its corners out to 12.4 in from its centre when it
    // turns, so it only turns where its centre is at least WALL_TURN_CLEAR_IN from every
    // wall. A drive that starts or ends nearer a wall with a turn of more than
    // WALL_TURN_MAX_DEG first moves straight off the wall / finishes turning before moving
    // straight in. PLACEHOLDER: check with the real bumpers.
    public static double WALL_TURN_CLEAR_IN = 13.5;
    public static double WALL_TURN_MAX_DEG = 2;

    // FLOWER poses are where the robot STOPS to collect. It lines up FLOWER_LINE_UP_IN
    // further back first, lowers the arm there, then creeps in. If it still goes in too
    // far, move these back (away from the FLOWER).

    // In front of the FLOWER on our wall (the left one from the start), ready to collect.
    public static double WALL_FLOWER_X = 19, WALL_FLOWER_Y = 48, WALL_FLOWER_H = 180;

    // In front of the FLOWER on the far (back) wall, ready to collect.
    public static double BACK_FLOWER_X = 48, BACK_FLOWER_Y = 125, BACK_FLOWER_H = 90;

    // GARDEN: line up against the audience wall at GARDEN_LEAD_IN_X, then drive straight
    // along it with the intake running until the front reaches our wall.
    public static double GARDEN_X = 9, GARDEN_Y = 8.5, GARDEN_H = 180;
    public static double GARDEN_LEAD_IN_X = 32;

    // Park partly in the LOADING ZONE (worth PARK points in AUTO), only in its lower
    // (audience) half so the upper half stays free. The zone is x 0..12, y 97..119
    // (middle at y 108). Side on (heading 180) the robot is 17 in along Y, so with its
    // centre at y 98 it covers y 89.5..106.5: 9.5 in inside the zone and 1.5 in short of
    // the middle. Keep PARK_Y <= 99 or it reaches into the upper half.
    public static boolean PARK_ENABLED = true;
    public static double PARK_MIN_TIME_LEFT_S = 3;
    // Park this long after the last shot or cycle. 0 = at once; the rest
    // of the time goes to more cycles (see CYCLE_MIN_TIME_LEFT_S).
    public static double PARK_DELAY_S = 0;
    // SOLO, after a burst that brought our count to the tip threshold: keep watching the
    // HIVE until the camera sees it tip, at most this long, before planning the next trip.
    // It ends the moment the tip is seen (each HIVE tips at a slightly different speed, so
    // there is no fixed wait). Set it above the slowest tip seen. 0 = do not watch.
    public static double TIP_WATCH_S = 1.5;
    public static double PARK_X = 10, PARK_Y = 98, PARK_H = 180;
    // Reserve window: when this many seconds are left and the normal route has not started
    // its park drive, whatever step is running is abandoned and the robot drives to PARK
    // (LEAVE + PARK are banked even if a step is stuck or a cycle ran long). Must cover the
    // drive from anywhere on the route: at least DRIVE_TIMEOUT_S plus a margin. PLACEHOLDER:
    // time the longest drive to PARK. Needs PARK_ENABLED.
    public static double RESERVE_TIME_LEFT_S = 6;

    // Shot points are kept this far from FLOWERS.
    public static double SPOT_KEEP_OUT_IN = 14;

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
        BACK_FLOWER,
        /** No cycle: the slot has no more POLLEN to take. */
        NONE
    }

    // SOLO (decided with the team): keep shooting the CELL that is up (the other side once it
    // tips); before each shot after the pre-loads, collect from the unused source nearest to
    // the robot that still fits in the time left. Off: SOLO_SOURCES in order instead.
    // PAIR: only our side's sources: start A the GARDEN then the wall FLOWER, start B the back
    // FLOWER. The 20 POLLEN are 4 pre-loaded on each robot and one cycle (4) at each source.
    public static boolean SOLO_NEAREST_SOURCE = true;
    public static CycleSource[] SOLO_SOURCES = {CycleSource.GARDEN, CycleSource.WALL_FLOWER, CycleSource.BACK_FLOWER};
    public static CycleSource[] PAIR_A_SOURCES = {CycleSource.GARDEN, CycleSource.WALL_FLOWER, CycleSource.NONE};
    public static CycleSource[] PAIR_B_SOURCES = {CycleSource.BACK_FLOWER, CycleSource.NONE, CycleSource.NONE};

    // How many the auto runs; the driver can change it during init with the d-pad.
    public static int DEFAULT_CYCLES = 3;
    public static int MAX_CYCLES = 3;

    // A cycle (drive out, collect, drive back, burst) only starts if it fits in the time left
    // together with the park trip: trip times + BURST_S + park trip + CYCLE_MARGIN_S. The 30 s
    // does not hold every cycle, so later ones are dropped, never the park. Trip times are
    // worked out from the poses above: straight-line distance x CROSS_DETOUR when the drive
    // crosses between the audience and far halves (it bulges around the HIVE), at
    // AVG_SPEED_IN_PER_S, plus TURN_S per drive. PLACEHOLDERS at an assumed ~35 in/s: replace
    // with measured times from the "AutoStep" log lines (each step change is logged).
    public static double AVG_SPEED_IN_PER_S = 35;
    public static double TURN_S = 0.5;
    public static double CROSS_DETOUR = 1.25;
    // Lowering the arm at a FLOWER, on top of FLOWER_COLLECT_S.
    public static double FLOWER_ARM_S = 1.0;
    // One burst at the spot: spin-up, feed (SHOOT_FEED_S), tip settle.
    public static double BURST_S = 4.5;
    public static double CYCLE_MARGIN_S = 1.5;

    // ---- Following the HIVE when it tips. ----
    // The Limelight is the main way the auto knows which CELL is up. A tip needs balls in the
    // up CELL: the first only BALLS_FIRST_TIP (the CELL starts with 3 NECTAR in it), every
    // later one BALLS_NEXT_TIP. So the 20 POLLEN make at most three tips (4 + 8 + 8), and each
    // later tip takes two bursts. Our own balls are counted; the partner's are not. When the
    // camera cannot see the HIVE after a burst, believe the count and take it that the CELL
    // tipped; when it can, the camera wins.
    public static int BALLS_FIRST_TIP = 4;
    public static int BALLS_NEXT_TIP = 8;
    public static boolean PREDICT_TIP_WHEN_BLIND = true;
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
