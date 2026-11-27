package org.firstinspires.ftc.teamcode.constants;


import org.firstinspires.ftc.teamcode.auto.AutoSelection;
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

    // Start presets, picked during init (see auto/StartPreset). Both are side on against a
    // wall, front facing our alliance wall (heading 180); the turret aims at the CELL, so
    // the pre-loads go straight away. Legal starts (G304): touching the wall, inside our
    // columns (red x <= 72), clear of the GARDEN, LOADING ZONE and FLOWER volumes.
    // A: audience wall. Footprint x 50..68, y 0..17.
    public static double START_A_X = 59, START_A_Y = 8.5, START_A_H = 180;
    // B: far wall. The FLOWER there spans x 45..51 and column C ends at x 72, so the
    // centre must be 60..63 (footprint 52.5..70.5 at 61.5). Only ~1.5 in of slack each
    // side: place it against a tape mark.
    public static double START_B_X = 61.5, START_B_Y = 135.5, START_B_H = 180;

    // Shooting position while our RIGHT CELL is up (the normal case). Same as the start:
    // the GARDEN run is straight there and straight back, and the turret aims.
    public static double SHOOT_X = 59, SHOOT_Y = 8.5, SHOOT_H = 180;

    // Shooting position if our LEFT CELL is up instead (HIVE tipped): against the far
    // wall. Same heading as everywhere else, so getting there needs no turn; the turret
    // aims. Same spot as start B: at x 59 the footprint would overlap the far-wall FLOWER
    // (x 45..51) by about an inch.
    public static double SHOOT_LEFT_UP_X = 61.5, SHOOT_LEFT_UP_Y = 135.5, SHOOT_LEFT_UP_H = 180;

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
    // SOLO and SECOND: park this long after the last shot or cycle. 0 = at once; the rest
    // of the time goes to more cycles (see CYCLE_MIN_TIME_LEFT_S).
    public static double PARK_DELAY_S = 0;
    // After a burst, wait at least TIP_SETTLE_MIN_S (the HIVE needs time to tip), then until
    // the camera has read one of our CELLS, but no longer than TIP_SETTLE_S, before leaving
    // the shot spot. The next trip then goes to the right side: the same spot if it did
    // not tip, the other CELL's if it did. 0 = do not wait. PLACEHOLDERS: time a real tip.
    public static double TIP_SETTLE_MIN_S = 0.4;
    public static double TIP_SETTLE_S = 1.5;
    // On arriving at a shot spot, wait up to this long for a camera read taken since the
    // trip began. If the up CELL changed meanwhile (the HIVE tipped), drive to the other
    // CELL's spot before spinning up. 0 = do not check.
    public static double ARRIVE_CHECK_S = 0.5;
    public static double PARK_X = 10, PARK_Y = 98, PARK_H = 180;
    // Reserve window: when this many seconds are left and the normal route has not started
    // its park drive, whatever step is running is abandoned and the robot drives to PARK
    // (LEAVE + PARK are banked even if a step is stuck or a cycle ran long). Must cover the
    // drive from anywhere on the route: at least DRIVE_TIMEOUT_S plus a margin. PLACEHOLDER:
    // time the longest drive to PARK. Needs PARK_ENABLED.
    public static double RESERVE_TIME_LEFT_S = 6;

    // ---- Partner coordination ----
    // SECOND slot: stop waiting as soon as the camera sees the HIVE tip (the partner has
    // shot); the delay stays as the fallback when there is no camera or no tip.
    public static boolean END_DELAY_ON_TIP = true;
    // Before the pre-loads (after any delay), if the start pose is not in the shooting zone
    // for the CELL that is up now (e.g. the HIVE tipped during the delay, or start B with
    // the audience CELL up), drive to the shot spot first.
    public static boolean DRIVE_TO_ZONE_FIRST = true;
    // FIRST slot: park this long after the last shot (instead of PARK_DELAY_S), so the
    // partner has the shooting area to itself.
    public static double FIRST_PARK_DELAY_S = 0;
    // SECOND slot shoots from the allowed point farthest from the fixed shot spot (where the
    // partner is expected). Candidates: the fixed spot turned about the CELL in steps of
    // SPOT_ARC_STEP_DEG, up to SPOT_ARC_STEPS each way; the same distance, so the same table
    // row. Spots within SPOT_KEEP_OUT_IN of a FLOWER are not used.
    public static boolean SECOND_USES_OFFSET_SPOT = true;
    public static double SPOT_ARC_STEP_DEG = 20;
    public static int SPOT_ARC_STEPS = 2;
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

    // Planned split (ideal case, nothing missed): SOLO takes the GARDEN then the back
    // FLOWER; FIRST the GARDEN then the wall FLOWER; SECOND the back FLOWER. The 20 POLLEN
    // are 4 pre-loaded on each robot and one cycle (4) at each source.
    public static CycleSource[] SOLO_SOURCES = {CycleSource.GARDEN, CycleSource.BACK_FLOWER, CycleSource.NONE};
    public static CycleSource[] FIRST_SOURCES = {CycleSource.GARDEN, CycleSource.WALL_FLOWER, CycleSource.NONE};
    public static CycleSource[] SECOND_SOURCES = {CycleSource.BACK_FLOWER, CycleSource.NONE, CycleSource.NONE};
    // FIRST and SECOND each shoot only into the CELL nearest their start (A: audience CELL,
    // B: far CELL), and only while it is up; each burst tips it and so frees the partner.
    // How long to wait for it to come up before giving up and shooting at whichever is up.
    // PLACEHOLDER: set from a timed run with the partner.
    public static double OWN_CELL_WAIT_S = 6;
    // SECOND's first burst does not wait OWN_CELL_WAIT_S after the delay (the wait would stack
    // on it). If the camera read the HIVE within this long and our CELL is still down, the
    // partner has not tipped it: SECOND goes to the other CELL. With no recent read it cannot
    // tell, and takes it that the partner (who agreed to shoot first) tipped it.
    public static double BLIND_READ_MAX_AGE_S = 2.0;
    // Stop waiting at this much of the auto left, whatever OWN_CELL_WAIT_S says: it is the
    // last moment to drive to the other CELL's spot, shoot and still bank PARK (the reserve
    // window starts at RESERVE_TIME_LEFT_S). PLACEHOLDER: time the trip and a burst.
    public static double GIVE_UP_AT_TIME_LEFT_S = 14;

    // How many the auto runs; the driver can change it during init with the d-pad.
    public static int DEFAULT_CYCLES = 3;
    public static int MAX_CYCLES = 3;

    // ---- Sharing the HIVE with an alliance partner (chosen during init). ----
    // When we agreed to shoot SECOND, wait this long before the pre-loads (bumpers adjust
    // it in steps of START_DELAY_STEP_S, up to MAX_START_DELAY_S). Measure the partner's
    // burst and agree on the number before the match.
    public static double DEFAULT_START_DELAY_S = 5.0;
    // FIRST/SECOND, if the first robot's pre-loads do not tip the HIVE (agree with the
    // partner, both pick the same; init menu: left stick press). LEAVE: FIRST parks and
    // SECOND drives over and shoots when its delay ends. STAY: FIRST keeps cycling until
    // the HIVE tips, then parks; SECOND waits for the tip instead of the delay.
    public static AutoSelection.NoTipPlan DEFAULT_NO_TIP_PLAN = AutoSelection.NoTipPlan.LEAVE;
    // STAY, SECOND: if the HIVE has not tipped by this much time left, stop waiting and go
    // anyway (the partner is evidently not tipping it).
    public static double SECOND_GO_ANYWAY_LEFT_S = 15;
    public static double START_DELAY_STEP_S = 0.5;
    public static double MAX_START_DELAY_S = 15.0;
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
    // SOLO picks each cycle's source as the cheapest left for the next shot (the CELL that is
    // up after the last burst), instead of the fixed SOLO_SOURCES order. FIRST/SECOND keep
    // their fixed lists so the partner can rely on them.
    public static boolean ADAPTIVE_SOLO_SOURCES = true;

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
