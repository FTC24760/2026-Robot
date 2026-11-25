package org.firstinspires.ftc.teamcode.constants;


/**
 * Inputs to ShotSolver, which picks a launch angle and wheel speed for a measured
 * distance. Metres, degrees and RPM. Everything marked CALIBRATE is a starting guess.
 *
 * Calibration order:
 *   1. Hood: measure the real launch angle at two servo positions.
 *   2. EXIT_SPEED_PER_RPM: at one known distance, adjust until shots land in the CELL.
 *   3. Check a near and a far distance. If far shots fall short, raise DRAG_PER_M;
 *      if only near shots are off, adjust TARGET_ABOVE_TAG_M or ENTRY_ANGLE_DEG.
 */
public class ShotConstants {

    // ---- Hood: servo position to launch angle (above horizontal). CALIBRATE. ----
    // One servo. Defaults match the old right-hand servo, which the positions below
    // were tuned on; change the name if the remaining servo is configured differently.
    public static String HOOD_SERVO_NAME = "rightHood";
    public static boolean HOOD_REVERSED = false;
    // Added to every position, to trim the hood without retuning the positions below.
    public static double HOOD_OFFSET = 0.0;
    public static double HOOD_DOWN_POSITION = 0.7;
    public static double HOOD_DOWN_ANGLE_DEG = 54.0;
    public static double HOOD_UP_POSITION = 0.95;
    public static double HOOD_UP_ANGLE_DEG = HOOD_DOWN_ANGLE_DEG + 18.0;

    // ---- Geometry. CALIBRATE. ----
    // Height of the ball above the tiles as it leaves the shooter.
    public static double EXIT_HEIGHT_M = 0.30;
    // How far in front of the robot centre the ball leaves the shooter.
    public static double EXIT_FORWARD_M = 0.0;
    // Height of the point to aim through, above the AprilTag cluster. The tags are on
    // the bottom face of the CELL and the opening is above them (the CELL is about
    // 0.30 m deep per the manual).
    public static double TARGET_ABOVE_TAG_M = 0.35;

    // ---- Shot shape. ----
    // How steeply the ball should be falling when it reaches the CELL. The CELL faces
    // up, so the ball has to drop in; steeper is more forgiving on distance but needs
    // more speed.
    public static double ENTRY_ANGLE_DEG = 50.0;

    // ---- Wheel speed to ball speed: anchored on a shot known to score. ----
    // The underhand shooter scores from the back of the field at 2100 RPM with the hood
    // at the bottom. The solver works out what ball speed that shot needs and scales
    // every other shot from it, so the model is exact at that spot.
    // To set it: park at that spot, run "Test: Hive Vision", and copy the distance and
    // tag height it shows. CALIBRATE both.
    public static double CAL_RPM = 3350;
    public static double CAL_DISTANCE_M = 2.60;
    public static double CAL_TAG_HEIGHT_M = 2.0;
    // Hood servo position used for that shot (the bottom).
    // The hood also sits here whenever auto-aim has no shot (start of the match, no CELL
    // known, out of range, or auto-aim off), so the fallback is the known-good shot.
    // Keep the hood at CAL_HOOD_POSITION for every auto-aim shot and vary only the wheel
    // speed, so the calibration shot is reproduced exactly at its spot. Set false to let
    // the solver also choose the hood angle (aiming for ENTRY_ANGLE_DEG).
    public static boolean PIN_HOOD_TO_CAL = false;
    // Used only if CAL_RPM is 0: ball exit speed in m/s per wheel RPM.
    public static double EXIT_SPEED_PER_RPM = 0.0032;
    // Air drag deceleration per (m/s)^2, in 1/m. 0.045 is the textbook value for a
    // smooth 71 mm, 25 g ball; set to 0 to ignore drag.
    public static double DRAG_PER_M = 0.045;

    // ---- Limits. ----
    // A shot needing a speed outside these counts as out of range. MAX_RPM must be above
    // CAL_RPM, or even the calibration shot is "out of range" and auto-aim never sets
    // the hood.
    public static double MIN_RPM = 1000;
    public static double MAX_RPM = 4000;
}
