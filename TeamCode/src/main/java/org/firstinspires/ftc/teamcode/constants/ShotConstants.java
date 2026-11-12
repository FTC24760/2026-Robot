package org.firstinspires.ftc.teamcode.constants;

import com.bylazar.configurables.annotations.Configurable;

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
@Configurable
public class ShotConstants {

    // ---- Hood: servo position to launch angle (above horizontal). CALIBRATE. ----
    public static String HOOD_SERVO_NAME = "hood";
    public static double HOOD_DOWN_POSITION = 0.0;
    public static double HOOD_DOWN_ANGLE_DEG = 65.0;
    public static double HOOD_UP_POSITION = 1.0;
    public static double HOOD_UP_ANGLE_DEG = 40.0;

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

    // ---- Wheel speed to ball speed. CALIBRATE. ----
    // Ball exit speed in m/s per wheel RPM. A 72 mm wheel has a surface speed of
    // 0.00377 m/s per RPM; the ball leaves somewhat slower than that.
    public static double EXIT_SPEED_PER_RPM = 0.0032;
    // Air drag deceleration per (m/s)^2, in 1/m. 0.045 is the textbook value for a
    // smooth 71 mm, 25 g ball; set to 0 to ignore drag.
    public static double DRAG_PER_M = 0.045;

    // ---- Limits. ----
    public static double MIN_RPM = 1000;
    public static double MAX_RPM = 3000;
}
