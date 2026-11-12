package org.firstinspires.ftc.teamcode.constants;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Limelight mounting and HIVE detection. Distances are in metres, angles in degrees.
 * Everything marked CALIBRATE is a starting guess: check it with the "Test: Hive Vision"
 * OpMode against a tape measure before trusting auto-aim.
 */
@Configurable
public class VisionConstants {
    public static String LIMELIGHT_NAME = "limelight";
    // The Limelight pipeline must be an AprilTag (36h11) pipeline with tag size 82.5 mm
    // (3.25 in) and full 3D enabled.
    public static int PIPELINE = 0;
    public static int POLL_RATE_HZ = 100;

    // ---- Where the camera is on the robot. CALIBRATE. ----
    // True if the Limelight is mounted on the turret and turns with it; false if it is
    // fixed to the chassis. This decides how the turret auto-aligns.
    public static boolean CAMERA_ON_TURRET = false;
    // How far the lens points above horizontal.
    public static double CAMERA_PITCH_DEG = 45.0;
    // True when the camera is mounted upside down and the image is NOT already flipped in
    // the Limelight's own settings. If tag heights read negative, flip this.
    public static boolean CAMERA_UPSIDE_DOWN = true;
    // Which way the lens faces, measured to the left of robot forward (0 = straight ahead).
    public static double CAMERA_YAW_DEG = 0.0;
    // Lens position relative to the centre of the robot at floor level.
    public static double CAMERA_FORWARD_M = 0.0;
    public static double CAMERA_LEFT_M = 0.0;
    public static double CAMERA_UP_M = 0.25;

    // ---- Which CELL is up. CALIBRATE. ----
    // A CELL whose tags sit higher than this above the tiles counts as the upward-facing
    // one. Read "Tag height" in the test OpMode with the HIVE tipped each way and set this
    // halfway between the two readings. Only used when one CELL of the HIVE is visible;
    // when both are, the higher one is taken as up.
    public static double CELL_UP_MIN_TAG_HEIGHT_M = 1.0;

    // ---- Filtering. ----
    // A detection older than this is ignored and auto-aim falls back to fixed values.
    public static double TARGET_TIMEOUT_MS = 500;
    // Smoothing of distance and bearing: 0 = none, closer to 1 = steadier but slower.
    public static double SMOOTHING = 0.5;
}
