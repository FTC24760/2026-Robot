package org.firstinspires.ftc.teamcode.constants;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Sample rapid-fire autonomous. Inches and degrees, in a frame fixed at the robot's
 * starting spot: the robot starts at (0, 0) facing +X (away from the wall it is
 * touching, toward the HIVE in the middle of the field), and +Y is to its left.
 *
 * EVERY POSE HERE IS A PLACEHOLDER. Measure them on a real field before running.
 * Values are written for red; blue mirrors left and right (Y and heading flip sign),
 * which assumes the two sides of the field are mirror images.
 */
@Configurable
public class AutoConstants {
    // Aim with the Limelight when it is configured; otherwise the auto shoots at the
    // fixed TARGET_RPM without turning the turret.
    public static boolean USE_AUTO_AIM = true;

    // Where to shoot from, facing the HIVE.
    public static double SHOOT_X = 24;
    public static double SHOOT_Y = 0;
    public static double SHOOT_HEADING_DEG = 0;

    // Second cycle: drive to the start of the row of 4 POLLEN in our GARDEN, then
    // along it with the intake running.
    public static boolean SECOND_CYCLE_ENABLED = true;
    public static double COLLECT_START_X = 8;
    public static double COLLECT_START_Y = -36;
    public static double COLLECT_END_X = 8;
    public static double COLLECT_END_Y = -60;
    public static double COLLECT_HEADING_DEG = -90;

    // Lower the flower arm while collecting (for taking POLLEN from a FLOWER instead
    // of the GARDEN). Has no effect until the flower servo is on the robot.
    public static boolean COLLECT_WITH_FLOWER_ARM = false;

    // Burst timing (spin-up wait, feed time) is shared with teleop: see the BURST_
    // values in ShooterConstants.
    // Do not start the second cycle with less than this much of the 30 s left.
    public static double SECOND_CYCLE_MIN_TIME_LEFT_S = 12.0;
    public static double AUTO_LENGTH_S = 30.0;
}
