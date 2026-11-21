package org.firstinspires.ftc.teamcode.constants;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.utils.Alliance;

/** TeleOp starting poses in the field frame documented in AutoConstants. */
public class TeleOpConstants {
    // Provisional defaults from the existing autonomous starts, NOT an auto-end pose.
    // Set these to the actual robot centre at TeleOp INIT: inches and degrees CCW.
    public static double RED_START_X = 59;
    public static double RED_START_Y = 8.5;
    public static double RED_START_HEADING_DEG = 180;

    public static double BLUE_START_X = 85;
    public static double BLUE_START_Y = 135.5;
    public static double BLUE_START_HEADING_DEG = 0;

    public static Pose startingPose(Alliance alliance) {
        return alliance == Alliance.BLUE
                ? new Pose(BLUE_START_X, BLUE_START_Y, Math.toRadians(BLUE_START_HEADING_DEG))
                : new Pose(RED_START_X, RED_START_Y, Math.toRadians(RED_START_HEADING_DEG));
    }
}
