package org.firstinspires.ftc.teamcode.constants;

import com.bylazar.configurables.annotations.Configurable;

/**
 * TeleOp field poses, written for RED in the frame described in AutoConstants (inches,
 * heading in degrees); blue is worked out automatically. The CELL positions ShotSolver
 * aims at are AutoConstants.RIGHT_CELL_* and LEFT_CELL_*.
 */
@Configurable
public class FieldConstants {
    // Operator X: put the robot here and press X to tell odometry where it is.
    // Default: the auto start pose (left side on the audience wall, facing our wall).
    public static double RESET_POSE_1_X = 59, RESET_POSE_1_Y = 8.5, RESET_POSE_1_H = 180;
    // Operator B: second known spot. Default: against the far wall, facing our wall
    // (AutoConstants.SHOOT_LEFT_UP).
    public static double RESET_POSE_2_X = 59, RESET_POSE_2_Y = 135.5, RESET_POSE_2_H = 180;
}
