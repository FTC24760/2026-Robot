package org.firstinspires.ftc.teamcode.constants;

/**
 * Hardware configuration names and fixed geometry for JerryRigged TeleOp.
 * Names must match the Robot Controller configuration.
 */
public final class RobotHardwareNames {
    private RobotHardwareNames() {}

    public static final String FRONT_LEFT = "lf";
    public static final String FRONT_RIGHT = "rf";
    public static final String BACK_LEFT = "lb";
    public static final String BACK_RIGHT = "rb";
    public static final String INTAKE = "intake";
    public static final String TRANSFER = "transfer";
    public static final String SHOOTER = "shooter";
    public static final String LEFT_TURRET = "leftturret";
    public static final String RIGHT_TURRET = "rightturret";
    public static final String BLOCKER = "blocker";
    public static final String IMU = "imu";

    // Built-in encoder on the bare goBILDA 5000-series motor.
    // The shooter motor -> flywheel belt is 1:1.
    public static final double SHOOTER_TICKS_PER_REV = 28.0;
}
