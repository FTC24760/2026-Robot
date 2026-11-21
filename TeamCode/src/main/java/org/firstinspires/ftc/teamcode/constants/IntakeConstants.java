package org.firstinspires.ftc.teamcode.constants;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

/** Intake roller, transfer, and the blocker servo that gates the shooter. */
public class IntakeConstants {
    public static String INTAKE_MOTOR_NAME = "intake";
    public static String TRANSFER_MOTOR_NAME = "transfer";
    public static String BLOCKER_SERVO_NAME = "blocker";

    public static final DcMotorSimple.Direction TRANSFER_DIRECTION = DcMotorSimple.Direction.REVERSE;

    // Transfer power at RobotConstants.NOMINAL_VOLTAGE.
    // Backup: power for running the intake and transfer backwards to clear a jam.
    // Scale transfer power by battery voltage so balls reach the shooter at the same
    // speed on a fresh battery as on a tired one.
    public static boolean TRANSFER_VOLTAGE_COMPENSATION = true;


    // Flower arm: a servo that lowers for collecting from a FLOWER. Not on the robot
    // yet; until a servo with this name is configured the arm does nothing. CALIBRATE.
    public static String FLOWER_SERVO_NAME = "flower";

    // Getting POLLEN out of a FLOWER: with the arm down, the robot strafes side to side
    // under motor power to work the POLLEN out of the Retrieval Opening at the bottom.
    // Total sideways travel (one side to the other), in inches.
    public static double FLOWER_SHIFT_IN = 3.0;
    // Strafe power while shifting. Raise it if the shake is too gentle.
    public static double FLOWER_SHIFT_POWER = 0.6;
    // Longest time on one side before switching, in case it cannot reach (e.g. pressed
    // against the FLOWER).
    public static double FLOWER_SHIFT_PERIOD_S = 0.25;
    // TeleOp: shift automatically while the arm is down and the drive sticks are still.
    // Touching a stick stops it at once.
    public static boolean FLOWER_SHIFT_IN_TELEOP = true;
}
