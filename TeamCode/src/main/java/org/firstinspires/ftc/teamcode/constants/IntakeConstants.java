package org.firstinspires.ftc.teamcode.constants;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/** Intake roller, transfer, and the blocker servo that gates the shooter. */
@Configurable
public class IntakeConstants {
    public static String INTAKE_MOTOR_NAME = "intake";
    public static String TRANSFER_MOTOR_NAME = "transfer";
    public static String BLOCKER_SERVO_NAME = "blocker";

    public static final DcMotorSimple.Direction TRANSFER_DIRECTION = DcMotorSimple.Direction.REVERSE;

    public static double INTAKE_POWER = 0.6;
    public static double TRANSFER_POWER = 0.8;

    public static double BLOCKER_OPEN_POSITION = 0.032;
    public static double BLOCKER_CLOSED_POSITION = 0.0;
}
