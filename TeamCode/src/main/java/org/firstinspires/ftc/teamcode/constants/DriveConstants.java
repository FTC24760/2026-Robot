package org.firstinspires.ftc.teamcode.constants;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Configurable
public class DriveConstants {
    public static String FRONT_LEFT_NAME = "lf";
    public static String FRONT_RIGHT_NAME = "rf";
    public static String BACK_LEFT_NAME = "lb";
    public static String BACK_RIGHT_NAME = "rb";
    public static String IMU_NAME = "imu";

    public static final DcMotorSimple.Direction LEFT_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction RIGHT_DIRECTION = DcMotorSimple.Direction.FORWARD;

    public static final RevHubOrientationOnRobot.LogoFacingDirection IMU_LOGO_DIRECTION =
            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
    public static final RevHubOrientationOnRobot.UsbFacingDirection IMU_USB_DIRECTION =
            RevHubOrientationOnRobot.UsbFacingDirection.UP;
}
