package org.firstinspires.ftc.teamcode.constants;

import com.bylazar.configurables.annotations.Configurable;

/** Hood servo settings. */
@Configurable
public class HoodConstants {
    // One servo. Defaults match the old right-hand servo, which the positions below
    // were tuned on; change the name if the remaining servo is configured differently.
    public static String HOOD_SERVO_NAME = "rightHood";
    public static boolean HOOD_REVERSED = false;
    // Added to every position, to trim the hood without retuning the positions below.
    public static double HOOD_OFFSET = 0.0;
    public static double HOOD_DOWN_POSITION = 0.7;
    public static double HOOD_UP_POSITION = 0.95;

    // Auto-aim hood position from distance and wheel speed, a linear regression fitted in
    // Desmos to points from the "Test: Shot Tuning" OpMode:
    //   hood = HOOD_INTERCEPT + HOOD_DISTANCE_COEF * distance + HOOD_RPM_COEF * RPM
    // distance in inches from the turret pivot to the CELL centre, RPM the target speed.
    // In Desmos: h_1 ~ a + b d_1 + c r_1. The defaults hold the hood up until it's fitted.
    // The result is clamped between HOOD_DOWN_POSITION and HOOD_UP_POSITION.
    public static double HOOD_INTERCEPT = 0.95;
    public static double HOOD_DISTANCE_COEF = 0.0;
    public static double HOOD_RPM_COEF = 0.0;
}
