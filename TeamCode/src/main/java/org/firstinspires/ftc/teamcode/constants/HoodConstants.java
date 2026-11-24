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
}
