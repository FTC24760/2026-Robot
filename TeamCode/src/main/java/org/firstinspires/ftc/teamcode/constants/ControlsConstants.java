package org.firstinspires.ftc.teamcode.constants;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
/** Thresholds for turning analog gamepad inputs into on/off controls. */
public class ControlsConstants {
    // A trigger counts as pressed above this (0 to 1).
    public static double TRIGGER_THRESHOLD = 0.2;
    // The driver's right stick y must pass this (0 to 1) to engage the butterfly.
    public static double BUTTERFLY_THRESHOLD = 0.8;
}
