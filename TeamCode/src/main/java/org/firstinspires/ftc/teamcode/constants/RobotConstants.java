package org.firstinspires.ftc.teamcode.constants;

import com.bylazar.configurables.annotations.Configurable;

/** Robot-wide values that do not belong to a single subsystem. */
@Configurable
public class RobotConstants {
    // How far a trigger must be pulled before it counts as pressed.
    public static double TRIGGER_THRESHOLD = 0.2;

    // Loop time is capped at this so a stall never causes a huge jump in rate-based motion.
    public static double MAX_LOOP_SECONDS = 0.1;

    // Battery voltage the motor gains were tuned at; output is scaled by NOMINAL / actual.
    public static double NOMINAL_VOLTAGE = 12.0;
    // Battery voltage is slow to change and slow to read, so only sample it this often.
    public static double VOLTAGE_READ_PERIOD_MS = 250;

    // Shooter diagnostics: CSV sample period and throttled telemetry period.
    public static double RPM_LOG_PERIOD_MS = 20;
    public static double DIAGNOSTIC_TELEMETRY_PERIOD_MS = 200;
}
