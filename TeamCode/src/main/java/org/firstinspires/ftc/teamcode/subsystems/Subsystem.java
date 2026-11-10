package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * One mechanism on the robot. A subsystem owns its hardware and exposes
 * intent-level methods ("open", "shoot", "drive"); it never reads a gamepad.
 */
public interface Subsystem {
    /** Put the mechanism in a safe, unpowered state. Called when the OpMode stops. */
    void stop();

    /** Add this subsystem's status lines. The caller is responsible for telemetry.update(). */
    void addTelemetry(Telemetry telemetry);
}
