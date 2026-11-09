package org.firstinspires.ftc.teamcode.controls;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.utils.EdgeDetector;

/**
 * The one place that maps gamepad buttons to robot actions.
 * To rebind a control, change it here; nothing else reads the gamepad.
 *
 * Gamepad 1:
 *   left stick      drive (field-centric)        right stick x   rotate
 *   right trigger   spin up shooter              left trigger    intake + transfer
 *   X               transfer only (feed)         A               toggle blocker
 *   B               reset field-centric heading
 *   right bumper    turret one way AND invert butterflies
 *   left bumper     turret the other way
 */
public class DriverControls {
    private final OpMode opMode;
    private final EdgeDetector blockerToggle = new EdgeDetector();

    private boolean blockerTogglePressed;

    public DriverControls(OpMode opMode) {
        this.opMode = opMode;
    }

    /** Call once per loop before reading any "pressed" (edge-triggered) control. */
    public void update() {
        blockerTogglePressed = blockerToggle.rose(driver().a);
    }

    public double forward() {
        return -driver().left_stick_y;
    }

    public double strafe() {
        return driver().left_stick_x;
    }

    public double rotate() {
        return driver().right_stick_x;
    }

    public boolean resetYaw() {
        return driver().b;
    }

    public boolean butterflyInvert() {
        return driver().right_bumper;
    }

    /** +1 while the right bumper is held, -1 for the left bumper, otherwise 0. */
    public double turretDirection() {
        if (driver().right_bumper) {
            return 1.0;
        }
        if (driver().left_bumper) {
            return -1.0;
        }
        return 0.0;
    }

    public boolean shoot() {
        return driver().right_trigger > RobotConstants.TRIGGER_THRESHOLD;
    }

    public boolean intake() {
        return driver().left_trigger > RobotConstants.TRIGGER_THRESHOLD;
    }

    public boolean transferOnly() {
        return driver().x;
    }

    /** True for one loop each time the blocker button is pressed. */
    public boolean blockerTogglePressed() {
        return blockerTogglePressed;
    }

    // Read through the OpMode every time rather than caching the Gamepad object.
    private Gamepad driver() {
        return opMode.gamepad1;
    }
}
