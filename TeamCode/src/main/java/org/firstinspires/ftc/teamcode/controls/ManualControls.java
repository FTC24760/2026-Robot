package org.firstinspires.ftc.teamcode.controls;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.utils.EdgeDetector;

/**
 * The original hold-to-run control scheme, kept for drivers who prefer to run each
 * mechanism by hand. Used when Robot.useManualControls() is called; the one-click
 * scheme is in DriverControls.
 *
 * Gamepad 1:
 *   left stick            drive (field-centric)    right stick x   rotate
 *   right trigger (hold)  spin the shooter         left trigger (hold)  intake + transfer
 *   X (hold)              transfer only (feed)     A               open / close the blocker
 *   B                     reset pose               Y               stop the shooter's idle spin
 *   right bumper (hold)   turret one way           left bumper (hold)   turret the other way
 */
public class ManualControls {
    private final OpMode opMode;
    private final EdgeDetector blockerToggle = new EdgeDetector();
    private final EdgeDetector shooterStop = new EdgeDetector();

    private boolean blockerTogglePressed;
    private boolean shooterStopPressed;

    public ManualControls(OpMode opMode) {
        this.opMode = opMode;
    }

    /** Call once per loop before reading any "pressed" (edge-triggered) control. */
    public void update() {
        blockerTogglePressed = blockerToggle.rose(driver().a);
        shooterStopPressed = shooterStop.rose(driver().y);
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

    public boolean resetPose() {
        return driver().b;
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

    /** Held: spin the shooter. */
    public boolean shoot() {
        return driver().right_trigger > RobotConstants.TRIGGER_THRESHOLD;
    }

    /** Held: run the intake roller and transfer. */
    public boolean intake() {
        return driver().left_trigger > RobotConstants.TRIGGER_THRESHOLD;
    }

    /** Held: run the transfer only. */
    public boolean transferOnly() {
        return driver().x;
    }

    /** True for one loop each time the blocker button is pressed. */
    public boolean blockerTogglePressed() {
        return blockerTogglePressed;
    }

    /** True for one loop each time the shooter-stop button is pressed. */
    public boolean shooterStopPressed() {
        return shooterStopPressed;
    }

    // Read through the OpMode every time rather than caching the Gamepad object.
    private Gamepad driver() {
        return opMode.gamepad1;
    }
}
