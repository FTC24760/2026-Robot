package org.firstinspires.ftc.teamcode.controls;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.utils.EdgeDetector;

/**
 * The one place that maps gamepad buttons to robot actions.
 * To rebind a control, change it here; nothing else reads the gamepad.
 *
 * Gamepad 1, everyday controls:
 *   left stick      drive (field-centric)       right stick x   rotate
 *   right trigger   SHOOT: press once to spin up, open the blocker and feed;
 *                   press again to stop early (it also stops by itself)
 *   left trigger    intake on / off
 *   Y               flower arm up / down
 *   X (hold)        reverse intake and transfer, to clear a jam
 *   A               auto-aim on / off (turret, wheel speed and hood from the Limelight)
 *   B               reset pose (current position and heading become the field origin)
 *   bumpers (hold)  turn the turret by hand: right bumper one way, left the other.
 *                   Overrides auto-align while held.
 *
 * Gamepad 1, manual backups on the d-pad:
 *   up (hold)       spin the shooter without feeding
 *   right (hold)    run the transfer only
 *   down            open / close the blocker
 *   left            stop the shooter completely (cancels a shot and the idle spin)
 */
public class DriverControls {
    private final OpMode opMode;

    private final EdgeDetector shoot = new EdgeDetector();
    private final EdgeDetector intakeToggle = new EdgeDetector();
    private final EdgeDetector flowerToggle = new EdgeDetector();
    private final EdgeDetector aimToggle = new EdgeDetector();
    private final EdgeDetector blockerToggle = new EdgeDetector();
    private final EdgeDetector shooterStop = new EdgeDetector();

    private boolean shootPressed;
    private boolean intakeTogglePressed;
    private boolean flowerTogglePressed;
    private boolean aimTogglePressed;
    private boolean blockerTogglePressed;
    private boolean shooterStopPressed;

    public DriverControls(OpMode opMode) {
        this.opMode = opMode;
    }

    /** Call once per loop before reading any "pressed" (edge-triggered) control. */
    public void update() {
        Gamepad pad = driver();

        shootPressed = shoot.rose(pad.right_trigger > RobotConstants.TRIGGER_THRESHOLD);
        intakeTogglePressed = intakeToggle.rose(pad.left_trigger > RobotConstants.TRIGGER_THRESHOLD);
        flowerTogglePressed = flowerToggle.rose(pad.y);
        aimTogglePressed = aimToggle.rose(pad.a);
        blockerTogglePressed = blockerToggle.rose(pad.dpad_down);
        shooterStopPressed = shooterStop.rose(pad.dpad_left);
    }

    // ---- Driving ----

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

    // ---- Everyday controls ----

    /** True for one loop each time the shoot trigger is pulled. */
    public boolean shootPressed() {
        return shootPressed;
    }

    /** True for one loop each time the intake trigger is pulled. */
    public boolean intakeTogglePressed() {
        return intakeTogglePressed;
    }

    public boolean flowerTogglePressed() {
        return flowerTogglePressed;
    }

    public boolean aimTogglePressed() {
        return aimTogglePressed;
    }

    /** Held: run the intake and transfer backwards. */
    public boolean reverse() {
        return driver().x;
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

    // ---- Manual backups ----

    /** Held: spin the shooter without feeding. */
    public boolean manualSpin() {
        return driver().dpad_up;
    }

    /** Held: run the transfer only. */
    public boolean transferOnly() {
        return driver().dpad_right;
    }

    public boolean blockerTogglePressed() {
        return blockerTogglePressed;
    }

    public boolean shooterStopPressed() {
        return shooterStopPressed;
    }

    // Read through the OpMode every time rather than caching the Gamepad object.
    private Gamepad driver() {
        return opMode.gamepad1;
    }
}
