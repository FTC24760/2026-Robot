package org.firstinspires.ftc.teamcode.controls;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.constants.OperatorConstants;
import org.firstinspires.ftc.teamcode.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.utils.EdgeDetector;

/**
 * Gamepad 2, the operator. Mechanisms work the same as on the driver's gamepad (either
 * player can use them); driving stays with the driver.
 *
 *   right trigger   SHOOT (hold)                left trigger   intake on / off
 *   Y               flower arm up / down        X (hold)       reverse intake and transfer
 *   A               auto-aim on / off           B              open / close the blocker
 *
 *   Options (PS5) / Start   MANUAL AIM on / off. While on, the operator sets these and
 *                           auto-aim leaves them alone:
 *     right stick x     turret left / right (stops at the turret limits)
 *     d-pad up / down   hood up / down (hold; stops at the hood limits)
 *     d-pad right / left  wheel speed + / - RPM_STEP per press
 *   Manual aim starts from wherever turret, hood and wheel speed were when it was
 *   switched on, so nothing jumps.
 */
public class OperatorControls {
    private final OpMode opMode;

    private final EdgeDetector shoot = new EdgeDetector();
    private final EdgeDetector shootRelease = new EdgeDetector();
    private final EdgeDetector intakeToggle = new EdgeDetector();
    private final EdgeDetector flowerToggle = new EdgeDetector();
    private final EdgeDetector aimToggle = new EdgeDetector();
    private final EdgeDetector blockerToggle = new EdgeDetector();
    private final EdgeDetector manualToggle = new EdgeDetector();
    private final EdgeDetector rpmUp = new EdgeDetector();
    private final EdgeDetector rpmDown = new EdgeDetector();

    private boolean shootPressed;
    private boolean shootReleased;
    private boolean intakeTogglePressed;
    private boolean flowerTogglePressed;
    private boolean aimTogglePressed;
    private boolean blockerTogglePressed;
    private boolean manualTogglePressed;
    private boolean rpmUpPressed;
    private boolean rpmDownPressed;

    public OperatorControls(OpMode opMode) {
        this.opMode = opMode;
    }

    /** Call once per loop before reading any "pressed" (edge-triggered) control. */
    public void update() {
        Gamepad pad = operator();
        boolean trigger = pad.right_trigger > RobotConstants.TRIGGER_THRESHOLD;

        shootPressed = shoot.rose(trigger);
        shootReleased = shootRelease.rose(!trigger);
        intakeTogglePressed = intakeToggle.rose(pad.left_trigger > RobotConstants.TRIGGER_THRESHOLD);
        flowerTogglePressed = flowerToggle.rose(pad.y);
        aimTogglePressed = aimToggle.rose(pad.a);
        blockerTogglePressed = blockerToggle.rose(pad.b);
        // PS5 Options button; the SDK reports it as both start and options.
        manualTogglePressed = manualToggle.rose(pad.start || pad.options);
        rpmUpPressed = rpmUp.rose(pad.dpad_right);
        rpmDownPressed = rpmDown.rose(pad.dpad_left);
    }

    // ---- Same as the driver ----

    public boolean shootPressed() {
        return shootPressed;
    }

    /** True for one loop when the shoot trigger is let go (stops the shot). */
    public boolean shootReleased() {
        return shootReleased;
    }

    public boolean intakeTogglePressed() {
        return intakeTogglePressed;
    }

    public boolean flowerTogglePressed() {
        return flowerTogglePressed;
    }

    public boolean aimTogglePressed() {
        return aimTogglePressed;
    }

    public boolean blockerTogglePressed() {
        return blockerTogglePressed;
    }

    public boolean reverse() {
        return operator().x;
    }

    // ---- Manual aim ----

    public boolean manualTogglePressed() {
        return manualTogglePressed;
    }

    /** -1 to 1, positive = turret toward higher servo positions. */
    public double turret() {
        double x = operator().right_stick_x;
        return Math.abs(x) < OperatorConstants.STICK_DEADZONE ? 0 : x;
    }

    /** +1 while d-pad up is held, -1 for d-pad down, otherwise 0. */
    public double hood() {
        if (operator().dpad_up) {
            return 1;
        }
        if (operator().dpad_down) {
            return -1;
        }
        return 0;
    }

    public boolean rpmUpPressed() {
        return rpmUpPressed;
    }

    public boolean rpmDownPressed() {
        return rpmDownPressed;
    }

    // Read through the OpMode every time rather than caching the Gamepad object.
    private Gamepad operator() {
        return opMode.gamepad2;
    }
}
