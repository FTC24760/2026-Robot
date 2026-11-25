package org.firstinspires.ftc.teamcode.controls;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.constants.OperatorConstants;
import org.firstinspires.ftc.teamcode.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.utils.EdgeDetector;

/**
 * Right trigger: press to start/cancel shooting. Right bumper: hold intake.
 * Left bumper: hold outtake (takes priority). Y: toggle flower arm. X: toggle pulse shoot.
 * Right stick X aims the turret. D-pad up/down adjusts RPM by 50 per press.
 * Hood and RPM are also set in Panels Configurables.
 */
public class OperatorControls {
    private final OpMode opMode;
    public boolean shootHeld;

    private final EdgeDetector shoot = new EdgeDetector();
    private final EdgeDetector rpmUp = new EdgeDetector();
    private final EdgeDetector rpmDown = new EdgeDetector();
    private boolean rpmUpPressed;
    private boolean rpmDownPressed;
    private final EdgeDetector flowerToggle = new EdgeDetector();
    private final EdgeDetector aimToggle = new EdgeDetector();

    private boolean shootPressed;
    private boolean flowerTogglePressed;
    private boolean aimTogglePressed;
    private final EdgeDetector pulseToggle = new EdgeDetector();
    private boolean pulseTogglePressed;

    public OperatorControls(OpMode opMode) {
        this.opMode = opMode;
    }

    /** Call once per loop before reading any "pressed" (edge-triggered) control. */
    public void update() {
        Gamepad pad = operator();
        boolean trigger = pad.right_trigger > RobotConstants.TRIGGER_THRESHOLD;
        shootHeld = pad.right_trigger > RobotConstants.TRIGGER_THRESHOLD;
        shootPressed = shoot.rose(trigger);
        rpmUpPressed = rpmUp.rose(pad.dpad_up);
        rpmDownPressed = rpmDown.rose(pad.dpad_down);
        flowerTogglePressed = flowerToggle.rose(pad.y);
        aimTogglePressed = aimToggle.rose(pad.a);
        pulseTogglePressed = pulseToggle.rose(pad.x);
    }

    /** X: switch pulse shooting on/off. */
    public boolean pulseTogglePressed() {
        return pulseTogglePressed;
    }

    // ---- Same as the driver ----

    public boolean shootPressed() {
        return shootPressed;
    }

    public boolean rpmUpPressed() {
        return rpmUpPressed;
    }

    public boolean rpmDownPressed() {
        return rpmDownPressed;
    }

    public boolean intake() {
        return operator().right_bumper;
    }

    public boolean flowerTogglePressed() {
        return flowerTogglePressed;
    }

    public boolean aimTogglePressed() {
        return aimTogglePressed;
    }

    public boolean reverse() {
        return operator().left_bumper;
    }

    // ---- Manual aim ----

    /** -1 to 1, positive = turret toward higher servo positions. */
    public double turret() {
        double x = operator().right_stick_x;
        return Math.abs(x) < OperatorConstants.STICK_DEADZONE ? 0 : x;
    }

    // Read through the OpMode every time rather than caching the Gamepad object.
    private Gamepad operator() {
        return opMode.gamepad2;
    }
}
