package org.firstinspires.ftc.teamcode.controls;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.constants.OperatorConstants;
import org.firstinspires.ftc.teamcode.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.utils.EdgeDetector;

/**
 * Right trigger: press to start/cancel shooting. Right bumper: hold intake.
 * Left bumper: hold outtake (takes priority). Y: toggle flower arm.
 * Right stick X aims the turret. Hood and RPM are set in Panels Configurables.
 */
public class OperatorControls {
    private final OpMode opMode;

    private final EdgeDetector shoot = new EdgeDetector();
    private final EdgeDetector flowerToggle = new EdgeDetector();
    private final EdgeDetector aimToggle = new EdgeDetector();

    private boolean shootPressed;
    private boolean flowerTogglePressed;
    private boolean aimTogglePressed;

    public OperatorControls(OpMode opMode) {
        this.opMode = opMode;
    }

    /** Call once per loop before reading any "pressed" (edge-triggered) control. */
    public void update() {
        Gamepad pad = operator();
        boolean trigger = pad.right_trigger > RobotConstants.TRIGGER_THRESHOLD;

        shootPressed = shoot.rose(trigger);
        flowerTogglePressed = flowerToggle.rose(pad.y);
        aimTogglePressed = aimToggle.rose(pad.a);
    }

    // ---- Same as the driver ----

    public boolean shootPressed() {
        return shootPressed;
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
