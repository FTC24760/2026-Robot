package org.firstinspires.ftc.teamcode.controls;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.utils.EdgeDetector;

/**
 * Right trigger: press to start/cancel shooting. Right bumper: hold intake.
 * Left bumper: hold outtake (takes priority). Y: toggle flower arm.
 * Left stick drives; right stick X rotates. D-pad left stops shooter. Up/down adjusts RPM by 50 per press.
 * Create/Back resets pose. Turret aiming uses gamepad 2 right stick X.
 */
public class DriverControls {
    private final OpMode opMode;
    public boolean shootHeld = false;
    private final EdgeDetector shoot = new EdgeDetector();
    private final EdgeDetector rpmUp = new EdgeDetector();
    private final EdgeDetector rpmDown = new EdgeDetector();
    private boolean rpmUpPressed;
    private boolean rpmDownPressed;
    private final EdgeDetector flowerToggle = new EdgeDetector();
    private final EdgeDetector aimToggle = new EdgeDetector();
    private final EdgeDetector shooterStop = new EdgeDetector();
    private final EdgeDetector resetPose = new EdgeDetector();

    private boolean shootPressed;
    private boolean flowerTogglePressed;
    private boolean aimTogglePressed;
    private boolean shooterStopPressed;
    private boolean resetPosePressed;

    public DriverControls(OpMode opMode) {
        this.opMode = opMode;
    }

    /** Call once per loop before reading any "pressed" (edge-triggered) control. */
    public void update() {
        Gamepad pad = driver();

        shootPressed = shoot.rose(pad.right_trigger > RobotConstants.TRIGGER_THRESHOLD);
        shootHeld = pad.right_trigger > RobotConstants.TRIGGER_THRESHOLD;
        rpmUpPressed = rpmUp.rose(pad.dpad_up);
        rpmDownPressed = rpmDown.rose(pad.dpad_down);
        flowerTogglePressed = flowerToggle.rose(pad.y);
        aimTogglePressed = aimToggle.rose(pad.a);
        shooterStopPressed = shooterStop.rose(pad.dpad_left);
        // PS5 Create button; the SDK reports it as both back and share.
        resetPosePressed = resetPose.rose(driver().back || driver().share);
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

    /** True for one loop each time Create (PS5) / Back is pressed. */
    public boolean resetPose() {
        return resetPosePressed;
    }

    // ---- Everyday controls ----

    /** True for one loop each time the shoot trigger is pulled. */
    public boolean shootPressed() {
        return shootPressed;
    }

    /** Held: run intake and transfer forward. */
    public boolean rpmUpPressed() {
        return rpmUpPressed;
    }

    public boolean rpmDownPressed() {
        return rpmDownPressed;
    }

    public boolean intake() {
        return driver().left_trigger > 0.2;
    }

    public boolean flowerTogglePressed() {
        return flowerTogglePressed;
    }

    public boolean aimTogglePressed() {
        return aimTogglePressed;
    }

    /** Held: run the intake and transfer backwards. */
    public boolean reverse() {
        return driver().left_bumper;
    }

    // ---- Manual backups ----

    public boolean shooterStopPressed() {
        return shooterStopPressed;
    }

    // Read through the OpMode every time rather than caching the Gamepad object.
    public Gamepad driver() {
        return opMode.gamepad1;
    }
}
