package org.firstinspires.ftc.teamcode.controls;

import static org.firstinspires.ftc.teamcode.constants.ControlsConstants.BUTTERFLY_THRESHOLD;
import static org.firstinspires.ftc.teamcode.constants.ControlsConstants.TRIGGER_THRESHOLD;

import static java.lang.Math.abs;
import static java.lang.Math.signum;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.utils.EdgeDetector;

/**
 * Both gamepads. Turns sticks and buttons into intents; Robot decides what they do.
 * Call update() once per loop, before reading any of the *Pressed fields.
 *
 * Driver (gamepad 1):
 *   left stick       drive (field-centric): up = forward, sideways = strafe
 *   right stick x    turn
 *   right stick y    butterfly engage / tank direction (not used by Robot yet)
 *   left trigger     intake
 *   right trigger    shoot (hold)
 *   Y                flower arm toggle
 *
 * Operator (gamepad 2):
 *   right stick x    manual turret
 *   right stick btn  toggle auto / manual aim
 *   left trigger     intake
 *   right trigger    shoot (hold)
 *   Y                flower arm toggle
 *   X / B            reset pose 1 / 2 (not used by Robot yet)
 */
public class Controls {
    Gamepad driver;
    Gamepad operator;

    private final EdgeDetector flowerToggle = new EdgeDetector();
    private final EdgeDetector resetPose1 = new EdgeDetector();
    private final EdgeDetector resetPose2 = new EdgeDetector();
    private final EdgeDetector shotModeToggle = new EdgeDetector();
    // True only on the loop a button goes down. Set by update().
    public boolean resetPose1Pressed;
    public boolean resetPose2Pressed;
    public boolean flowerTogglePressed;
    public boolean shotModeTogglePressed;
    public double manualTurret() {return operator.right_stick_x;}
    public double forward() {return -driver.left_stick_y;}
    public double strafe() {return driver.left_stick_x;}
    public double rotate() {return driver.right_stick_x;}

    public boolean butterflyEngage() {
        return abs(driver.right_stick_y) > BUTTERFLY_THRESHOLD;
    }
    public double tankDirection() {
        return signum(-driver.right_stick_y);
    }
    public boolean shoot() {return driver.right_trigger > TRIGGER_THRESHOLD ||
                                   operator.right_trigger > TRIGGER_THRESHOLD;}
    public boolean intake() {return driver.left_trigger > TRIGGER_THRESHOLD ||
                                    operator.left_trigger > TRIGGER_THRESHOLD;}

    public Controls(OpMode opMode) {
        driver = opMode.gamepad1;
        operator = opMode.gamepad2;
    }

    public void update() {
        flowerTogglePressed = flowerToggle.rose(driver.y || operator.y);
        resetPose1Pressed = resetPose1.rose(operator.x);
        resetPose2Pressed = resetPose2.rose(operator.b);
        shotModeTogglePressed = shotModeToggle.rose(operator.right_stick_button);
    }
}
