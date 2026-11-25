package org.firstinspires.ftc.teamcode.controls;

import static org.firstinspires.ftc.teamcode.constants.ControlsConstants.BUTTERFLY_THRESHOLD;
import static org.firstinspires.ftc.teamcode.constants.ControlsConstants.TRIGGER_THRESHOLD;

import static java.lang.Math.abs;
import static java.lang.Math.signum;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.utils.EdgeDetector;

/**
 * Both gamepads. Turns sticks and buttons into intents; Robot binds them to Ivy
 * commands (see Robot's constructor and controls/Bindings). Call update() once per loop,
 * before reading any of the *Pressed fields.
 *
 * Driver (gamepad 1):
 *   left stick       drive (field-centric): up = forward, sideways = strafe
 *   right stick x    turn
 *   right stick y    hold up or down: butterfly pods down, field-oriented tank drive.
 *                    Up faces the left stick, down faces away from it (drives backward)
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

    private final EdgeDetector resetPose1 = new EdgeDetector();
    private final EdgeDetector resetPose2 = new EdgeDetector();
    // True only on the loop a button goes down. Set by update().
    public boolean resetPose1Pressed;
    public boolean resetPose2Pressed;
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
    // Held state; Bindings does the edge detection.
    public boolean shotModeToggle() {return operator.right_stick_button;}
    public boolean flowerToggle() {return driver.y || operator.y;}
    public boolean intake() {return driver.left_trigger > TRIGGER_THRESHOLD ||
                                    operator.left_trigger > TRIGGER_THRESHOLD;}

    public Controls(OpMode opMode) {
        driver = opMode.gamepad1;
        operator = opMode.gamepad2;
    }

    public void update() {
        resetPose1Pressed = resetPose1.rose(operator.x);
        resetPose2Pressed = resetPose2.rose(operator.b);
    }
}
