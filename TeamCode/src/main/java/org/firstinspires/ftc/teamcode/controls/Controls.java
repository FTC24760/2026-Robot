package org.firstinspires.ftc.teamcode.controls;

import static org.firstinspires.ftc.teamcode.constants.ControlsConstants.BUTTERFLY_THRESHOLD;
import static org.firstinspires.ftc.teamcode.constants.ControlsConstants.TRIGGER_THRESHOLD;

import static java.lang.Math.abs;
import static java.lang.Math.signum;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;


/**
 * Both gamepads. Turns sticks and buttons into intents; Robot binds them to Ivy
 * commands (see Robot's constructor and controls/Bindings). Every method reads the
 * gamepads as they are now; Bindings does press/release detection.
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
 *   X / B            reset odometry to FieldConstants.RESET_POSE_1 / RESET_POSE_2
 */
public class Controls {
    Gamepad driver;
    Gamepad operator;

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
    public boolean shotModeToggle() {return operator.right_stick_button;}
    public boolean flowerToggle() {return driver.y || operator.y;}
    public boolean resetPose1() {return operator.x;}
    public boolean resetPose2() {return operator.b;}
    public boolean intake() {return driver.left_trigger > TRIGGER_THRESHOLD ||
                                    operator.left_trigger > TRIGGER_THRESHOLD;}

    public Controls(OpMode opMode) {
        driver = opMode.gamepad1;
        operator = opMode.gamepad2;
    }
}
