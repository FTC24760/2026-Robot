package org.firstinspires.ftc.teamcode.controls;

import static org.firstinspires.ftc.teamcode.constants.ControlsConstants.BUTTERFLY_THRESHOLD;
import static org.firstinspires.ftc.teamcode.constants.ControlsConstants.TRIGGER_THRESHOLD;

import static java.lang.Math.abs;
import static java.lang.Math.signum;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.utils.EdgeDetector;

public class Controls {
    Gamepad driver;
    Gamepad operator;

    private final EdgeDetector flowerToggle = new EdgeDetector();
    private final EdgeDetector resetPose1 = new EdgeDetector();
    private final EdgeDetector resetPose2 = new EdgeDetector();
    private final EdgeDetector shotModeToggle = new EdgeDetector();
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
