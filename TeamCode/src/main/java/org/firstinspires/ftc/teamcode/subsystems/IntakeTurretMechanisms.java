package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.constants.RobotHardwareNames;
import org.firstinspires.ftc.teamcode.constants.ManualShooterTuning;

/** Turret, blocker and continuous intake/transfer control for manual shooter. */
public final class IntakeTurretMechanisms {
    private final DcMotor intake, transfer;
    private final Servo leftTurret, rightTurret, blocker;
    private double leftPosition = ManualShooterTuning.TURRET_INITIAL_POSITION;
    private double rightPosition = ManualShooterTuning.TURRET_INITIAL_POSITION;
    private boolean blockerOpen;
    private boolean lastAPressed;

    public IntakeTurretMechanisms(HardwareMap map) {
        intake = map.get(DcMotor.class, RobotHardwareNames.INTAKE);
        transfer = map.get(DcMotor.class, RobotHardwareNames.TRANSFER);
        transfer.setDirection(DcMotor.Direction.REVERSE);
        leftTurret = map.get(Servo.class, RobotHardwareNames.LEFT_TURRET);
        rightTurret = map.get(Servo.class, RobotHardwareNames.RIGHT_TURRET);
        blocker = map.get(Servo.class, RobotHardwareNames.BLOCKER);
        blocker.setPosition(ManualShooterTuning.BLOCKER_CLOSED_POSITION);
    }

    public void updateTurret(boolean rightBumper, boolean leftBumper, double dt) {
        // Preserve existing one-loop servo update ordering.
        leftTurret.setPosition(leftPosition);
        rightTurret.setPosition(rightPosition);
        double delta = ManualShooterTuning.TURRET_RATE_POSITION_PER_SECOND * dt;
        if (rightBumper && rightPosition < 1.0) {
            rightPosition = Math.min(1.0, rightPosition + delta);
            leftPosition = Math.min(1.0, leftPosition + delta);
        } else if (!rightBumper && leftBumper && rightPosition > 0.0) {
            rightPosition = Math.max(0.0, rightPosition - delta);
            leftPosition = Math.max(0.0, leftPosition - delta);
        }
    }

    public void updateFeed(double leftTrigger, boolean xPressed) {
        if (leftTrigger > ManualShooterTuning.FEED_TRIGGER_THRESHOLD) {
            intake.setPower(ManualShooterTuning.INTAKE_POWER);
            transfer.setPower(ManualShooterTuning.TRANSFER_POWER);
        } else if (xPressed) {
            intake.setPower(0.0);
            transfer.setPower(ManualShooterTuning.TRANSFER_POWER);
        } else {
            intake.setPower(0.0);
            transfer.setPower(0.0);
        }
    }

    public void updateBlocker(boolean aPressed) {
        if (aPressed && !lastAPressed) blockerOpen = !blockerOpen;
        lastAPressed = aPressed;
        blocker.setPosition(blockerOpen
                ? ManualShooterTuning.BLOCKER_OPEN_POSITION
                : ManualShooterTuning.BLOCKER_CLOSED_POSITION);
    }

    /** Preserve the legacy hub-PIDF per-loop turret increments. */
    public void updateTurretStep(boolean rightBumper, boolean leftBumper) {
        leftTurret.setPosition(leftPosition);
        rightTurret.setPosition(rightPosition);
        if (rightBumper && rightPosition < 1.0) {
            rightPosition += 0.005;
            leftPosition += 0.005;
        } else if (!rightBumper && leftBumper && rightPosition > 0.0) {
            rightPosition -= 0.005;
            leftPosition -= 0.005;
        }
    }

    /** Legacy mode opens the blocker while A is held (not toggle). */
    public void updateBlockerMomentary(boolean aPressed) {
        blockerOpen = aPressed;
        blocker.setPosition(aPressed
                ? ManualShooterTuning.BLOCKER_OPEN_POSITION
                : ManualShooterTuning.BLOCKER_CLOSED_POSITION);
    }

    public boolean isBlockerOpen() { return blockerOpen; }

    public void stop() {
        intake.setPower(0.0);
        transfer.setPower(0.0);
    }
}
