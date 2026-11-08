package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.constants.JerryRiggedHardwareConstants;
import org.firstinspires.ftc.teamcode.constants.JerryRiggedTeleOpConstants;

/** Turret, blocker and continuous intake/transfer control for JerryRigged. */
public final class JerryRiggedMechanisms {
    private final DcMotor intake, transfer;
    private final Servo leftTurret, rightTurret, blocker;
    private double leftPosition = JerryRiggedTeleOpConstants.TURRET_INITIAL_POSITION;
    private double rightPosition = JerryRiggedTeleOpConstants.TURRET_INITIAL_POSITION;
    private boolean blockerOpen;
    private boolean lastAPressed;

    public JerryRiggedMechanisms(HardwareMap map) {
        intake = map.get(DcMotor.class, JerryRiggedHardwareConstants.INTAKE);
        transfer = map.get(DcMotor.class, JerryRiggedHardwareConstants.TRANSFER);
        transfer.setDirection(DcMotor.Direction.REVERSE);
        leftTurret = map.get(Servo.class, JerryRiggedHardwareConstants.LEFT_TURRET);
        rightTurret = map.get(Servo.class, JerryRiggedHardwareConstants.RIGHT_TURRET);
        blocker = map.get(Servo.class, JerryRiggedHardwareConstants.BLOCKER);
        blocker.setPosition(JerryRiggedTeleOpConstants.BLOCKER_CLOSED_POSITION);
    }

    public void updateTurret(boolean rightBumper, boolean leftBumper, double dt) {
        // Preserve existing one-loop servo update ordering.
        leftTurret.setPosition(leftPosition);
        rightTurret.setPosition(rightPosition);
        double delta = JerryRiggedTeleOpConstants.TURRET_RATE_POSITION_PER_SECOND * dt;
        if (rightBumper && rightPosition < 1.0) {
            rightPosition = Math.min(1.0, rightPosition + delta);
            leftPosition = Math.min(1.0, leftPosition + delta);
        } else if (!rightBumper && leftBumper && rightPosition > 0.0) {
            rightPosition = Math.max(0.0, rightPosition - delta);
            leftPosition = Math.max(0.0, leftPosition - delta);
        }
    }

    public void updateFeed(double leftTrigger, boolean xPressed) {
        if (leftTrigger > JerryRiggedTeleOpConstants.FEED_TRIGGER_THRESHOLD) {
            intake.setPower(JerryRiggedTeleOpConstants.INTAKE_POWER);
            transfer.setPower(JerryRiggedTeleOpConstants.TRANSFER_POWER);
        } else if (xPressed) {
            intake.setPower(0.0);
            transfer.setPower(JerryRiggedTeleOpConstants.TRANSFER_POWER);
        } else {
            intake.setPower(0.0);
            transfer.setPower(0.0);
        }
    }

    public void updateBlocker(boolean aPressed) {
        if (aPressed && !lastAPressed) blockerOpen = !blockerOpen;
        lastAPressed = aPressed;
        blocker.setPosition(blockerOpen
                ? JerryRiggedTeleOpConstants.BLOCKER_OPEN_POSITION
                : JerryRiggedTeleOpConstants.BLOCKER_CLOSED_POSITION);
    }

    public boolean isBlockerOpen() { return blockerOpen; }

    public void stop() {
        intake.setPower(0.0);
        transfer.setPower(0.0);
    }
}
