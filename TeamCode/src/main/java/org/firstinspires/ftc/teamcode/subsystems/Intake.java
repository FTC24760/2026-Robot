package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.utils.MathUtil;

/**
 * Intake roller, transfer, and the blocker servo that gates the path into the shooter.
 * Set intakeOn, transferOn and blockerOpen, then call update() once per loop; nothing
 * moves until update(). Transfer power is scaled by Robot.voltageScale.
 */
public class Intake implements Subsystem {

    private final DcMotor intakeMotor;
    private final DcMotor transferMotor;
    private final Servo blockerServo;

    // What the intake should do. Set these, then call update() to apply them.
    public boolean intakeOn;
    public boolean transferOn;
    public boolean blockerOpen;
    private double transferPower;
    // Transfer power at nominal voltage, for example a slower feed while shooting.
    // NaN means use IntakeConstants.TRANSFER_POWER.
    public double transferPowerOverride = Double.NaN;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, IntakeConstants.INTAKE_MOTOR_NAME);
        transferMotor = hardwareMap.get(DcMotor.class, IntakeConstants.TRANSFER_MOTOR_NAME);
        blockerServo = hardwareMap.get(Servo.class, IntakeConstants.BLOCKER_SERVO_NAME);

        transferMotor.setDirection(IntakeConstants.TRANSFER_DIRECTION);

        blockerServo.setPosition(IntakeConstants.BLOCKER_CLOSED_POSITION);
    }

    /** Apply intakeOn, transferOn and blockerOpen to the motors and servo. Call every loop. */
    public void update() {
        intakeMotor.setPower(intakeOn ? IntakeConstants.INTAKE_POWER : 0.0);

        if (transferOn) {
            double scale = IntakeConstants.TRANSFER_VOLTAGE_COMPENSATION ? Robot.voltageScale : 1.0;
            double base = Double.isNaN(transferPowerOverride)
                    ? IntakeConstants.TRANSFER_POWER
                    : transferPowerOverride;
            transferPower = MathUtil.clamp(base * scale, 0.0, 1.0);
        } else {
            transferPower = 0.0;
        }
        transferMotor.setPower(transferPower);

        blockerServo.setPosition(blockerOpen
                ? IntakeConstants.BLOCKER_OPEN_POSITION
                : IntakeConstants.BLOCKER_CLOSED_POSITION);
    }

    @Override
    public void stop() {
        intakeOn = false;
        transferOn = false;
        update();
    }
    @Override
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Intake", intakeOn ? "on" : "off");
        telemetry.addData("Transfer power", "%.2f", transferPower);
        telemetry.addData("Blocker open", blockerOpen ? "yes" : "no");
    }
}
