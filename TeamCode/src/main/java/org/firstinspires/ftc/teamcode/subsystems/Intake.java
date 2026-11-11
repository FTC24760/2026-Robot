package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.utils.MathUtil;

/** Intake roller, transfer, and the blocker servo that gates the path into the shooter. */
public class Intake implements Subsystem {

    public enum Mode {
        /** Everything off. */
        OFF,
        /** Intake roller and transfer both running. */
        INTAKE,
        /** Transfer only, to feed what is already inside the robot. */
        TRANSFER_ONLY
    }

    private final DcMotor intakeMotor;
    private final DcMotor transferMotor;
    private final Servo blockerServo;

    private Mode mode = Mode.OFF;
    private boolean blockerOpen;
    private double voltageScale = 1.0;
    private double transferPower;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, IntakeConstants.INTAKE_MOTOR_NAME);
        transferMotor = hardwareMap.get(DcMotor.class, IntakeConstants.TRANSFER_MOTOR_NAME);
        blockerServo = hardwareMap.get(Servo.class, IntakeConstants.BLOCKER_SERVO_NAME);

        transferMotor.setDirection(IntakeConstants.TRANSFER_DIRECTION);

        setBlockerOpen(false);
    }

    /**
     * Give the transfer the latest battery reading. Call before setMode() each loop.
     *
     * @param voltageScale nominal voltage / battery voltage
     */
    public void setVoltageScale(double voltageScale) {
        this.voltageScale = voltageScale;
    }

    public void setMode(Mode mode) {
        this.mode = mode;

        if (mode == Mode.OFF) {
            transferPower = 0.0;
        } else {
            double scale = IntakeConstants.TRANSFER_VOLTAGE_COMPENSATION ? voltageScale : 1.0;
            transferPower = MathUtil.clamp(IntakeConstants.TRANSFER_POWER * scale, 0.0, 1.0);
        }

        intakeMotor.setPower(mode == Mode.INTAKE ? IntakeConstants.INTAKE_POWER : 0.0);
        transferMotor.setPower(transferPower);
    }

    public Mode getMode() {
        return mode;
    }

    /** True whenever the transfer is pushing toward the shooter. */
    public boolean isFeeding() {
        return mode != Mode.OFF;
    }

    public void setBlockerOpen(boolean open) {
        blockerOpen = open;
        blockerServo.setPosition(open
                ? IntakeConstants.BLOCKER_OPEN_POSITION
                : IntakeConstants.BLOCKER_CLOSED_POSITION);
    }

    public void toggleBlocker() {
        setBlockerOpen(!blockerOpen);
    }

    public boolean isBlockerOpen() {
        return blockerOpen;
    }

    @Override
    public void stop() {
        setMode(Mode.OFF);
    }

    @Override
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Intake", mode);
        telemetry.addData("Transfer power", "%.2f", transferPower);
        telemetry.addData("Blocker open", blockerOpen ? "yes" : "no");
    }
}
