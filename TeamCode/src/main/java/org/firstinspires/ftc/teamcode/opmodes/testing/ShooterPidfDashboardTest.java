package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/** Shooter RPM/gains come from Panels; hold gamepad 1 right bumper for intake/transfer. */
@Configurable
@TeleOp(name = "Shooter PIDF Dashboard Test", group = "Testing")
public class ShooterPidfDashboardTest extends ShooterTestBase {
    public static double TARGET_RPM = 3500;
    // Software power coefficients, NOT REV hub setVelocityPIDFCoefficients values.
    // Tune in order: kF -> kS -> kP -> kD -> kI -> kA.
    public static double kP = 0.006;
    public static double kI = 0.0;
    public static double kD = 0.00001;
    public static double kF = 0.00045;
    public static double kS = 0.12;
    public static double kA = 0.0;
    private static final double TRANSFER_POWER = 0.8;
    // Matches INTAKE_POWER in main's constants/IntakeConstants.java.
    private static final double INTAKE_POWER = 0.6;
    private DcMotor transferMotor;
    private DcMotor intakeMotor;

    @Override
    public void init() {
        super.init();
        transferMotor = hardwareMap.get(DcMotor.class, "transfer");
        transferMotor.setPower(0.0);
        transferMotor.setDirection(DcMotor.Direction.REVERSE);
        // Transfer uses fixed power, with no velocity PIDF controller.
        transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        intakeMotor.setPower(0.0);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        double transferPower = gamepad1.right_bumper ? TRANSFER_POWER : 0.0;
        transferMotor.setPower(transferPower);
        double intakePower = gamepad1.right_bumper ? INTAKE_POWER : 0.0;
        intakeMotor.setPower(intakePower);
        panelsTelemetry.addData("Transfer power", transferPower);
        panelsTelemetry.addData("Intake power", intakePower);
        super.loop();
    }

    @Override
    public void stop() {
        if (transferMotor != null) transferMotor.setPower(0.0);
        if (intakeMotor != null) intakeMotor.setPower(0.0);
        super.stop();
    }

    @Override
    protected double targetRpm() { return TARGET_RPM; }

    @Override
    protected void updateCoefficients() {
        controller.setCoefficients(kP, kI, kD, kF, kS, kA);
    }
}
