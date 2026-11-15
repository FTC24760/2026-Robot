package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/** Shooter RPM/gains come from Panels; hold gamepad 1 right bumper for transfer. */
@Configurable
@TeleOp(name = "Shooter PIDF Dashboard Test", group = "Testing")
public class ShooterPidfDashboardTest extends ShooterTestBase {
    public static double TARGET_RPM = 0.0;
    // Software power coefficients, NOT REV hub setVelocityPIDFCoefficients values.
    // Tune in order: kF -> kS -> kP -> kD -> kI -> kA.
    public static double kP = 0.0;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.0;
    public static double kS = 0.0;
    public static double kA = 0.0;
    private static final double TRANSFER_POWER = 0.8;
    private DcMotor transferMotor;

    @Override
    public void init() {
        super.init();
        transferMotor = hardwareMap.get(DcMotor.class, "transfer");
        transferMotor.setPower(0.0);
        transferMotor.setDirection(DcMotor.Direction.REVERSE);
        // Transfer uses fixed power, with no velocity PIDF controller.
        transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        double transferPower = gamepad1.right_bumper ? TRANSFER_POWER : 0.0;
        transferMotor.setPower(transferPower);
        panelsTelemetry.addData("Transfer power", transferPower);
        super.loop();
    }

    @Override
    public void stop() {
        if (transferMotor != null) transferMotor.setPower(0.0);
        super.stop();
    }

    @Override
    protected double targetRpm() { return TARGET_RPM; }

    @Override
    protected void updateCoefficients() {
        controller.setCoefficients(kP, kI, kD, kF, kS, kA);
    }
}
