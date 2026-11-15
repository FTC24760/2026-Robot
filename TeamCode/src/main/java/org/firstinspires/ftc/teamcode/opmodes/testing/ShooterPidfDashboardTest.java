package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/** After START, set RPM and gains live in Panels. No gamepad input is used. */
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

    @Override
    protected double targetRpm() { return TARGET_RPM; }

    @Override
    protected void updateCoefficients() {
        controller.setCoefficients(kP, kI, kD, kF, kS, kA);
    }
}
