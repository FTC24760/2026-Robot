
package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "one motor with pid", group = "Robot")
public class OneMotorWithPID extends OpMode {

    private DcMotorEx motor1;

    // goBILDA 5000 Series (no gearbox)
    private static final double TICKS_PER_REV = 28.0;

    // Target shooter speed (RPM)
    private static double TARGET_RPM = 2480;

    // Initial PIDF values - tune on your robot
    private static double P = 12.5;
    private static double I = 0.0;
    private static double D = 0.0;
    private static double F = 21.9;

    @Override
    public void init() {

        motor1 = hardwareMap.get(DcMotorEx.class, "motor1");
        // motor1.setDirection(DcMotor.Direction.REVERSE);

        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set velocity PIDF coefficients
        motor1.setVelocityPIDFCoefficients(P, I, D, F);

        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addData("Status", "Shooter Ready");
        telemetry.update();
    }

    @Override
    public void loop() {

        double targetTicksPerSecond = TARGET_RPM * TICKS_PER_REV / 60.0;

        // Hold right trigger to run shooter
        if (gamepad1.right_trigger > 0.2) {
            motor1.setVelocity(targetTicksPerSecond);
        } else {
            motor1.setVelocity(0);
        }

        // Measure current shooter speed
        double actualTicksPerSecond = motor1.getVelocity();
        double actualRPM = actualTicksPerSecond * 60.0 / TICKS_PER_REV;

        // Telemetry
        telemetry.addData("Target RPM", TARGET_RPM);
        telemetry.addData("Actual RPM", "%.1f", actualRPM);
        telemetry.addData("RPM Error", "%.1f", Math.abs(TARGET_RPM) - actualRPM);
        telemetry.addData("P", P);
        telemetry.addData("I", I);
        telemetry.addData("D", D);
        telemetry.addData("F", F);
        telemetry.update();
    }

    @Override
    public void stop() {
        motor1.setVelocity(0);
        motor1.setPower(0);
    }
}
