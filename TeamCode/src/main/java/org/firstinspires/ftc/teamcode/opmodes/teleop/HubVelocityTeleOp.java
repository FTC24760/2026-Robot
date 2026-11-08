package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.constants.RobotHardwareNames;
import org.firstinspires.ftc.teamcode.subsystems.ButterflyController;
import org.firstinspires.ftc.teamcode.subsystems.FieldCentricMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.IntakeTurretMechanisms;

/** Legacy shooter mode using the REV hub velocity PIDF loop (not software PIDF). */
@TeleOp(name = "Hub Velocity TeleOp", group = "Robot")
public class HubVelocityTeleOp extends OpMode {
    public static double TARGET_RPM = 1525;
    public static double P = 30, I = 0.05, D = 0.0, F = 29;

    private FieldCentricMecanumDrive drive;
    private IntakeTurretMechanisms mechanisms;
    private ButterflyController butterfly;
    private DcMotorEx shooterMotor;

    @Override
    public void init() {
        drive = new FieldCentricMecanumDrive(hardwareMap);
        mechanisms = new IntakeTurretMechanisms(hardwareMap);
        butterfly = new ButterflyController(hardwareMap);
        shooterMotor = hardwareMap.get(DcMotorEx.class, RobotHardwareNames.SHOOTER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setVelocityPIDFCoefficients(P, I, D, F);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void loop() {
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        // Retain original legacy A behavior: heading reset and momentary blocker.
        if (gamepad1.a) drive.resetHeading();
        butterfly.update(forward, strafe, turn, gamepad1.right_bumper);
        drive.driveFieldRelative(forward, strafe, turn);

        mechanisms.updateTurretStep(gamepad1.right_bumper, gamepad1.left_bumper);
        mechanisms.updateFeed(gamepad1.left_trigger, gamepad1.x);
        mechanisms.updateBlockerMomentary(gamepad1.a);

        double targetTicks = TARGET_RPM * RobotHardwareNames.SHOOTER_TICKS_PER_REV / 60.0;
        shooterMotor.setVelocity(gamepad1.right_trigger > 0.2 ? targetTicks : 0.0);
        double actualRpm = shooterMotor.getVelocity() * 60.0 /
                RobotHardwareNames.SHOOTER_TICKS_PER_REV;

        telemetry.addData("Target RPM", TARGET_RPM);
        telemetry.addData("Actual RPM", "%.1f", actualRpm);
        telemetry.addData("RPM Error", "%.1f", Math.abs(TARGET_RPM) - actualRpm);
        telemetry.addData("P", P);
        telemetry.addData("I", I);
        telemetry.addData("D", D);
        telemetry.addData("F", F);
        telemetry.update();
    }

    @Override
    public void stop() {
        if (shooterMotor != null) shooterMotor.setPower(0.0);
        if (mechanisms != null) mechanisms.stop();
        if (drive != null) drive.stop();
    }
}
