
package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.ButterflyController;

@TeleOp(name = "Jerry Rigged Teleop", group = "Robot")
public class JerryRiggedTeleOp extends OpMode {

    // variables
    double intakePower = 0.46;
    double transferPower = 0.5;

    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backLeftDrive;
    private DcMotor backRightDrive;
    private DcMotor intakeMotor;
    private DcMotor transferMotor;

    private IMU imu;
    private ButterflyController butterfly;

    Servo leftTurretServo;
    Servo rightTurretServo;

    double leftTurretServoPosition = 0.5;
    double rightTurretServoPosition = 0.5;



    // SHOOTER
    private DcMotorEx shooterMotor;

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
        frontLeftDrive = hardwareMap.get(DcMotor.class, "lf");
        frontRightDrive = hardwareMap.get(DcMotor.class, "rf");
        backLeftDrive = hardwareMap.get(DcMotor.class, "lb");
        backRightDrive = hardwareMap.get(DcMotor.class, "rb");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        transferMotor = hardwareMap.get(DcMotor.class, "transfer");
        leftTurretServo = hardwareMap.get(Servo.class, "leftturret");
        rightTurretServo = hardwareMap.get(Servo.class, "rightturret");

        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        // shooterMotor.setDirection(DcMotor.Direction.REVERSE);

        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set velocity PIDF coefficients
        shooterMotor.setVelocityPIDFCoefficients(P, I, D, F);

        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot orientationOnRobot =
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                );

        imu.initialize(new IMU.Parameters(orientationOnRobot));

        butterfly = new ButterflyController(hardwareMap);
    }

    @Override
    public void loop() {
        leftTurretServo.setPosition(leftTurretServoPosition);
        rightTurretServo.setPosition(rightTurretServoPosition);
        
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        boolean intakeTransferOn = gamepad1.left_trigger > 0.2;

        if (gamepad1.a) {
            imu.resetYaw();
        }

        butterfly.update(
                forward,
                strafe,
                rotate,
                gamepad1.right_bumper
        );

        driveFieldRelative(forward, strafe, rotate);

        if (gamepad1.right_trigger > 0.2) {
            if (leftTurretServoPosition < 1.0) {
                leftTurretServoPosition += 0.005;
            }
        } else if (gamepad1.left_trigger > 0.2) {
            if (leftTurretServoPosition > 0.0) {
                leftTurretServoPosition -= 0.005;
            }
        }

        if (gamepad1.right_bumper) {
            if (rightTurretServoPosition < 1.0) {
                rightTurretServoPosition += 0.005;
            }
        } else if (gamepad1.left_bumper) {
            if (rightTurretServoPosition > 0.0) {
                rightTurretServoPosition -= 0.005;
            }
        }

        if (intakeTransferOn) {
            intakeMotor.setPower(intakePower);
            transferMotor.setPower(transferPower);
        } else {
            intakeMotor.setPower(0);
            transferMotor.setPower(0);
        }


        // SHOOTER
        double targetTicksPerSecond = TARGET_RPM * TICKS_PER_REV / 60.0;

        // Hold right trigger to run shooter
        if (gamepad1.right_trigger > 0.2) {
            shooterMotor.setVelocity(targetTicksPerSecond);
        } else {
            shooterMotor.setVelocity(0);
        }

        // Measure current shooter speed
        double actualTicksPerSecond = shooterMotor.getVelocity();
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

    private void driveFieldRelative(double forward, double right, double rotate) {
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        theta = AngleUnit.normalizeRadians(
                theta - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)
        );

        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        drive(newForward, newRight, rotate);
    }

    private void drive(double forward, double right, double rotate) {
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double maxPower = Math.max(1.0,
                Math.max(
                        Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                        Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))
                )
        );

        frontLeftDrive.setPower(frontLeftPower / maxPower);
        frontRightDrive.setPower(frontRightPower / maxPower);
        backLeftDrive.setPower(backLeftPower / maxPower);
        backRightDrive.setPower(backRightPower / maxPower);
    }
}
