
package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.List;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.ButterflyController;

@Configurable
@TeleOp(name = "Jerry Rigged PIDF Teleop", group = "Robot")
public class JerryRiggedPidfTeleOp extends OpMode {

    // variables
    public static double intakePower = 0.6;
    public static double transferPower = 0.8;

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
    Servo blockerServo;

    double leftTurretServoPosition = 0.5;
    double rightTurretServoPosition = 0.5;
    double blockerOpenPosition = 0.032;
    double blockerClosedPosition = 0.0;



    // SHOOTER
    private DcMotorEx shooterMotor;

    // goBILDA 5000 Series (no gearbox)
    private static final double TICKS_PER_REV = 28.0;

    // Target shooter speed (RPM)
    public static double TARGET_RPM = 1525;

    // Software controller gains: power per tick/second (not the hub PIDF units).
    // Starting estimates only; tune on the robot: kF, kS, kP, kD, kI, kA.
    public static double kF = 29.0 / 32767.0;
    public static double kS = 0.0;
    public static double kP = 0.0001;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kA = 0.0;
    public static double MAX_INTEGRAL = 1000.0;
    public static double NOMINAL_VOLTAGE = 12.0;
    public static double MAX_ACCEL = 6000.0; // ticks/s^2
    public static double VELOCITY_ALPHA = 0.7;
    public static double KS_DEADBAND = 50.0; // ticks/s
    public static double TURRET_RATE = 0.25; // servo position per second
    private List<LynxModule> hubs;
    private VoltageSensor voltageSensor;
    private long lastVoltageReadNs;
    private long lastLoopNs;
    private double voltageScale = 1.0;
    private final SpeedController speedController = new SpeedController();

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
        blockerServo = hardwareMap.get(Servo.class, "blocker");

        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        transferMotor.setDirection(DcMotor.Direction.REVERSE);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        // shooterMotor.setDirection(DcMotor.Direction.REVERSE);

        // Power is controlled in software; avoid the hub velocity loop.
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        voltageSensor = hardwareMap.voltageSensor.iterator().hasNext()
                ? hardwareMap.voltageSensor.iterator().next() : null;

        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot orientationOnRobot =
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                );

        imu.initialize(new IMU.Parameters(orientationOnRobot));

        butterfly = new ButterflyController(hardwareMap);
    }

    @Override
    public void loop() {
        long now = System.nanoTime();
        double loopSeconds = lastLoopNs == 0 ? 0 : Math.min((now - lastLoopNs) * 1e-9, 0.1);
        lastLoopNs = now;
        for (LynxModule hub : hubs) hub.clearBulkCache();
        // Exactly one fresh velocity read per loop; reuse it for control and telemetry.
        double actualTicksPerSecond = shooterMotor.getVelocity();
        boolean shooterEnabled = gamepad1.right_trigger > 0.2;
        double targetTicksPerSecond = shooterEnabled ? TARGET_RPM * TICKS_PER_REV / 60.0 : 0.0;
        if (lastVoltageReadNs == 0 || now - lastVoltageReadNs >= 250_000_000L) {
            double voltage = voltageSensor == null ? 0.0 : voltageSensor.getVoltage();
            voltageScale = voltage > 0.1 ? NOMINAL_VOLTAGE / voltage : 1.0;
            lastVoltageReadNs = now;
        }
        double shooterPower;
        if (shooterEnabled) {
            shooterPower = speedController.calculate(targetTicksPerSecond, actualTicksPerSecond, voltageScale, now);
        } else {
            speedController.reset();
            shooterPower = 0.0;
        }
        shooterMotor.setPower(shooterPower);
        leftTurretServo.setPosition(leftTurretServoPosition);
        rightTurretServo.setPosition(rightTurretServoPosition);
        
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;


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

        if (gamepad1.right_bumper) {
            if (rightTurretServoPosition < 1.0) {
                rightTurretServoPosition = Math.min(1.0, rightTurretServoPosition + TURRET_RATE * loopSeconds);
                leftTurretServoPosition = Math.min(1.0, leftTurretServoPosition + TURRET_RATE * loopSeconds);
            }
        } else if (gamepad1.left_bumper) {
            if (rightTurretServoPosition > 0.0) {
                rightTurretServoPosition = Math.max(0.0, rightTurretServoPosition - TURRET_RATE * loopSeconds);
                leftTurretServoPosition = Math.max(0.0, leftTurretServoPosition - TURRET_RATE * loopSeconds);
            }
        }

        if (gamepad1.left_trigger > 0.2) {
            intakeMotor.setPower(intakePower);
            transferMotor.setPower(transferPower);
        }
        else if (gamepad1.x) {
            intakeMotor.setPower(0);
            transferMotor.setPower(transferPower);
        } else {
            intakeMotor.setPower(0);
            transferMotor.setPower(0);
        }

        if (gamepad1.a) {
            blockerServo.setPosition(blockerOpenPosition);
        } else {
            blockerServo.setPosition(blockerClosedPosition);
        }


        double actualRPM = actualTicksPerSecond * 60.0 / TICKS_PER_REV;
        telemetry.addData("Target RPM", targetTicksPerSecond * 60.0 / TICKS_PER_REV);
        telemetry.addData("Actual RPM", "%.1f", actualRPM);
        telemetry.addData("Profiled RPM", "%.1f", speedController.profiledTarget * 60.0 / TICKS_PER_REV);
        telemetry.addData("Shooter power", "%.3f", shooterPower);
        telemetry.addData("Loop ms", "%.1f", loopSeconds * 1000.0);
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
    @Override
    public void stop() {
        shooterMotor.setPower(0);
        speedController.reset();
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
    }

    // Based on the supplied velocity controller; all velocities are encoder ticks/s.
    private static class SpeedController {
        private double integral, filteredVel, profiledTarget;
        private long lastTimeNs = -1;

        double calculate(double target, double measured, double voltageScale, long now) {
            if (lastTimeNs < 0) {
                lastTimeNs = now;
                filteredVel = measured;
                profiledTarget = measured;
                return 0.0;
            }
            double dt = (now - lastTimeNs) * 1e-9;
            lastTimeNs = now;
            if (dt <= 0) return 0.0;
            double previousVel = filteredVel;
            double alpha = clamp(VELOCITY_ALPHA, 0, 1);
            filteredVel = alpha * filteredVel + (1 - alpha) * measured;
            double previousTarget = profiledTarget;
            double maxDelta = Math.max(0, MAX_ACCEL) * dt;
            profiledTarget += clamp(target - profiledTarget, -maxDelta, maxDelta);
            double accel = (profiledTarget - previousTarget) / dt;
            double error = profiledTarget - filteredVel;
            double derivative = -(filteredVel - previousVel) / dt;
            if (Math.abs(kP * error) < 1.0) {
                double limit = Math.abs(MAX_INTEGRAL);
                integral = clamp(integral + error * dt, -limit, limit);
            }
            double staticTerm = Math.abs(profiledTarget) > KS_DEADBAND
                    ? Math.copySign(kS, profiledTarget) : 0.0;
            return clamp((kF * profiledTarget + staticTerm + kA * accel
                    + kP * error + kI * integral + kD * derivative) * voltageScale, -1, 1);
        }

        void reset() {
            integral = filteredVel = profiledTarget = 0;
            lastTimeNs = -1;
        }
        private static double clamp(double value, double min, double max) {
            return Math.max(min, Math.min(max, value));
        }
    }

}

