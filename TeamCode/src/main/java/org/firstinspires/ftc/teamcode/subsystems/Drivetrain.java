package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.DriveConstants;

/** Mecanum drive with IMU-based field-centric control. */
public class Drivetrain implements Subsystem {
    private final DcMotor frontLeft;
    private final DcMotor frontRight;
    private final DcMotor backLeft;
    private final DcMotor backRight;
    private final IMU imu;
    private double lastHeadingRadians;

    public Drivetrain(HardwareMap hardwareMap) {
        frontLeft = hardwareMap.get(DcMotor.class, DriveConstants.FRONT_LEFT_NAME);
        frontRight = hardwareMap.get(DcMotor.class, DriveConstants.FRONT_RIGHT_NAME);
        backLeft = hardwareMap.get(DcMotor.class, DriveConstants.BACK_LEFT_NAME);
        backRight = hardwareMap.get(DcMotor.class, DriveConstants.BACK_RIGHT_NAME);

        frontLeft.setDirection(DriveConstants.LEFT_DIRECTION);
        backLeft.setDirection(DriveConstants.LEFT_DIRECTION);
        frontRight.setDirection(DriveConstants.RIGHT_DIRECTION);
        backRight.setDirection(DriveConstants.RIGHT_DIRECTION);

        for (DcMotor motor : new DcMotor[]{frontLeft, frontRight, backLeft, backRight}) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        imu = hardwareMap.get(IMU.class, DriveConstants.IMU_NAME);
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                DriveConstants.IMU_LOGO_DIRECTION,
                DriveConstants.IMU_USB_DIRECTION
        )));
    }

    /** Drive relative to the field: "forward" is away from the driver regardless of heading. */
    public void driveFieldCentric(double forward, double strafe, double rotate) {
        double theta = Math.atan2(forward, strafe);
        double r = Math.hypot(strafe, forward);

        theta = AngleUnit.normalizeRadians(theta - getHeadingRadians());

        driveRobotCentric(r * Math.sin(theta), r * Math.cos(theta), rotate);
    }

    /** Drive relative to the robot's own front. */
    public void driveRobotCentric(double forward, double strafe, double rotate) {
        double frontLeftPower = forward + strafe + rotate;
        double frontRightPower = forward - strafe - rotate;
        double backRightPower = forward + strafe - rotate;
        double backLeftPower = forward - strafe + rotate;

        // Scale everything down together if any wheel would exceed full power.
        double maxPower = Math.max(1.0, Math.max(
                Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))
        ));

        frontLeft.setPower(frontLeftPower / maxPower);
        frontRight.setPower(frontRightPower / maxPower);
        backLeft.setPower(backLeftPower / maxPower);
        backRight.setPower(backRightPower / maxPower);
    }

    /** Make the robot's current heading the new field "forward". */
    public void resetYaw() {
        imu.resetYaw();
    }

    public double getHeadingRadians() {
        lastHeadingRadians = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        return lastHeadingRadians;
    }

    @Override
    public void stop() {
        driveRobotCentric(0, 0, 0);
    }

    @Override
    public void addTelemetry(Telemetry telemetry) {
        // Reuses the last reading: a fresh IMU read is slow I2C and would cost loop time.
        telemetry.addData("Heading deg", "%.1f", Math.toDegrees(lastHeadingRadians));
    }
}
