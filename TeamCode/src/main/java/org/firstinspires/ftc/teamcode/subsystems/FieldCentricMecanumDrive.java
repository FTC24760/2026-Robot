package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.constants.RobotHardwareNames;

/** JerryRigged field-centric mecanum drive; preserves original axis conventions. */
public final class FieldCentricMecanumDrive {
    private final DcMotor frontLeft, frontRight, backLeft, backRight;
    private final IMU imu;

    public FieldCentricMecanumDrive(HardwareMap map) {
        frontLeft = map.get(DcMotor.class, RobotHardwareNames.FRONT_LEFT);
        frontRight = map.get(DcMotor.class, RobotHardwareNames.FRONT_RIGHT);
        backLeft = map.get(DcMotor.class, RobotHardwareNames.BACK_LEFT);
        backRight = map.get(DcMotor.class, RobotHardwareNames.BACK_RIGHT);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = map.get(IMU.class, RobotHardwareNames.IMU);
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP)));
    }

    public void resetHeading() { imu.resetYaw(); }

    public void driveFieldRelative(double forward, double right, double rotate) {
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);
        theta = AngleUnit.normalizeRadians(theta -
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        driveRobotRelative(r * Math.sin(theta), r * Math.cos(theta), rotate);
    }

    public void driveRobotRelative(double forward, double right, double rotate) {
        double fl = forward + right + rotate;
        double fr = forward - right - rotate;
        double br = forward + right - rotate;
        double bl = forward - right + rotate;
        double max = Math.max(1.0, Math.max(
                Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br))));
        frontLeft.setPower(fl / max);
        frontRight.setPower(fr / max);
        backLeft.setPower(bl / max);
        backRight.setPower(br / max);
    }

    public void stop() {
        frontLeft.setPower(0.0);
        frontRight.setPower(0.0);
        backLeft.setPower(0.0);
        backRight.setPower(0.0);
    }
}
