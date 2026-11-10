package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.subsystems.ButterflyController;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "TeleOp", group = "Competition")
public class CompetitionTeleOp extends OpMode {
    Shooter shooter;
    ButterflyController butterflyController;
    IMU imu;
    Follower follower;
    @Override
    public void init() {
        shooter = new Shooter(hardwareMap);
        butterflyController = new ButterflyController(hardwareMap);
        follower = Constants.create(hardwareMap);

    }

    @Override
    public void init_loop() {
        super.init_loop();
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {
        if (gamepad1.left_bumper)
            follower.setPose(new Pose(0, 0, 0));

        butterflyController.setDown();

        DrivePowers powers = ManualDrive.fieldCentric(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                follower.pose().heading()
        );
        ManualDrive.driveOrHold(follower, powers);
        follower.update();
        Pose robotPose = follower.pose(); // returns a Pose object
        telemetry.addData("Robot X", robotPose.x());
        telemetry.addData("Robot Y", robotPose.y());
        telemetry.addData("Robot Heading", Math.toDegrees(robotPose.heading()));
    }

    @Override
    public void stop() {
        super.stop();
    }
}
