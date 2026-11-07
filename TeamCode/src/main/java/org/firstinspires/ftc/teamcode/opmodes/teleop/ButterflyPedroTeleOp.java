/*
Author: OJB
Date: 2026-07-23
*/
package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ButterflyController;

@Configurable
@TeleOp(
        name = "Butterfly Pedro TeleOp",
        group = "Competition"
)
public class ButterflyPedroTeleOp extends OpMode {

    public static boolean ROBOT_CENTRIC = true;
    public static boolean USE_BRAKE_MODE = true;

    private Follower follower;
    private ButterflyController butterflyController;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        butterflyController = new ButterflyController(hardwareMap);

        telemetry.addLine("Butterfly Pedro TeleOp initialized");
        telemetry.addData("Left servo", ButterflyController.LEFT_SERVO_NAME);
        telemetry.addData("Right servo", ButterflyController.RIGHT_SERVO_NAME);
        telemetry.update();
    }

    @Override
    public void start() {
        //follower.startTeleopDrive(USE_BRAKE_MODE);
    }

    @Override
    public void loop() {
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        butterflyController.update(
                forward,
                strafe,
                turn,
                gamepad1.a
        );

        follower.manual(forward, strafe, turn);
        follower.update();

        telemetry.addData(
                "Butterflies",
                butterflyController.getState()
        );
        telemetry.addData(
                "A manual invert",
                gamepad1.a
        );
        telemetry.addData(
                "Forward",
                "%.2f",
                forward
        );
        telemetry.addData(
                "Strafe",
                "%.2f",
                strafe
        );
        telemetry.addData(
                "Turn",
                "%.2f",
                turn
        );
        telemetry.update();
    }

    @Override
    public void stop() {
        butterflyController.setUp();
    }
}