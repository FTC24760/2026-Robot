package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.controls.DriverControls;
import org.firstinspires.ftc.teamcode.controls.OperatorControls;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.utils.Alliance;
import org.firstinspires.ftc.teamcode.utils.HubManager;


import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * The whole robot. An OpMode only has to forward its lifecycle:
 *
 *   init()  -> robot = new Robot(this);
 *   start() -> robot.start();
 *   loop()  -> robot.update();
 *   stop()  -> robot.stop();
 *
 * Robot wires driver input (DriverControls, gamepad 1) and operator input
 * (OperatorControls, gamepad 2) to the subsystems. Hardware details live in the
 * subsystems, tunable numbers in constants/, and button bindings in the controls classes.
 *
 * An autonomous OpMode drives the same robot without a gamepad:
 *
 *   loop() -> robot.beginLoop();
 *             robot.drivetrain.update();      // follow the current path
 *             robot.shootSequence.update(robot.nowSeconds());
 *             robot.runShooter(true);         // spin up / hold speed
 *             robot.runTurret(0);             // auto-align if the HIVE is in view
 *             robot.runIntake(mode);          // feed, held back until the wheel is ready
 *             robot.endLoop();
 */
public class Robot {
    OpMode opmode;
    HardwareMap hardwareMap;
    Telemetry telemetry;
    Alliance alliance = Alliance.RED;
    HubManager hubManager;
    Drivetrain drivetrain;
    Butterfly butterfly;
    Intake intake;
    Turret turret;
    Shooter shooter;
    Follower follower;
    List<Subsystem> subsystems;

    DriverControls driverControls;
    OperatorControls operatorControls;

    public Robot(OpMode opMode) {
        hardwareMap = opMode.hardwareMap;
        telemetry = opMode.telemetry;

        hubManager = new HubManager(hardwareMap);
        follower = Constants.create(hardwareMap);

        drivetrain = new Drivetrain(hardwareMap, follower);
        butterfly = new Butterfly(hardwareMap);
        intake = new Intake(hardwareMap);
        turret = new Turret(hardwareMap);
        shooter = new Shooter(hardwareMap);
        subsystems = new ArrayList<>(Arrays.asList(drivetrain, butterfly, intake, turret, shooter));

        driverControls = new DriverControls(opMode);
        operatorControls = new OperatorControls(opMode);
    }

    public void start() {
    }
    public void update() {
        hubManager.clearBulkCache();
        Drive();
        UpdateTurret();

        updateTelemetry();
    }

    private void Drive() {
        double forward = driverControls.forward();
        double strafe = driverControls.strafe();
        double rotate = driverControls.rotate();

        drivetrain.driveFieldCentric(forward, strafe, rotate);
        drivetrain.update();
    }
    private void UpdateTurret() {
        turret.setFeedforward(follower.velocity().omega);
    }
    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
    }

    void updateTelemetry() {
        for (Subsystem subsystem : subsystems) {
            subsystem.addTelemetry(telemetry);
        }
        telemetry.update();
    }


    public void stop() {
        for (Subsystem subsystem : subsystems) {
            subsystem.stop();
        }
    }
}