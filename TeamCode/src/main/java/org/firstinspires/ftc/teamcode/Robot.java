package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.controls.Controls;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.utils.Alliance;
import org.firstinspires.ftc.teamcode.utils.HiveTracker;
import org.firstinspires.ftc.teamcode.utils.HubManager;
import org.firstinspires.ftc.teamcode.utils.ShotController;
import org.firstinspires.ftc.teamcode.utils.ShotSolver;

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
    public Alliance alliance = Alliance.RED;
    HubManager hubManager;
    Drivetrain drivetrain;
    Butterfly butterfly;
    Intake intake;
    public Turret turret;
    public Shooter shooter;
    public Follower follower;
    List<Subsystem> subsystems;

    HiveTracker hiveTracker;
    public ShotSolver shotSolver;
    ShotController shotController;

    Controls controls;

    public boolean tankMode = false;

    // System.nanoTime() at the start of the current loop, or 0 before the first loop.
    public long nowNs;
    // Seconds between the start of the previous loop and this one; 0 on the first loop.
    public double loopSeconds;
    // Nominal voltage / battery voltage, from HubManager; 1.0 until the first loop.
    // Static so subsystems can read Robot.voltageScale without a Robot reference.
    public static double voltageScale = 1.0;

    public Robot(OpMode opMode) {
        hardwareMap = opMode.hardwareMap;
        telemetry = opMode.telemetry;
        voltageScale = 1.0;  // don't carry over the last OpMode's reading

        hubManager = new HubManager(hardwareMap);
        follower = Constants.create(hardwareMap);

        drivetrain = new Drivetrain(hardwareMap, follower);
        butterfly = new Butterfly(hardwareMap);
        intake = new Intake(hardwareMap);
        turret = new Turret(hardwareMap);
        shooter = new Shooter(hardwareMap);
        subsystems = new ArrayList<>(Arrays.asList(drivetrain, butterfly, intake, turret, shooter));

        hiveTracker = new HiveTracker();
        shotSolver = new ShotSolver(this);
        shotController = new ShotController(this);

        controls = new Controls(opMode);
    }

    public void start() {
        shooter.state = Shooter.State.ENABLED;
    }
    public void update() {
        hubManager.clearBulkCache();
        updateLoopTime();
        controls.update();
        runDrive();
        runShotController();
        runIntake();

        updateTelemetry();
    }

    private void updateLoopTime() {
        long now = System.nanoTime();
        loopSeconds = nowNs == 0 ? 0 : (now - nowNs) / 1e9;
        nowNs = now;
        voltageScale = hubManager.getVoltageScale(nowNs);
    }

    private void runDrive() {
        double forward = controls.forward();
        double strafe = controls.strafe();
        double rotate = controls.rotate();

        drivetrain.driveFieldCentric(forward, strafe, rotate);
        drivetrain.update();
    }

    private void runShotController() {
        if (controls.shotModeTogglePressed)
            shotController.automaticShot = !shotController.automaticShot;

        if (!shotController.automaticShot) {
            shotController.manualMoveTurret(controls.manualTurret());
        }
        shotController.shootPressed = controls.shoot();
        shotController.update();
    }

    private void runIntake() {
        intake.intakeOn = controls.intake();
        intake.transferOn = controls.intake()
                || shotController.state == ShotController.State.SHOOTING;
        intake.blockerOpen = shotController.state == ShotController.State.BLOCKER_OPEN ||
                             shotController.state == ShotController.State.SHOOTING;
        intake.update();
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