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
 *   start() -> robot.start();     // enables the shooter
 *   loop()  -> robot.update();
 *   stop()  -> robot.stop();
 *
 * Each update() runs, in order:
 *   1. clear the hubs' bulk cache, so this loop's sensor reads are fresh
 *   2. updateLoopTime(): set nowNs, loopSeconds and voltageScale
 *   3. controls.update(): work out this loop's button presses
 *   4. runDrive(): field-centric drive from gamepad 1, plus odometry
 *   5. runShotController(): shot mode, manual turret, shoot button -> ShotController
 *   6. runIntake(): intake roller, transfer and blocker from the trigger and shot state
 *   7. runFlowerArm(): Y toggles the flower arm up / down
 *   8. telemetry from every subsystem in the subsystems list
 *
 * Other classes read the loop timing from here (robot.nowNs, robot.loopSeconds) and
 * the battery compensation from Robot.voltageScale, rather than timing things themselves.
 * Hardware details live in the subsystems, tunable numbers in constants/, and button
 * bindings in Controls.
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
    FlowerArm flowerArm;
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
        flowerArm = new FlowerArm(hardwareMap);
        turret = new Turret(hardwareMap);
        shooter = new Shooter(hardwareMap);
        subsystems = new ArrayList<>(Arrays.asList(drivetrain, butterfly, intake, flowerArm, turret, shooter));

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
        runFlowerArm();

        updateTelemetry();
    }

    // Called once at the top of each loop so every class sees the same time and voltage.
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

    // Right stick button toggles auto/manual aim. In manual the operator's right stick
    // turns the turret. Either right trigger runs the shot sequence.
    private void runShotController() {
        if (controls.shotModeTogglePressed)
            shotController.automaticShot = !shotController.automaticShot;

        if (!shotController.automaticShot) {
            shotController.manualMoveTurret(controls.manualTurret());
        }
        shotController.shootPressed = controls.shoot();
        shotController.update();
    }

    // Either left trigger runs the intake roller and transfer. The transfer also runs while
    // the ShotController is SHOOTING, and the blocker opens for BLOCKER_OPEN and SHOOTING.
    private void runIntake() {
        intake.intakeOn = controls.intake();
        intake.transferOn = controls.intake()
                || shotController.state == ShotController.State.SHOOTING;
        intake.blockerOpen = shotController.state == ShotController.State.BLOCKER_OPEN ||
                             shotController.state == ShotController.State.SHOOTING;
        intake.update();
    }

    // Y on either gamepad toggles the flower arm. It starts up.
    private void runFlowerArm() {
        if (controls.flowerTogglePressed)
            flowerArm.down = !flowerArm.down;
        flowerArm.update();
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