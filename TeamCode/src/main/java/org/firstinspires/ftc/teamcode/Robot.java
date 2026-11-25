package org.firstinspires.ftc.teamcode;

import static com.pedropathing.ivy.Scheduler.schedule;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.DriveCommands;
import org.firstinspires.ftc.teamcode.commands.FlowerCommands;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands;
import org.firstinspires.ftc.teamcode.commands.ShotCommands;
import org.firstinspires.ftc.teamcode.constants.AutoConstants;
import org.firstinspires.ftc.teamcode.constants.FieldConstants;
import org.firstinspires.ftc.teamcode.controls.Bindings;
import org.firstinspires.ftc.teamcode.controls.Controls;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.utils.Alliance;
import org.firstinspires.ftc.teamcode.utils.FieldPoses;
import org.firstinspires.ftc.teamcode.utils.HiveTracker;
import org.firstinspires.ftc.teamcode.utils.HubManager;
import org.firstinspires.ftc.teamcode.utils.PoseStorage;
import org.firstinspires.ftc.teamcode.utils.ShotSolver;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * The whole robot, run with Ivy commands (Pedro Pathing's command library). An OpMode
 * only has to forward its lifecycle:
 *
 *   init()  -> robot = new Robot(this, alliance);   // resets the Ivy Scheduler, sets the
 *                                                   // field pose, builds commands
 *   start() -> robot.start();             // enables the shooter, starts drive and auto aim
 *   loop()  -> robot.update();
 *   stop()  -> robot.stop();
 *
 * Commands decide what each mechanism should do (they set targets and flags on the
 * subsystems); the per-loop code here reads sensors and writes hardware. Each update()
 * runs, in order:
 *   1. clear the hubs' bulk cache, so this loop's sensor reads are fresh
 *   2. updateLoopTime(): set nowNs, loopSeconds and voltageScale
 *   3. bindings.update(): button presses schedule/cancel commands
 *   4. shotSolver.update(): CELL, turret bearing, RPM and hood from the pose
 *   5. Scheduler.execute(): run every scheduled command once
 *   6. write hardware: follower (drive + odometry), shooter velocity loop, intake,
 *      flower arm (re-sent so live Panels edits apply)
 *   7. telemetry from every subsystem in the subsystems list
 *
 * Commands (in commands/), all built once here:
 *   drive      field-centric mecanum drive; scheduled at start, runs all match
 *   tank       butterfly pods down + field-oriented tank drive while the driver's right
 *              stick Y is pushed; drive is suspended meanwhile and resumes after
 *   autoAim /  turret + wheel speed; one of them always runs. Starts on autoAim; the
 *   manualAim  operator's right stick button swaps them
 *   shoot      blocker + feed sequence while either right trigger is held
 *   intake     roller + transfer while either left trigger is held
 *   flower     toggles the flower arm on Y (either gamepad)
 * Operator X / B reset odometry to FieldConstants.RESET_POSE_1 / 2.
 *
 * Odometry is in the field frame described in AutoConstants. At init the pose comes
 * from PoseStorage (left there by the auto) or, if there isn't one, the auto start pose.
 *
 * Vision is not built in TeleOp: it is only for detecting which CELL is up in auto.
 *
 * Other classes read the loop timing from here (robot.nowNs, robot.loopSeconds) and
 * the battery compensation from Robot.voltageScale, rather than timing things themselves.
 * Hardware details live in the subsystems, tunable numbers in constants/, and button
 * bindings in the constructor below (buttons are read in Controls).
 */
public class Robot {
    OpMode opmode;
    HardwareMap hardwareMap;
    Telemetry telemetry;
    // Fixed for the OpMode; picks our HIVE and which way field-centric "forward" points.
    public final Alliance alliance;
    HubManager hubManager;
    Drivetrain drivetrain;
    Butterfly butterfly;
    Intake intake;
    public Turret turret;
    public Shooter shooter;
    public Hood hood;
    FlowerArm flowerArm;
    public Follower follower;
    List<Subsystem> subsystems;

    HiveTracker hiveTracker;
    public ShotSolver shotSolver;

    Controls controls;
    Bindings bindings = new Bindings();

    final Command drive;
    final Command tank;
    final Command autoAim;
    final Command manualAim;
    final Command shoot;
    final Command intakeCommand;
    final Command flowerToggle;

    // System.nanoTime() at the start of the current loop, or 0 before the first loop.
    public long nowNs;
    // Seconds between the start of the previous loop and this one; 0 on the first loop.
    public double loopSeconds;
    // Nominal voltage / battery voltage, from HubManager; 1.0 until the first loop.
    // Static so subsystems can read Robot.voltageScale without a Robot reference.
    public static double voltageScale = 1.0;

    public Robot(OpMode opMode, Alliance alliance) {
        this.alliance = alliance;
        hardwareMap = opMode.hardwareMap;
        telemetry = opMode.telemetry;
        voltageScale = 1.0;  // don't carry over the last OpMode's reading

        hubManager = new HubManager(hardwareMap);
        follower = Constants.create(hardwareMap);

        drivetrain = new Drivetrain(hardwareMap, follower);
        drivetrain.driverHeadingOffset = FieldPoses.driverHeadingOffset(alliance);
        if (PoseStorage.pose != null) {
            drivetrain.setPose(PoseStorage.pose);
            PoseStorage.pose = null;  // only use an auto's pose once
        } else {
            drivetrain.setPose(FieldPoses.forAlliance(alliance,
                    AutoConstants.START_X, AutoConstants.START_Y, AutoConstants.START_H));
        }
        butterfly = new Butterfly(hardwareMap);
        intake = new Intake(hardwareMap);
        turret = new Turret(hardwareMap);
        shooter = new Shooter(hardwareMap);
        hood = new Hood(hardwareMap);
        flowerArm = new FlowerArm(hardwareMap);
        subsystems = new ArrayList<>(Arrays.asList(
                drivetrain, butterfly, intake, turret, shooter, hood, flowerArm));

        hiveTracker = new HiveTracker();
        shotSolver = new ShotSolver(this);

        controls = new Controls(opMode);

        // The Scheduler is static: clear anything left over from the previous OpMode.
        Scheduler.reset();

        drive = DriveCommands.fieldCentric(drivetrain, controls);
        tank = DriveCommands.tank(drivetrain, butterfly, controls);
        autoAim = ShotCommands.autoAim(this);
        manualAim = ShotCommands.manualAim(this, controls::manualTurret);
        shoot = ShotCommands.shoot(intake, shooter);
        intakeCommand = IntakeCommands.intake(intake);
        flowerToggle = FlowerCommands.toggle(flowerArm);

        // Button map (which physical buttons these are is in Controls).
        bindings.whileHeld(controls::shoot, shoot);
        bindings.whileHeld(controls::intake, intakeCommand);
        bindings.whileHeld(controls::butterflyEngage, tank);
        bindings.onPress(controls::flowerToggle, () -> schedule(flowerToggle));
        bindings.onPress(controls::resetPose1, () -> drivetrain.setPose(FieldPoses.forAlliance(
                alliance, FieldConstants.RESET_POSE_1_X, FieldConstants.RESET_POSE_1_Y,
                FieldConstants.RESET_POSE_1_H)));
        bindings.onPress(controls::resetPose2, () -> drivetrain.setPose(FieldPoses.forAlliance(
                alliance, FieldConstants.RESET_POSE_2_X, FieldConstants.RESET_POSE_2_Y,
                FieldConstants.RESET_POSE_2_H)));
        bindings.onPress(controls::shotModeToggle,
                () -> schedule(autoAim.isScheduled() ? manualAim : autoAim));
    }

    public void start() {
        shooter.state = Shooter.State.ENABLED;
        schedule(drive, autoAim);
    }

    public void update() {
        hubManager.clearBulkCache();
        updateLoopTime();
        bindings.update();
        shotSolver.update();

        Scheduler.execute();

        // Hardware writes, every loop whatever is scheduled. The follower also updates
        // odometry, and the shooter runs its velocity loop off this loop's reading.
        drivetrain.update();
        shooter.update(nowNs);
        intake.update();
        flowerArm.refresh();

        updateTelemetry();
    }

    // Called once at the top of each loop so every class sees the same time and voltage.
    private void updateLoopTime() {
        long now = System.nanoTime();
        loopSeconds = nowNs == 0 ? 0 : (now - nowNs) / 1e9;
        nowNs = now;
        voltageScale = hubManager.getVoltageScale(nowNs);
    }

    void updateTelemetry() {
        telemetry.addData("Aim", autoAim.isScheduled() ? "auto"
                : manualAim.isScheduled() ? "manual" : "off");
        telemetry.addData("Shooting", shoot.isScheduled() ? "yes" : "no");
        telemetry.addData("Drive", tank.isScheduled() ? "tank" : "mecanum");
        shotSolver.addTelemetry(telemetry);
        for (Subsystem subsystem : subsystems) {
            subsystem.addTelemetry(telemetry);
        }
        telemetry.update();
    }

    public void stop() {
        // Drop every command without running its end(); stop() below makes each
        // mechanism safe anyway.
        Scheduler.reset();
        for (Subsystem subsystem : subsystems) {
            subsystem.stop();
        }
    }
}
