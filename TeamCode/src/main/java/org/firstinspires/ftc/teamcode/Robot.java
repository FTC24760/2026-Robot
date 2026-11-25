package org.firstinspires.ftc.teamcode;

import static com.pedropathing.ivy.Scheduler.schedule;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.DriveCommands;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands;
import org.firstinspires.ftc.teamcode.commands.ShotCommands;
import org.firstinspires.ftc.teamcode.controls.Bindings;
import org.firstinspires.ftc.teamcode.controls.Controls;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.utils.Alliance;
import org.firstinspires.ftc.teamcode.utils.HiveTracker;
import org.firstinspires.ftc.teamcode.utils.HubManager;
import org.firstinspires.ftc.teamcode.utils.ShotSolver;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * The whole robot, run with Ivy commands (Pedro Pathing's command library). An OpMode
 * only has to forward its lifecycle:
 *
 *   init()  -> robot = new Robot(this);   // resets the Ivy Scheduler, builds commands
 *   start() -> robot.start();             // enables the shooter, starts drive and auto aim
 *   loop()  -> robot.update();
 *   stop()  -> robot.stop();
 *
 * Commands decide what each mechanism should do (they set targets and flags on the
 * subsystems); the per-loop code here reads sensors and writes hardware. Each update()
 * runs, in order:
 *   1. clear the hubs' bulk cache, so this loop's sensor reads are fresh
 *   2. updateLoopTime(): set nowNs, loopSeconds and voltageScale
 *   3. controls.update() and bindings.update(): button presses schedule/cancel commands
 *   4. shotSolver.update(): aim targets for autoAim
 *   5. Scheduler.execute(): run every scheduled command once
 *   6. write hardware: follower (drive + odometry), shooter velocity loop, intake
 *   7. telemetry from every subsystem in the subsystems list
 *
 * Commands (in commands/), all built once here:
 *   drive      field-centric drive; scheduled at start, runs all match
 *   autoAim /  turret + wheel speed; one of them always runs. Starts on autoAim; the
 *   manualAim  operator's right stick button swaps them
 *   shoot      blocker + feed sequence while either right trigger is held
 *   intake     roller + transfer while either left trigger is held
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

    Controls controls;
    Bindings bindings = new Bindings();

    final Command drive;
    final Command autoAim;
    final Command manualAim;
    final Command shoot;
    final Command intakeCommand;

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

        controls = new Controls(opMode);

        // The Scheduler is static: clear anything left over from the previous OpMode.
        Scheduler.reset();

        drive = DriveCommands.fieldCentric(drivetrain, controls);
        autoAim = ShotCommands.autoAim(this);
        manualAim = ShotCommands.manualAim(this, controls::manualTurret);
        shoot = ShotCommands.shoot(intake, shooter);
        intakeCommand = IntakeCommands.intake(intake);

        // Button map (which physical buttons these are is in Controls).
        bindings.whileHeld(controls::shoot, shoot);
        bindings.whileHeld(controls::intake, intakeCommand);
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
        controls.update();
        bindings.update();
        shotSolver.update();

        Scheduler.execute();

        // Hardware writes, every loop whatever is scheduled. The follower also updates
        // odometry, and the shooter runs its velocity loop off this loop's reading.
        drivetrain.update();
        shooter.update(nowNs);
        intake.update();

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
