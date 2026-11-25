package org.firstinspires.ftc.teamcode.utils;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

/**
 * Aims the turret, sets the flywheel speed, and runs the shot sequence. Robot sets
 * automaticShot, shootPressed and the manual turret input, then calls update() once
 * per loop. Robot.runIntake() reads {@link #state} to open the blocker and feed.
 *
 * Aim modes (automaticShot):
 *   true   turret and wheel speed come from ShotSolver, with angular-velocity feedforward
 *   false  turret follows the operator's stick; wheel runs at ShooterConstants.MANUAL_RPM
 */
public class ShotController {
    public enum State {
        /** Blocker closed, not feeding. Waiting for shootPressed and the wheel at speed. */
        IDLE,
        /** Blocker opening; feeding starts after IntakeConstants.BLOCKER_OPEN_TIME_MS. */
        BLOCKER_OPEN,
        /** Blocker open and the transfer feeding balls into the shooter. */
        SHOOTING
    }
    public State state = State.IDLE;
    // Set every loop from the shoot button. While true the shot sequence runs:
    // IDLE until the wheel is at speed -> BLOCKER_OPEN for BLOCKER_OPEN_TIME_MS -> SHOOTING.
    // Going false at any point drops straight back to IDLE.
    public boolean shootPressed;
    // System.nanoTime() when the state became BLOCKER_OPEN.
    long blockerOpenedNs;
    // True: aim from ShotSolver. False: manual turret and MANUAL_RPM. Starts true.
    public boolean automaticShot;
    Shooter shooter;
    Turret turret;
    ShotSolver shotSolver;
    // Manual turret stick input, -1 to 1; positive turns left.
    double manualTurretInput;
    Robot robot;
    public ShotController(Robot robot) {
        this.robot = robot;
        shooter = robot.shooter;
        turret = robot.turret;
        shotSolver = robot.shotSolver;
        automaticShot = true;
    }

    /** Run the solver, turret, shooter and shot sequence for this loop. */
    public void update() {
        shotSolver.update();
        runTurret(robot.loopSeconds);
        runShooter();
        runState();
    }

    // Advance the shot sequence one step. Called after runShooter() so the speed check
    // uses this loop's wheel reading.
    private void runState() {
        if (!shootPressed) {
            state = State.IDLE;
            return;
        }

        switch (state) {
            case IDLE:
                if (shooter.isAtTargetRpm()) {
                    state = State.BLOCKER_OPEN;
                    blockerOpenedNs = robot.nowNs;
                }
                break;
            case BLOCKER_OPEN:
                if (robot.nowNs - blockerOpenedNs >= MathUtil.msToNs(IntakeConstants.BLOCKER_OPEN_TIME_MS)) {
                    state = State.SHOOTING;
                }
                break;
            case SHOOTING:
                break;
        }
    }

    /** Manual turret speed for this loop, -1 to 1 of TurretConstants.MANUAL_RATE; positive turns left. */
    public void manualMoveTurret(double turretRotate) {
        manualTurretInput = turretRotate;
    }

    private void runTurret(double loopSeconds) {
        if (automaticShot) {
            turret.setFeedforward(shotSolver.getAngularVelocity());
            turret.pointAt(shotSolver.turretAngle, loopSeconds);
        } else {
            turret.setFeedforward(0);
            turret.move(manualTurretInput, loopSeconds);
        }
    }

    private void runShooter() {
        if (automaticShot) {
            shooter.shootRpmOverride = shotSolver.shooterTargetRPM;
        }
        else {
            shooter.shootRpmOverride = ShooterConstants.MANUAL_RPM;
        }
        shooter.update(robot.nowNs);
    }

    // Not called yet: Robot only stops and reports the subsystems in its list.
    public void stop() {

    }

    public void addTelemetry(Telemetry telemetry) {

    }
}
