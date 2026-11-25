package org.firstinspires.ftc.teamcode.utils;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

public class ShotController {
    public enum State {
        IDLE,
        BLOCKER_OPEN,
        SHOOTING
    }
    public State state = State.IDLE;
    // Set every loop from the shoot button. While true the shot sequence runs:
    // IDLE until the wheel is at speed -> BLOCKER_OPEN for BLOCKER_OPEN_TIME_MS -> SHOOTING.
    // Going false at any point drops straight back to IDLE.
    public boolean shootPressed;
    // System.nanoTime() when the state became BLOCKER_OPEN.
    long blockerOpenedNs;
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

    public void update() {
        shotSolver.update();
        runTurret(robot.loopSeconds);
        runShooter();
        runState();
    }

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

    public void stop() {

    }

    public void addTelemetry(Telemetry telemetry) {

    }
}
