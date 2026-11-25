package org.firstinspires.ftc.teamcode.commands;

import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.onInterrupt;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.utils.ShotSolver;

import java.util.function.DoubleSupplier;

/**
 * Ivy commands for aiming and shooting. Replaces the old ShotController state machine.
 *
 * Aiming: exactly one of autoAim() / manualAim() runs at a time. Both require the turret
 * and the shooter, so scheduling one interrupts the other. They only set targets;
 * Robot.update() runs Shooter.update() after the Scheduler so the wheel's velocity loop
 * runs every loop whatever is scheduled.
 *
 * Shooting: shoot() is the old IDLE -> BLOCKER_OPEN -> SHOOTING sequence. It runs while
 * the shoot button is held; cancelling it (button released) closes the blocker and stops
 * feeding.
 */
public final class ShotCommands {
    private ShotCommands() {
    }

    /**
     * Turret bearing and wheel speed from ShotSolver, with angular-velocity feedforward on
     * the turret. Runs until interrupted. Robot updates the solver each loop before the
     * Scheduler runs.
     */
    public static Command autoAim(Robot robot) {
        ShotSolver solver = robot.shotSolver;
        return infinite(() -> {
            robot.turret.setFeedforward(solver.getAngularVelocity());
            robot.turret.pointAt(solver.turretAngle, robot.loopSeconds);
            robot.shooter.shootRpmOverride = solver.shooterTargetRPM;
        }).requiring(robot.turret, robot.shooter);
    }

    /**
     * Turret follows the stick; wheel runs at ShooterConstants.MANUAL_RPM. Runs until
     * interrupted.
     *
     * @param turretInput -1 to 1 of TurretConstants.MANUAL_RATE; positive turns left
     */
    public static Command manualAim(Robot robot, DoubleSupplier turretInput) {
        return infinite(() -> {
            robot.turret.setFeedforward(0);
            robot.turret.move(turretInput.getAsDouble(), robot.loopSeconds);
            robot.shooter.shootRpmOverride = ShooterConstants.MANUAL_RPM;
        }).requiring(robot.turret, robot.shooter);
    }

    /**
     * Wait for the wheel to reach speed, open the blocker, wait BLOCKER_OPEN_TIME_MS so it
     * is out of the way, then feed until cancelled. Requires only the intake's feeder, so
     * the intake roller can keep running. Reads the shooter's speed without requiring it,
     * so the aim command keeps control of the wheel.
     */
    public static Command shoot(Intake intake, Shooter shooter) {
        Command sequence = sequential(
                waitUntil(shooter::isAtTargetRpm),
                instant(() -> intake.blockerOpen = true),
                waitMs(IntakeConstants.BLOCKER_OPEN_TIME_MS),
                infinite(() -> intake.feeding = true)
        );
        // Don't call setEnd() on the sequential itself: that would replace the group's own
        // end logic. Racing it against onInterrupt() gives cleanup when it's cancelled.
        return sequence.raceWith(
                onInterrupt(() -> {
                    intake.feeding = false;
                    intake.blockerOpen = false;
                }).requiring(intake.feeder)
        );
    }
}
