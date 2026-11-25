package org.firstinspires.ftc.teamcode.commands;

import static com.pedropathing.ivy.commands.Commands.infinite;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.InterruptedBehavior;

import org.firstinspires.ftc.teamcode.controls.Controls;
import org.firstinspires.ftc.teamcode.subsystems.Butterfly;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

/**
 * Ivy commands for the drivetrain. For autonomous paths use Ivy's Pedro commands
 * (import static com.pedropathing.ivy.pedro.PedroCommands.*): follow(follower, path),
 * hold(follower). Robot.update() runs the follower every loop, so those just work.
 */
public final class DriveCommands {
    private DriveCommands() {
    }

    /**
     * Field-centric mecanum drive from gamepad 1. Runs until cancelled. When another
     * drivetrain command (tank) takes over it is suspended, not ended, and Ivy resumes it
     * once that command finishes.
     */
    public static Command fieldCentric(Drivetrain drivetrain, Controls controls) {
        return infinite(() -> drivetrain.driveFieldCentric(
                controls.forward(),
                controls.strafe(),
                controls.rotate()
        )).requiring(drivetrain)
                .setInterruptedBehavior(InterruptedBehavior.SUSPEND);
    }

    /**
     * Butterfly pods down and field-oriented tank drive from the left stick, until
     * cancelled; then the pods go back up. The driver's right stick Y picks the
     * direction: up faces the left stick, down faces away from it.
     */
    public static Command tank(Drivetrain drivetrain, Butterfly butterfly, Controls controls) {
        return Command.build()
                .setStart(() -> {
                    butterfly.setDown();
                    drivetrain.resetTankHeading();
                })
                .setExecute(() -> drivetrain.driveTank(
                        controls.forward(),
                        controls.strafe(),
                        controls.tankDirection() < 0
                ))
                .setEnd(endCondition -> butterfly.setUp())
                .requiring(drivetrain, butterfly);
    }
}
