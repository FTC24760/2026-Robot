package org.firstinspires.ftc.teamcode.commands;

import static com.pedropathing.ivy.commands.Commands.infinite;

import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.controls.Controls;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

/**
 * Ivy commands for the drivetrain. For autonomous paths use Ivy's Pedro commands
 * (import static com.pedropathing.ivy.pedro.PedroCommands.*): follow(follower, path),
 * hold(follower). Robot.update() runs the follower every loop, so those just work.
 */
public final class DriveCommands {
    private DriveCommands() {
    }

    /** Field-centric drive from gamepad 1. Runs until interrupted. */
    public static Command fieldCentric(Drivetrain drivetrain, Controls controls) {
        return infinite(() -> drivetrain.driveFieldCentric(
                controls.forward(),
                controls.strafe(),
                controls.rotate()
        )).requiring(drivetrain);
    }
}
