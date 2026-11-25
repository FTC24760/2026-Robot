package org.firstinspires.ftc.teamcode.commands;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.subsystems.FlowerArm;

/** Ivy commands for the flower arm. */
public final class FlowerCommands {
    private FlowerCommands() {
    }

    /** Raise the arm if it's down, lower it if it's up. */
    public static Command toggle(FlowerArm flowerArm) {
        return instant(flowerArm::toggle).requiring(flowerArm);
    }
}
