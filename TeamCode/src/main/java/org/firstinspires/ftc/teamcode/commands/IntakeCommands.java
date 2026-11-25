package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

/** Ivy commands for the intake roller. Shooting (blocker and feeding) is in ShotCommands. */
public final class IntakeCommands {
    private IntakeCommands() {
    }

    /** Run the roller and transfer until cancelled, then turn them off. */
    public static Command intake(Intake intake) {
        return Command.build()
                .setStart(() -> intake.intakeOn = true)
                .setEnd(endCondition -> intake.intakeOn = false)
                .requiring(intake.roller);
    }
}
