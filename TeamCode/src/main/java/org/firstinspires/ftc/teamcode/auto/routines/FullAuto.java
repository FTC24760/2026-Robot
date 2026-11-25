package org.firstinspires.ftc.teamcode.auto.routines;

import static org.firstinspires.ftc.teamcode.auto.Steps.byUpSide;
import static org.firstinspires.ftc.teamcode.auto.Steps.collectFromFlower;
import static org.firstinspires.ftc.teamcode.auto.Steps.driveTo;
import static org.firstinspires.ftc.teamcode.auto.Steps.intake;
import static org.firstinspires.ftc.teamcode.auto.Steps.sequence;
import static org.firstinspires.ftc.teamcode.auto.Steps.shoot;
import static org.firstinspires.ftc.teamcode.auto.Steps.spinUp;
import static org.firstinspires.ftc.teamcode.auto.Steps.startDelay;
import static org.firstinspires.ftc.teamcode.auto.Steps.waitSeconds;
import static org.firstinspires.ftc.teamcode.auto.Steps.when;

import org.firstinspires.ftc.teamcode.auto.AutoContext;
import org.firstinspires.ftc.teamcode.auto.AutoRoutine;
import org.firstinspires.ftc.teamcode.auto.AutoSelection;
import org.firstinspires.ftc.teamcode.auto.AutoStep;
import org.firstinspires.ftc.teamcode.constants.AutoConstants;
import org.firstinspires.ftc.teamcode.constants.AutoConstants.CycleSource;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

import java.util.ArrayList;
import java.util.List;

/**
 * Start at the chosen preset, wait for the partner if we shoot SECOND, shoot the
 * pre-loads from the start, run up to three collection cycles
 * (each one FLOWER or the GARDEN, then back to shoot), wait PARK_DELAY_S, then park.
 *
 * This file is only the order of steps. Every robot pose and option is in AutoConstants
 * and is read when each step starts, so Panels changes apply without re-initialising.
 *
 * To make a new auto: copy this file, change the step list, add any new poses to
 * AutoConstants, and add an OpMode for each alliance like AutoRed.
 */
public final class FullAuto {
    private FullAuto() {
    }

    public static AutoRoutine build(AutoContext ctx) {
        List<AutoStep> steps = new ArrayList<>();

        // SECOND slot: let the partner shoot first (no wait otherwise). The robot holds
        // only its 4 pre-loads, so there is nothing useful to collect meanwhile.
        steps.add(startDelay());

        // Pre-loads, from the start pose: no driving.
        steps.add(shootHere());

        CycleSource[] order = {AutoConstants.CYCLE_1, AutoConstants.CYCLE_2, AutoConstants.CYCLE_3};
        for (int i = 0; i < order.length; i++) {
            int number = i + 1;
            CycleSource source = order[i];
            steps.add(when("cycle " + number,
                    c -> c.cycles >= number && c.timeLeft() >= AutoConstants.CYCLE_MIN_TIME_LEFT_S,
                    sequence("cycle " + number + " " + source,
                            collect(source),
                            intake(Intake.Mode.OFF),
                            shootCycle()),
                    null));
        }

        steps.add(intake(Intake.Mode.OFF));
        steps.add(when("park", c -> AutoConstants.PARK_ENABLED,
                sequence("park",
                        waitSeconds(AutoConstants.PARK_DELAY_S),
                        when("park drive",
                                c -> c.timeLeft() >= AutoConstants.PARK_MIN_TIME_LEFT_S,
                                driveTo("park", c -> c.pose(AutoConstants.PARK_X,
                                        AutoConstants.PARK_Y, AutoConstants.PARK_H)),
                                null)),
                null));

        return new AutoRoutine("Full Auto",
                c -> AutoSelection.preset.pose(c),
                steps.toArray(new AutoStep[0]));
    }

    private static AutoStep collect(CycleSource source) {
        switch (source) {
            case BACK_FLOWER:
                return collectFromFlower("back FLOWER", c -> c.pose(AutoConstants.BACK_FLOWER_X,
                        AutoConstants.BACK_FLOWER_Y, AutoConstants.BACK_FLOWER_H));
            case WALL_FLOWER:
                return collectFromFlower("wall FLOWER", c -> c.pose(AutoConstants.WALL_FLOWER_X,
                        AutoConstants.WALL_FLOWER_Y, AutoConstants.WALL_FLOWER_H));
            case GARDEN:
            default:
                return sequence("GARDEN",
                        intake(Intake.Mode.INTAKE),
                        driveTo("along GARDEN", c -> c.pose(AutoConstants.GARDEN_X,
                                AutoConstants.GARDEN_Y, AutoConstants.GARDEN_H)));
        }
    }

    // Drive to the straight-on spot for whichever CELL is up when this starts (shooter
    // off while driving, to save battery), spin up, shoot, spin down. If the HIVE tips
    // mid-burst, move straight across to the other CELL's spot and finish the burst there.
    private static AutoStep shootCycle() {
        return sequence("shoot",
                driveToShot(),
                shootHere());
    }

    // Pre-loads: the robot starts at its shooting pose, so there is no drive step at all.
    // The drivetrain is never commanded; only the turret aims.
    private static AutoStep shootHere() {
        return sequence("shoot here",
                spinUp(true),
                shoot(),
                when("after a tip", c -> c.shotInterrupted,
                        sequence("finish at other CELL", driveToShot(), spinUp(true), shoot()),
                        null),
                spinUp(false));
    }

    // The shooter goes off only if the robot actually drives (see Steps.driveTo).
    private static AutoStep driveToShot() {
        return byUpSide(
                driveTo("to shot (LEFT CELL up)", c -> c.pose(AutoConstants.SHOOT_LEFT_UP_X,
                        AutoConstants.SHOOT_LEFT_UP_Y, AutoConstants.SHOOT_LEFT_UP_H), true),
                driveTo("to shot", c -> c.pose(AutoConstants.SHOOT_X,
                        AutoConstants.SHOOT_Y, AutoConstants.SHOOT_H), true));
    }
}
