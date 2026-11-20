package org.firstinspires.ftc.teamcode.auto.routines;

import static org.firstinspires.ftc.teamcode.auto.Steps.byUpSide;
import static org.firstinspires.ftc.teamcode.auto.Steps.collectFromFlower;
import static org.firstinspires.ftc.teamcode.auto.Steps.driveTo;
import static org.firstinspires.ftc.teamcode.auto.Steps.intake;
import static org.firstinspires.ftc.teamcode.auto.Steps.sequence;
import static org.firstinspires.ftc.teamcode.auto.Steps.shoot;
import static org.firstinspires.ftc.teamcode.auto.Steps.spinUp;
import static org.firstinspires.ftc.teamcode.auto.Steps.when;

import org.firstinspires.ftc.teamcode.auto.AutoContext;
import org.firstinspires.ftc.teamcode.auto.AutoRoutine;
import org.firstinspires.ftc.teamcode.auto.AutoStep;
import org.firstinspires.ftc.teamcode.constants.AutoConstants;
import org.firstinspires.ftc.teamcode.constants.AutoConstants.CycleSource;
import org.firstinspires.ftc.teamcode.constants.FieldConstants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

import java.util.ArrayList;
import java.util.List;

/**
 * Shoot the pre-loads straight on at the up CELL, run up to three collection cycles
 * (each one FLOWER or the GARDEN, then back to shoot), then park.
 *
 * This file is only the order of steps. Every pose and option is in AutoConstants, and
 * field positions are in FieldConstants; all are read when each step starts, so Panels
 * changes apply without re-initialising.
 *
 * To make a new auto: copy this file, change the step list, add any new poses to
 * AutoConstants, and add an OpMode for each alliance like AutoRed.
 */
public final class FullAuto {
    private FullAuto() {
    }

    public static AutoRoutine build(AutoContext ctx) {
        List<AutoStep> steps = new ArrayList<>();

        // Pre-loads.
        steps.add(shootCycle());

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
        steps.add(when("park",
                c -> AutoConstants.PARK_ENABLED && c.timeLeft() >= AutoConstants.PARK_MIN_TIME_LEFT_S,
                driveTo("park", c -> c.pose(
                        AutoConstants.PARK_X, AutoConstants.PARK_Y, AutoConstants.PARK_H)),
                null));

        return new AutoRoutine("Full Auto",
                c -> c.pose(AutoConstants.START_X, AutoConstants.START_Y, AutoConstants.START_H),
                steps.toArray(new AutoStep[0]));
    }

    private static AutoStep collect(CycleSource source) {
        switch (source) {
            case FAR_FLOWER:
                return collectFromFlower("far FLOWER", c -> c.flowerApproach(
                        FieldConstants.FAR_FLOWER_X, FieldConstants.FAR_FLOWER_Y,
                        FieldConstants.FAR_FLOWER_FACING_DEG));
            case WALL_FLOWER:
                return collectFromFlower("wall FLOWER", c -> c.flowerApproach(
                        FieldConstants.WALL_FLOWER_X, FieldConstants.WALL_FLOWER_Y,
                        FieldConstants.WALL_FLOWER_FACING_DEG));
            case GARDEN:
            default:
                return sequence("GARDEN",
                        intake(Intake.Mode.INTAKE),
                        driveTo("to GARDEN", c -> c.pose(AutoConstants.GARDEN_APPROACH_X,
                                AutoConstants.GARDEN_APPROACH_Y, AutoConstants.GARDEN_H)),
                        driveTo("along GARDEN", c -> c.pose(AutoConstants.GARDEN_END_X,
                                AutoConstants.GARDEN_END_Y, AutoConstants.GARDEN_H)));
        }
    }

    // Spin up on the way to the straight-on spot for whichever CELL is up when this
    // starts, shoot, spin down. If the HIVE tips mid-burst, move straight across to the
    // other CELL's spot and finish the burst there.
    private static AutoStep shootCycle() {
        return sequence("shoot",
                spinUp(true),
                driveToShot(),
                shoot(),
                when("after a tip", c -> c.shotInterrupted,
                        sequence("finish at other CELL", driveToShot(), shoot()),
                        null),
                spinUp(false));
    }

    private static AutoStep driveToShot() {
        return byUpSide(
                driveTo("to shot (LEFT CELL up)",
                        c -> c.pose(AutoConstants.SHOOT_X, FieldConstants.LEFT_CELL_Y, 0)),
                driveTo("to shot (RIGHT CELL up)",
                        c -> c.pose(AutoConstants.SHOOT_X, FieldConstants.RIGHT_CELL_Y, 0)));
    }
}
