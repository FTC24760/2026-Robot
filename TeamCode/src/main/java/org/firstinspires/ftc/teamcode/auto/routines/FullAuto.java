package org.firstinspires.ftc.teamcode.auto.routines;

import static org.firstinspires.ftc.teamcode.auto.Steps.byUpSide;
import static org.firstinspires.ftc.teamcode.auto.Steps.collectFromFlower;
import static org.firstinspires.ftc.teamcode.auto.Steps.driveTo;
import static org.firstinspires.ftc.teamcode.auto.Steps.intake;
import static org.firstinspires.ftc.teamcode.auto.Steps.sequence;
import static org.firstinspires.ftc.teamcode.auto.Steps.shoot;
import static org.firstinspires.ftc.teamcode.auto.Steps.spinUp;
import static org.firstinspires.ftc.teamcode.auto.Steps.when;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.auto.AutoContext;
import org.firstinspires.ftc.teamcode.auto.AutoRoutine;
import org.firstinspires.ftc.teamcode.auto.AutoStep;
import org.firstinspires.ftc.teamcode.constants.FieldConstants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

import java.util.ArrayList;
import java.util.List;

/**
 * Shoot the pre-loads from the back spot, then run up to three collection cycles (each
 * one FLOWER or the GARDEN, then back to shoot), then park. The number of cycles is
 * chosen on the d-pad during init; their order is CYCLE_1..CYCLE_3 below.
 *
 * Poses are RED, in inches and degrees, in the frame described in AutoConstants. Field
 * features (CELLS, FLOWERS, GARDEN, LOADING ZONE) are in FieldConstants. Everything is
 * @Configurable and read when each step starts. All values are starting guesses from
 * Figure 10-2, not measured on a field.
 *
 * To make a new auto: copy this file, change the poses and the step list, and add an
 * OpMode for each alliance like AutoRed.
 */
@Configurable
public class FullAuto {

    public enum Source {
        FAR_FLOWER,
        WALL_FLOWER,
        GARDEN
    }

    // ---- Start: touching our wall, just toward the audience from the LOADING ZONE. ----
    public static double START_X = 9, START_Y = 84, START_H = 0;

    // ---- Where to shoot from, one spot per up CELL (as our drivers see them). ----
    // Both default to the back corner of our side, away from the audience. That spot is
    // about 2.2 m from the RIGHT CELL, matching the distance the shooter was calibrated
    // at (ShotConstants.CAL_DISTANCE_M). The turret aims; the heading only needs to keep
    // the CELL within the turret's range.
    public static double SHOOT_RIGHT_X = 12, SHOOT_RIGHT_Y = 130, SHOOT_RIGHT_H = -55;
    public static double SHOOT_LEFT_X = 12, SHOOT_LEFT_Y = 130, SHOOT_LEFT_H = -40;

    // ---- Collection cycles, in the order they run. ----
    // Default: nearest to the shooting spot first.
    public static Source CYCLE_1 = Source.FAR_FLOWER;
    public static Source CYCLE_2 = Source.WALL_FLOWER;
    public static Source CYCLE_3 = Source.GARDEN;
    // Skip a cycle that starts with less than this much of the auto left.
    public static double CYCLE_MIN_TIME_LEFT_S = 9;

    // ---- GARDEN: drive along the audience wall toward our corner, intake first. ----
    public static double GARDEN_APPROACH_X = 32, GARDEN_APPROACH_Y = 10;
    public static double GARDEN_END_X = 10, GARDEN_END_Y = 10;
    public static double GARDEN_H = 180;

    // ---- Park partly in the LOADING ZONE (worth PARK points in AUTO). ----
    public static boolean PARK_ENABLED = true;
    public static double PARK_MIN_TIME_LEFT_S = 3;
    public static double PARK_X = 12, PARK_Y = 108, PARK_H = 0;

    public static AutoRoutine build(AutoContext ctx) {
        List<AutoStep> steps = new ArrayList<>();

        // Pre-loads: spin up on the way, shoot at whichever CELL is up.
        steps.add(shootCycle());

        Source[] order = {CYCLE_1, CYCLE_2, CYCLE_3};
        for (int i = 0; i < order.length; i++) {
            int number = i + 1;
            Source source = order[i];
            steps.add(when("cycle " + number,
                    c -> c.cycles >= number && c.timeLeft() >= CYCLE_MIN_TIME_LEFT_S,
                    sequence("cycle " + number + " " + source,
                            collect(source),
                            intake(Intake.Mode.OFF),
                            shootCycle()),
                    null));
        }

        steps.add(intake(Intake.Mode.OFF));
        steps.add(when("park",
                c -> PARK_ENABLED && c.timeLeft() >= PARK_MIN_TIME_LEFT_S,
                driveTo("park", c -> c.pose(PARK_X, PARK_Y, PARK_H)),
                null));

        return new AutoRoutine("Full Auto",
                c -> c.pose(START_X, START_Y, START_H),
                steps.toArray(new AutoStep[0]));
    }

    private static AutoStep collect(Source source) {
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
                        driveTo("to GARDEN", c -> c.pose(GARDEN_APPROACH_X, GARDEN_APPROACH_Y, GARDEN_H)),
                        driveTo("along GARDEN", c -> c.pose(GARDEN_END_X, GARDEN_END_Y, GARDEN_H)));
        }
    }

    // Spin up on the way, drive to the spot for whichever CELL is up when this starts,
    // shoot, spin down.
    private static AutoStep shootCycle() {
        return sequence("shoot",
                spinUp(true),
                byUpSide(
                        driveTo("to shot (LEFT CELL up)",
                                c -> c.pose(SHOOT_LEFT_X, SHOOT_LEFT_Y, SHOOT_LEFT_H)),
                        driveTo("to shot (RIGHT CELL up)",
                                c -> c.pose(SHOOT_RIGHT_X, SHOOT_RIGHT_Y, SHOOT_RIGHT_H))),
                shoot(),
                spinUp(false));
    }
}
