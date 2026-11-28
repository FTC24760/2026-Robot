package org.firstinspires.ftc.teamcode.auto.routines;

import static org.firstinspires.ftc.teamcode.auto.Steps.awaitCamera;
import static org.firstinspires.ftc.teamcode.auto.Steps.collectFromFlower;
import static org.firstinspires.ftc.teamcode.auto.Steps.driveTo;
import static org.firstinspires.ftc.teamcode.auto.Steps.instant;
import static org.firstinspires.ftc.teamcode.auto.Steps.toZoneAndReady;
import static org.firstinspires.ftc.teamcode.auto.Steps.intake;
import static org.firstinspires.ftc.teamcode.auto.Steps.sequence;
import static org.firstinspires.ftc.teamcode.auto.Steps.shoot;
import static org.firstinspires.ftc.teamcode.auto.Steps.waitSeconds;
import static org.firstinspires.ftc.teamcode.auto.Steps.when;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.auto.AutoContext;
import org.firstinspires.ftc.teamcode.auto.AutoRoutine;
import org.firstinspires.ftc.teamcode.auto.AutoSelection;
import org.firstinspires.ftc.teamcode.auto.AutoStep;
import org.firstinspires.ftc.teamcode.auto.StartPreset;
import org.firstinspires.ftc.teamcode.auto.TripTimes;
import org.firstinspires.ftc.teamcode.auto.ZoneTargets;
import org.firstinspires.ftc.teamcode.constants.AutoConstants;
import org.firstinspires.ftc.teamcode.constants.AutoConstants.CycleSource;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

import java.util.ArrayList;
import java.util.List;

/**
 * Start at the chosen preset, shoot the pre-loads from the start (PAIR at B: once the camera
 * sees our CELL up), run up to three collection cycles
 * (each one FLOWER or the GARDEN, then back to shoot), then park (the reserve window banks the park).
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


        // Pre-loads: the start is in the zone and faces the HIVE, so they go from there. SOLO
        // drives into the other zone first if that CELL is up. PAIR at start B holds fire
        // until its camera sees our CELL up (the partner's pre-loads tipped the HIVE).
        steps.add(zoneShot());

        for (int i = 0; i < 3; i++) {
            int number = i + 1;
            int index = i;
            steps.add(when("cycle " + number,
                    c -> c.cycles >= number && mayCycle(c) && chooseSource(c, index) != null,
                    sequence("cycle " + number,
                            instant("choose source", c -> {
                                c.cycleIndex = index + 1;
                                c.chosenSource = chooseSource(c, index);
                                c.sourceUsed[c.chosenSource.ordinal()] = true;
                            }),
                            // Chosen when the step starts: the slot and the up CELL are only
                            // known by then.
                            collectChosen(),
                            intake(Intake.Mode.OFF),
                            zoneShot()),
                    null));
        }

        steps.add(intake(Intake.Mode.OFF));
        steps.add(when("park", c -> AutoConstants.PARK_ENABLED,
                sequence("park",
                        waitSeconds(AutoConstants.PARK_DELAY_S),
                        when("park drive",
                                c -> c.timeLeft() >= AutoConstants.PARK_MIN_TIME_LEFT_S,
                                sequence("park drive",
                                        instant("park started", c -> c.parkStarted = true),
                                        driveToPark()),
                                null)),
                null));

        return new AutoRoutine("Full Auto",
                c -> AutoSelection.preset.pose(c),
                steps.toArray(new AutoStep[0]))
                .withReserve(driveToPark());
    }

    private static boolean mayCycle(AutoContext c) {
        return true;
    }

    // The source for this cycle (0, 1, 2) in our slot. Read when a step starts, because
    // the slot is picked in the init menu after the routine is built.
    private static CycleSource sourceFor(int index) {
        if (!AutoSelection.pair()) {
            return AutoConstants.SOLO_SOURCES[index];
        }
        // PAIR: only our side's sources, nearest first.
        return AutoSelection.preset == StartPreset.A
                ? AutoConstants.PAIR_A_SOURCES[index] : AutoConstants.PAIR_B_SOURCES[index];
    }

    /**
     * The source for this cycle, or null if none fits. SOLO (SOLO_NEAREST_SOURCE) takes the
     * unused source nearest to the robot; PAIR takes the next one of our side's sources. A
     * source counts only if its trips, a burst, the park trip and the margin fit in the time
     * left: the park is never given up for a cycle.
     */
    private static CycleSource chooseSource(AutoContext c, int index) {
        CycleSource[] candidates;
        if (!AutoSelection.pair() && AutoConstants.SOLO_NEAREST_SOURCE) {
            candidates = new CycleSource[] {
                    CycleSource.GARDEN, CycleSource.WALL_FLOWER, CycleSource.BACK_FLOWER};
        } else {
            candidates = new CycleSource[] {sourceFor(index)};
        }

        Pose me = c.robot.drivetrain.getPose();
        double[] from = {c.redX(me), c.redY(me)};
        CycleSource best = null;
        double bestS = Double.MAX_VALUE;
        for (CycleSource source : candidates) {
            if (source == CycleSource.NONE || c.sourceUsed[source.ordinal()]) {
                continue;
            }
            double[] at = TripTimes.sourceXY(source);
            double[] t = ZoneTargets.target(c.shotSide(), at[0], at[1], AutoSelection.pair());
            double[] next = t != null ? new double[] {t[0], t[1]} : at;
            double park = TripTimes.driveS(next[0], next[1], AutoConstants.PARK_X,
                    AutoConstants.PARK_Y);
            double need = TripTimes.cycleS(source, from, next) + AutoConstants.BURST_S + park
                    + AutoConstants.CYCLE_MARGIN_S;
            double away = TripTimes.driveS(from[0], from[1], at[0], at[1]);
            if (need <= c.timeLeft() && away < bestS) {
                best = source;
                bestS = away;
            }
        }
        return best;
    }

    private static AutoStep collectChosen() {
        return when("source garden", c -> c.chosenSource == CycleSource.GARDEN,
                collect(CycleSource.GARDEN),
                when("source wall flower", c -> c.chosenSource == CycleSource.WALL_FLOWER,
                        collect(CycleSource.WALL_FLOWER),
                        collect(CycleSource.BACK_FLOWER)));
    }

    private static AutoStep driveToPark() {
        return driveTo("park", c -> c.pose(AutoConstants.PARK_X,
                AutoConstants.PARK_Y, AutoConstants.PARK_H));
    }

    private static AutoStep collect(CycleSource source) {
        switch (source) {
            case BACK_FLOWER:
                return collectFromFlower("back FLOWER", c -> c.pose(AutoConstants.BACK_FLOWER_X,
                        AutoConstants.BACK_FLOWER_Y, AutoConstants.BACK_FLOWER_H));
            case WALL_FLOWER:
                return collectFromFlower("wall FLOWER", c -> c.pose(AutoConstants.WALL_FLOWER_X,
                        AutoConstants.WALL_FLOWER_Y, AutoConstants.WALL_FLOWER_H));
            case NONE:
                return instant("no source", c -> { });
            case GARDEN:
            default:
                return sequence("GARDEN",
                        intake(Intake.Mode.INTAKE),
                        driveTo("line up at GARDEN", c -> c.pose(AutoConstants.GARDEN_LEAD_IN_X,
                                AutoConstants.GARDEN_Y, AutoConstants.GARDEN_H)),
                        driveTo("along GARDEN", c -> c.pose(AutoConstants.GARDEN_X,
                                AutoConstants.GARDEN_Y, AutoConstants.GARDEN_H)));
        }
    }

    // Into the zone for the CELL that is up (no drive if already in it), wait until ready to
    // fire, fire. If the HIVE tips mid-burst, go to the other CELL's zone and finish there.
    // Then let the HIVE tip and the camera (facing it) see it before the next trip is planned.
    private static AutoStep zoneShot() {
        return sequence("zone shot",
                toZoneAndReady(),
                shoot(),
                when("after a tip", c -> c.shotInterrupted,
                        sequence("finish at other CELL", toZoneAndReady(), shoot()),
                        null),
                when("tip settle",
                        c -> AutoConstants.TIP_SETTLE_S > 0 && c.robot.hasAutoAim()
                                && c.cycles > c.cycleIndex && c.mayHaveTipped()
                                && chooseSource(c, c.cycleIndex) != null,
                        awaitCamera("tip check",
                                c -> c.now() + AutoConstants.TIP_SETTLE_MIN_S,
                                AutoConstants.TIP_SETTLE_S),
                        null),
                // No camera read of the HIVE: believe the ball count (4, then 8).
                instant("tip belief", AutoContext::settleTipBelief));
    }
}
