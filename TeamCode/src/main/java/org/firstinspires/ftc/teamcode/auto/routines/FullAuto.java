package org.firstinspires.ftc.teamcode.auto.routines;

import static org.firstinspires.ftc.teamcode.auto.Steps.awaitCamera;
import static org.firstinspires.ftc.teamcode.auto.Steps.byUpSide;
import static org.firstinspires.ftc.teamcode.auto.Steps.collectFromFlower;
import static org.firstinspires.ftc.teamcode.auto.Steps.driveTo;
import static org.firstinspires.ftc.teamcode.auto.Steps.instant;
import static org.firstinspires.ftc.teamcode.auto.Steps.intake;
import static org.firstinspires.ftc.teamcode.auto.Steps.sequence;
import static org.firstinspires.ftc.teamcode.auto.Steps.shoot;
import static org.firstinspires.ftc.teamcode.auto.Steps.spinUp;
import static org.firstinspires.ftc.teamcode.auto.Steps.startDelay;
import static org.firstinspires.ftc.teamcode.auto.Steps.waitSeconds;
import static org.firstinspires.ftc.teamcode.auto.Steps.when;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.auto.AutoContext;
import org.firstinspires.ftc.teamcode.auto.AutoRoutine;
import org.firstinspires.ftc.teamcode.auto.AutoSelection;
import org.firstinspires.ftc.teamcode.auto.AutoStep;
import org.firstinspires.ftc.teamcode.auto.ShotSpots;
import org.firstinspires.ftc.teamcode.constants.AutoConstants;
import org.firstinspires.ftc.teamcode.constants.AutoConstants.CycleSource;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.utils.CellSide;

import java.util.ArrayList;
import java.util.List;

/**
 * Start at the chosen preset, wait for the partner if we shoot SECOND, shoot the
 * pre-loads from the start, run up to three collection cycles
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

        // SECOND slot: let the partner shoot first (no wait otherwise), or go as soon as the
        // camera sees the partner tip the HIVE. The robot holds only its 4 pre-loads, so
        // there is nothing useful to collect meanwhile. After the wait SECOND runs the same
        // loop as SOLO: shoot the up CELL from its spot, collect, shoot again, then park.
        steps.add(startDelay());

        // Pre-loads, from the start pose with no driving, unless that pose cannot score into
        // the CELL that is up now (the HIVE tipped while we waited, or start B with the
        // audience CELL up): then drive to the shot spot first.
        steps.add(when("start pose in zone?",
                c -> AutoConstants.DRIVE_TO_ZONE_FIRST && !c.inZoneNow(c.upSide),
                driveToShot(), null));
        steps.add(shootHere());

        CycleSource[] order = {AutoConstants.CYCLE_1, AutoConstants.CYCLE_2, AutoConstants.CYCLE_3};
        for (int i = 0; i < order.length; i++) {
            int number = i + 1;
            CycleSource source = order[i];
            steps.add(when("cycle " + number,
                    c -> c.cycles >= number && c.timeLeft() >= AutoConstants.CYCLE_MIN_TIME_LEFT_S
                            && mayCycle(c),
                    sequence("cycle " + number + " " + source,
                            collect(source),
                            intake(Intake.Mode.OFF),
                            shootCycle()),
                    null));
        }

        steps.add(intake(Intake.Mode.OFF));
        steps.add(when("park", c -> AutoConstants.PARK_ENABLED,
                sequence("park",
                        // FIRST leaves early so the partner has the shooting area.
                        when("park delay", c -> AutoSelection.slot == AutoSelection.Slot.FIRST,
                                waitSeconds(AutoConstants.FIRST_PARK_DELAY_S),
                                waitSeconds(AutoConstants.PARK_DELAY_S)),
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

    // FIRST leaves the area to the partner: it cycles only if we agreed to stay until the
    // HIVE tips (AutoSelection.noTipPlan), and stops as soon as it has tipped. SOLO and
    // SECOND cycle as long as there is time.
    private static boolean mayCycle(AutoContext c) {
        if (AutoSelection.slot != AutoSelection.Slot.FIRST) {
            return true;
        }
        return AutoSelection.firstStaysUntilTip() && !c.tippedSinceStart();
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
                spinUp(false),
                // Let the HIVE tip and the camera see it before the next trip is planned.
                when("tip settle",
                        c -> AutoConstants.TIP_SETTLE_S > 0 && c.robot.hasAutoAim()
                                && c.cycles > 0 && c.timeLeft() >= AutoConstants.CYCLE_MIN_TIME_LEFT_S
                                && mayCycle(c),
                        awaitCamera("tip check",
                                c -> c.now() + AutoConstants.TIP_SETTLE_MIN_S,
                                AutoConstants.TIP_SETTLE_S),
                        null));
    }

    // Drive to the spot for the CELL up now, then check the camera once more on arrival: if
    // the HIVE tipped during the trip, go on to the other CELL's spot before spinning up.
    private static AutoStep driveToShot() {
        return sequence("to shot",
                instant("plan trip", c -> {
                    c.plannedSide = c.upSide;
                    c.tripStartS = c.now();
                }),
                driveToSpot(),
                when("arrival check",
                        c -> AutoConstants.ARRIVE_CHECK_S > 0 && c.robot.hasAutoAim(),
                        sequence("arrival check",
                                awaitCamera("camera on arrival", c -> c.tripStartS,
                                        AutoConstants.ARRIVE_CHECK_S),
                                when("tipped on the way", c -> c.upSide != c.plannedSide,
                                        sequence("redirect",
                                                instant("replan", c -> c.plannedSide = c.upSide),
                                                driveToSpot()),
                                        null)),
                        null));
    }

    // The shooter goes off only if the robot actually drives (see Steps.driveTo).
    private static AutoStep driveToSpot() {
        return byUpSide(
                driveTo("to shot (LEFT CELL up)", c -> shotPose(c, CellSide.LEFT), true),
                driveTo("to shot", c -> shotPose(c, CellSide.RIGHT), true));
    }

    // The spot for this CELL and our slot (ShotSpots), as a pose for our alliance.
    private static Pose shotPose(AutoContext c, CellSide cell) {
        double[] s = ShotSpots.spot(cell, AutoSelection.slot);
        return c.pose(s[0], s[1], s[2]);
    }
}
