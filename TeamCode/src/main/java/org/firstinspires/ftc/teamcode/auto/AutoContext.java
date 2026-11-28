package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;

import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.constants.AutoConstants;
import org.firstinspires.ftc.teamcode.constants.ShootGateConstants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.utils.Alliance;
import org.firstinspires.ftc.teamcode.utils.CellSide;

/**
 * What steps can see and change while an auto runs: the robot, the alliance, which CELL
 * of our HIVE is up, the clock, and the background jobs (shooter spin-up, intake, flower
 * arm) that keep running underneath whatever step is active.
 */
public class AutoContext {
    public final Robot robot;
    public final Alliance alliance;

    /** Which CELL of our HIVE is up, as our drivers see it. Change it with setUpSide(). */
    public CellSide upSide;

    /**
     * How many tips the camera (or a broken-off burst) has shown since the routine started.
     * A count, not a comparison with the starting side, so two tips are not read as none.
     */
    public int tips;

    /** Which sources a cycle has already emptied, by CycleSource ordinal. */
    public final boolean[] sourceUsed = new boolean[AutoConstants.CycleSource.values().length];
    /** The source of the cycle running now. */
    public AutoConstants.CycleSource chosenSource;

    /** Cycles started so far (0, 1, 2); the next one uses this index. */
    public int cycleIndex;

    /**
     * The CELL to shoot into next. SOLO and FIRST: whichever is up, whoever tipped it.
     * SECOND (the weaker robot, start B) and EQUAL: always our own CELL (the zone by our
     * start), so we never drive into the partner's zone; we wait there until that CELL is
     * up (waitsForCell).
     */
    public CellSide shotSide() {
        CellSide own = AutoSelection.ownSide(alliance);
        if (AutoSelection.ownCellOnly()) {
            return own;
        }
        // The other CELL has no usable zone point (e.g. FIRST keeping clear of a partner on
        // the alliance wall): stay with our own CELL and hold fire until it is up again.
        if (upSide != own && !ZoneTargets.hasTarget(upSide, AutoSelection.pair())) {
            return own;
        }
        return upSide;
    }

    /** True while the CELL we shoot into is facing down: hold fire and wait for a tip. */
    public boolean waitsForCell() {
        return shotSide() != upSide;
    }

    /** True if the last shot step stopped early because the HIVE tipped part-way. */
    public boolean shotInterrupted;

    /** How many collection cycles to run after the pre-loads (chosen during init). */
    public int cycles = AutoSelection.cycles;

    // Balls we have shot into the current up CELL, for guessing when it tips.
    private int ballsSinceTip;
    private double burstEndS;

    // Background jobs, applied every loop by AutoOpMode.
    public boolean spinUp;
    public Intake.Mode intakeMode = Intake.Mode.OFF;
    public boolean flowerDown;

    // Set when the route's own park drive starts; the reserve window then stays out of it.
    public boolean parkStarted;

    private double startS;

    // Poses written past our side of the field (G402), found while the routine was built.
    private final List<String> sideWarnings = new ArrayList<>();

    public AutoContext(Robot robot, Alliance alliance, CellSide upSide) {
        this.robot = robot;
        this.alliance = alliance;
        this.upSide = upSide;
    }

    /** Record which CELL is up. A change means the HIVE tipped. */
    public void setUpSide(CellSide side) {
        if (side != upSide) {
            upSide = side;
            tips++;
            ballsSinceTip = 0;
        }
    }

    /** Balls needed in the up CELL for the next tip: 4 for the first, then 8. */
    public int ballsToTip() {
        return tips == 0 ? AutoConstants.BALLS_FIRST_TIP : AutoConstants.BALLS_NEXT_TIP;
    }

    /** Record a finished burst: our balls count toward the next tip. */
    public void recordShot(int balls) {
        ballsSinceTip += balls;
        burstEndS = now();
    }

    /**
     * Alone, a burst that leaves the count under the threshold cannot have tipped the HIVE,
     * so the camera wait can be skipped. With a partner its balls are not counted.
     */
    public boolean mayHaveTipped() {
        return AutoSelection.pair() || ballsSinceTip >= ballsToTip();
    }

    /**
     * After a burst and the camera wait: if the camera read the HIVE since the burst, it
     * decides (and a CELL it still shows up caps the count below the threshold). If it did
     * not, believe the count and take it that the CELL tipped.
     */
    public void settleTipBelief() {
        boolean read = lastCellReadS >= burstEndS && burstEndS > 0;
        if (read) {
            if (ballsSinceTip >= ballsToTip()) {
                ballsSinceTip = ballsToTip() - 1;
            }
        } else if (AutoConstants.PREDICT_TIP_WHEN_BLIND && ballsSinceTip >= ballsToTip()) {
            setUpSide(upSide.other());
        }
    }

    /**
     * The "which CELL is up" half of the ready-to-fire test: the camera read one of our CELLS
     * within UPSIDE_MAX_AGE_S and after the last burst (which may have tipped the HIVE).
     * Always true without a camera (there is nothing to wait for).
     */
    public boolean upStateKnown() {
        if (!ShootGateConstants.UPSIDE_CHECK || !robot.hasAutoAim() || !AutoConstants.DETECT_UP_SIDE) {
            return true;
        }
        return lastCellReadS >= 0 && now() - lastCellReadS <= ShootGateConstants.UPSIDE_MAX_AGE_S
                && lastCellReadS >= burstEndS;
    }

    /** Distance from the robot (odometry) to the CELL believed up, inches. */
    public double upCellDistanceIn() {
        Pose p = robot.drivetrain.getPose();
        return ZoneTargets.distanceIn(redX(p), redY(p), upSide);
    }

    /** Degrees the shot from here (odometry) is off the up CELL's opening axis. */
    public double upCellOffAxisDeg() {
        Pose p = robot.drivetrain.getPose();
        return ZoneTargets.offAxisDeg(redX(p), redY(p), upSide);
    }

    void markStart(double nowS) {
        ZoneTargets.forget();
        startS = nowS;
        tips = 0;
        ballsSinceTip = 0;
        burstEndS = 0;
        cycleIndex = 0;
        java.util.Arrays.fill(sourceUsed, false);
        chosenSource = null;
    }

    /** When (ctx.now()) the camera last read one of our CELLS, up or down; -1 = never. */
    public double lastCellReadS = -1;
    /** When the current trip into the zone started, and which CELL it was planned for. */
    public double tripStartS;
    public CellSide plannedSide;

    /** True if the HIVE has tipped since the routine started (the camera saw it). */
    public boolean tippedSinceStart() {
        return tips > 0;
    }

    public double now() {
        return robot.nowSeconds();
    }

    /** Seconds since the auto started. */
    public double elapsed() {
        return now() - startS;
    }

    /** Seconds left in the autonomous period. */
    public double timeLeft() {
        return AutoConstants.AUTO_LENGTH_S - elapsed();
    }

    /**
     * A pose written for red (inches, degrees; see AutoConstants for the axes), converted
     * to this alliance. Write every pose in a routine through this.
     *
     * The blue side is the red side turned half a circle about the field centre (Figure
     * 10-2), so "our left" and "our right" stay the same for both alliances.
     */
    public Pose pose(double x, double y, double headingDeg) {
        double h = Math.toRadians(headingDeg);
        if (x > AutoConstants.OWN_SIDE_MAX_X_IN - AutoConstants.OWN_SIDE_MARGIN_IN
                && sideWarnings.size() < 5) {
            sideWarnings.add(String.format("x %.0f y %.0f", x, y));
        }
        if (alliance != Alliance.BLUE) {
            return new Pose(x, y, h);
        }

        double size = AutoConstants.FIELD_SIZE_IN;
        return new Pose(size - x, size - y, h + Math.PI);
    }

    /**
     * Run the fixed route poses through pose() so any that are past our side of the field
     * are reported at init (the routine's own poses are only computed as each step starts).
     */
    public void checkRouteConstants() {
        pose(AutoConstants.START_A_X, AutoConstants.START_A_Y, 0);
        pose(AutoConstants.START_B_X, AutoConstants.START_B_Y, 0);
        pose(AutoConstants.GARDEN_LEAD_IN_X, AutoConstants.GARDEN_Y, 0);
        pose(AutoConstants.WALL_FLOWER_X, AutoConstants.WALL_FLOWER_Y, 0);
        pose(AutoConstants.BACK_FLOWER_X, AutoConstants.BACK_FLOWER_Y, 0);
        pose(AutoConstants.GARDEN_X, AutoConstants.GARDEN_Y, 0);
        pose(AutoConstants.PARK_X, AutoConstants.PARK_Y, 0);
    }

    /**
     * Problems with the shooting zone, as text for the init screen (red frame, where our
     * RIGHT cell is the AUDIENCE one): the chosen start is outside the zone for the CELL up
     * at the start (the robot drives in first), or a CELL has no usable zone point at all
     * (e.g. the shot table is empty, or the limits leave nothing).
     */
    public List<String> zoneWarnings(StartPreset preset, CellSide upAtStart) {
        List<String> out = new ArrayList<>();
        boolean startB = preset == StartPreset.B;
        double sx = startB ? AutoConstants.START_B_X : AutoConstants.START_A_X;
        double sy = startB ? AutoConstants.START_B_Y : AutoConstants.START_A_Y;
        if (!ZoneTargets.inZone(sx, sy, upAtStart)) {
            out.add(String.format("start %s -> %s CELL: %.0f in, %.0f deg off the opening (drives in first)",
                    preset, upAtStart, ZoneTargets.distanceIn(sx, sy, upAtStart),
                    ZoneTargets.offAxisDeg(sx, sy, upAtStart)));
        }
        for (CellSide cell : CellSide.values()) {
            if (ZoneTargets.target(cell, sx, sy, AutoSelection.pair()) == null) {
                out.add("no usable zone point for the " + cell + " CELL");
            }
        }
        return out;
    }

    /** Poses the routine uses that are past our side of the field in AUTO (red frame). */
    public List<String> sideWarnings() {
        return sideWarnings;
    }

    /** A pose's x as red sees it, for either alliance. */
    public double redX(Pose pose) {
        return alliance == Alliance.BLUE ? AutoConstants.FIELD_SIZE_IN - pose.x() : pose.x();
    }

    /** True if the robot is where a shot into the up CELL can score (shooting zone). */
    public boolean inZoneNow(CellSide cell) {
        Pose p = robot.drivetrain.getPose();
        return ZoneTargets.inZone(redX(p), redY(p), cell);
    }

    /** A pose's distance from the audience wall as red sees it, for either alliance. */
    public double redY(Pose pose) {
        return alliance == Alliance.BLUE ? AutoConstants.FIELD_SIZE_IN - pose.y() : pose.y();
    }
}
