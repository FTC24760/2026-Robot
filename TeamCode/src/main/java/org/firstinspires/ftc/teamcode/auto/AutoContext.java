package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;

import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.constants.AutoConstants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.utils.Alliance;
import org.firstinspires.ftc.teamcode.utils.CellSide;
import org.firstinspires.ftc.teamcode.utils.HiveSide;
import org.firstinspires.ftc.teamcode.utils.ShotZoneGeometry;

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

    /** The up CELL when the routine started; a different one later means the HIVE tipped. */
    public CellSide upAtStart;

    /** True if the last shot step stopped early because the HIVE tipped part-way. */
    public boolean shotInterrupted;

    /** How many collection cycles to run after the pre-loads (chosen during init). */
    public int cycles = AutoSelection.cycles;

    // Balls we have shot into the current up CELL, for guessing when it tips.
    private int ballsSinceTip;

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
            ballsSinceTip = 0;
        }
    }

    /**
     * Record a finished shot. If BALLS_TO_TIP is set, assume the HIVE tips once that many
     * balls have gone in, so the next shot goes to the other CELL even if the Limelight
     * never saw it move.
     */
    public void recordShot(int balls) {
        ballsSinceTip += balls;
        if (AutoConstants.BALLS_TO_TIP > 0 && ballsSinceTip >= AutoConstants.BALLS_TO_TIP) {
            setUpSide(upSide.other());
        }
    }

    void markStart(double nowS) {
        startS = nowS;
        upAtStart = upSide;
    }

    /** True if the HIVE has tipped since the routine started (the camera saw it). */
    public boolean tippedSinceStart() {
        return upAtStart != null && upSide != upAtStart;
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
        pose(AutoConstants.SHOOT_X, AutoConstants.SHOOT_Y, 0);
        pose(AutoConstants.SHOOT_LEFT_UP_X, AutoConstants.SHOOT_LEFT_UP_Y, 0);
        pose(AutoConstants.WALL_FLOWER_X, AutoConstants.WALL_FLOWER_Y, 0);
        pose(AutoConstants.BACK_FLOWER_X, AutoConstants.BACK_FLOWER_Y, 0);
        pose(AutoConstants.GARDEN_X, AutoConstants.GARDEN_Y, 0);
        pose(AutoConstants.PARK_X, AutoConstants.PARK_Y, 0);
    }

    /**
     * Shots the route takes from a spot that is outside the shooting zone (the same test
     * TeleOp uses: ShotZoneGeometry), as text for the init screen. Checked in the red
     * frame, where our RIGHT cell is the AUDIENCE one and opens toward the audience wall.
     * Covers the pre-loads from the chosen start pose into the CELL up at the start, and
     * the two fixed shot spots into the CELL each one is for.
     */
    public List<String> zoneWarnings(StartPreset preset, CellSide upAtStart) {
        List<String> out = new ArrayList<>();
        boolean startB = preset == StartPreset.B;
        checkZone(out, "start " + preset + " pre-loads",
                startB ? AutoConstants.START_B_X : AutoConstants.START_A_X,
                startB ? AutoConstants.START_B_Y : AutoConstants.START_A_Y, upAtStart);
        checkZone(out, "shot spot (RIGHT up)", AutoConstants.SHOOT_X, AutoConstants.SHOOT_Y,
                CellSide.RIGHT);
        checkZone(out, "shot spot (LEFT up)", AutoConstants.SHOOT_LEFT_UP_X,
                AutoConstants.SHOOT_LEFT_UP_Y, CellSide.LEFT);
        return out;
    }

    private static void checkZone(List<String> out, String what, double x, double y, CellSide cell) {
        boolean right = cell == CellSide.RIGHT;
        double cx = right ? AutoConstants.RIGHT_CELL_X : AutoConstants.LEFT_CELL_X;
        double cy = right ? AutoConstants.RIGHT_CELL_Y : AutoConstants.LEFT_CELL_Y;
        double dist = ShotZoneGeometry.distanceIn(x, y, cx, cy);
        double off = ShotZoneGeometry.offAxisDeg(x, y, cx, cy,
                right ? HiveSide.AUDIENCE : HiveSide.FAR);
        if (!zoneOk(x, y, cell)) {
            out.add(String.format("%s -> %s CELL: %.0f in, %.0f deg off the opening",
                    what, cell, dist, off));
        }
    }

    // The shooting-zone test for a spot (red frame) and the CELL it shoots into.
    private static boolean zoneOk(double x, double y, CellSide cell) {
        boolean right = cell == CellSide.RIGHT;
        double cx = right ? AutoConstants.RIGHT_CELL_X : AutoConstants.LEFT_CELL_X;
        double cy = right ? AutoConstants.RIGHT_CELL_Y : AutoConstants.LEFT_CELL_Y;
        return ShotZoneGeometry.inZone(ShotZoneGeometry.distanceIn(x, y, cx, cy),
                ShotZoneGeometry.offAxisDeg(x, y, cx, cy, right ? HiveSide.AUDIENCE : HiveSide.FAR),
                false);
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
        return zoneOk(redX(p), redY(p), cell);
    }

    /** A pose's distance from the audience wall as red sees it, for either alliance. */
    public double redY(Pose pose) {
        return alliance == Alliance.BLUE ? AutoConstants.FIELD_SIZE_IN - pose.y() : pose.y();
    }
}
