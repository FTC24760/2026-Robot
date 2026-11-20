package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.constants.AutoConstants;
import org.firstinspires.ftc.teamcode.constants.FieldConstants;
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

    /** True if the last shot step stopped early because the HIVE tipped part-way. */
    public boolean shotInterrupted;

    /** How many collection cycles to run after the pre-loads (chosen during init). */
    public int cycles = AutoConstants.DEFAULT_CYCLES;

    // Balls we have shot into the current up CELL, for guessing when it tips.
    private int ballsSinceTip;

    // Background jobs, applied every loop by AutoOpMode.
    public boolean spinUp;
    public Intake.Mode intakeMode = Intake.Mode.OFF;
    public boolean flowerDown;

    private double startS;

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
        if (alliance != Alliance.BLUE) {
            return new Pose(x, y, h);
        }

        double size = AutoConstants.FIELD_SIZE_IN;
        return new Pose(size - x, size - y, h + Math.PI);
    }

    /** Field position of one of our CELLS (heading unused). */
    public Pose cellPose(CellSide side) {
        return side == CellSide.LEFT
                ? pose(FieldConstants.LEFT_CELL_X, FieldConstants.LEFT_CELL_Y, 0)
                : pose(FieldConstants.RIGHT_CELL_X, FieldConstants.RIGHT_CELL_Y, 0);
    }

    /**
     * Where the robot stops to collect from a FLOWER, written for red: FLOWER_STANDOFF_IN
     * back from the FLOWER, facing it.
     */
    public Pose flowerApproach(double flowerX, double flowerY, double facingDeg) {
        double f = Math.toRadians(facingDeg);
        double d = FieldConstants.FLOWER_STANDOFF_IN;
        return pose(flowerX - d * Math.cos(f), flowerY - d * Math.sin(f), facingDeg);
    }
}
