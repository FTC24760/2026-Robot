package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

import org.firstinspires.ftc.teamcode.constants.AutoConstants;

import java.util.Arrays;
import java.util.function.Function;

/**
 * A whole autonomous: where the robot starts, and the steps it runs in order.
 * Build one in a class under auto/routines; see RapidFire for an example.
 */
public class AutoRoutine {
    private final String name;
    private final Function<AutoContext, Pose> startPose;
    private final Steps.SequenceStep steps;
    private boolean finished;
    private AutoStep reserve;
    private boolean reserveStarted;

    /**
     * @param name      shown on telemetry
     * @param startPose where the robot is placed, in red coordinates through ctx.pose()
     * @param steps     run one after another
     */
    public AutoRoutine(String name, Function<AutoContext, Pose> startPose, AutoStep... steps) {
        this.name = name;
        this.startPose = startPose;
        this.steps = new Steps.SequenceStep("", Arrays.asList(steps));
    }

    /**
     * The reserve window: when AutoConstants.RESERVE_TIME_LEFT_S are left and the steps
     * have not begun their own park drive (ctx.parkStarted), abandon them and run
     * {@code step} (drive to PARK) instead, so LEAVE and PARK are never at risk.
     */
    public AutoRoutine withReserve(AutoStep step) {
        this.reserve = step;
        return this;
    }

    public String getName() {
        return name;
    }

    public Pose startPose(AutoContext ctx) {
        return startPose.apply(ctx);
    }

    void start(AutoContext ctx) {
        finished = false;
        reserveStarted = false;
        ctx.parkStarted = false;
        steps.start(ctx);
    }

    /** Run the current step. Call once per loop. */
    void update(AutoContext ctx) {
        if (finished) {
            return;
        }
        if (reserveStarted) {
            if (reserve.update(ctx)) {
                reserve.end(ctx);
                finished = true;
            }
            return;
        }
        if (reserve != null && AutoConstants.PARK_ENABLED && !ctx.parkStarted
                && ctx.timeLeft() <= AutoConstants.RESERVE_TIME_LEFT_S) {
            steps.end(ctx);
            ctx.spinUp = false;
            ctx.intakeMode = Intake.Mode.OFF;
            ctx.robot.shootSequence.cancel();
            reserveStarted = true;
            reserve.start(ctx);
            return;
        }
        if (steps.update(ctx)) {
            steps.end(ctx);
            finished = true;
        }
    }

    void stop(AutoContext ctx) {
        if (!finished) {
            if (reserveStarted) {
                reserve.end(ctx);
            } else {
                steps.end(ctx);
            }
        }
    }

    public boolean isFinished() {
        return finished;
    }

    /** The step running now, for telemetry. */
    public String currentStep() {
        return finished ? "done" : reserveStarted ? "RESERVE > " + reserve.name() : steps.name();
    }
}
