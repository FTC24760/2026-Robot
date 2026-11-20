package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;

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

    public String getName() {
        return name;
    }

    public Pose startPose(AutoContext ctx) {
        return startPose.apply(ctx);
    }

    void start(AutoContext ctx) {
        finished = false;
        steps.start(ctx);
    }

    /** Run the current step. Call once per loop. */
    void update(AutoContext ctx) {
        if (!finished && steps.update(ctx)) {
            steps.end(ctx);
            finished = true;
        }
    }

    void stop(AutoContext ctx) {
        if (!finished) {
            steps.end(ctx);
        }
    }

    public boolean isFinished() {
        return finished;
    }

    /** The step running now, for telemetry. */
    public String currentStep() {
        return finished ? "done" : steps.name();
    }
}
