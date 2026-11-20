package org.firstinspires.ftc.teamcode.auto;

/**
 * One thing an autonomous routine does: drive a path, shoot, wait, and so on.
 * Steps run one after another; see {@link Steps} for the ready-made ones.
 */
public interface AutoStep {
    /** Called once, on the loop the step begins. */
    void start(AutoContext ctx);

    /** Called every loop until it returns true, meaning the step is finished. */
    boolean update(AutoContext ctx);

    /** Called once when the step finishes, or when the OpMode stops during it. */
    default void end(AutoContext ctx) {
    }

    /** Shown on telemetry while the step runs. */
    default String name() {
        return getClass().getSimpleName();
    }
}
