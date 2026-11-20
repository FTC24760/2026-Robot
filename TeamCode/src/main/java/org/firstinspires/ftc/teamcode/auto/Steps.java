package org.firstinspires.ftc.teamcode.auto;

import static com.pedropathing.api.Paths.line;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.constants.AutoConstants;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.utils.CellSide;

import java.util.Arrays;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Predicate;

/**
 * Ready-made steps. A routine is a list of these:
 *
 *   spinUp(true),
 *   driveTo("to shot", ctx -> ctx.pose(48, 96, 135)),
 *   shoot(),
 *   intake(Intake.Mode.INTAKE),
 *   follow("collect", (ctx, from) -> curve(from, ctx.pose(30, 60, 0), ctx.pose(20, 40, 180))
 *           .linear(from.heading(), Math.toRadians(180))),
 *   ...
 */
public final class Steps {
    private Steps() {
    }

    /** Builds a path at the moment its step starts, from wherever the robot is then. */
    public interface PathFactory {
        Path build(AutoContext ctx, Pose from);
    }

    // ---- Driving ----

    /** Drive a path. The path is built when the step starts, so it begins at the robot. */
    public static AutoStep follow(String name, PathFactory path) {
        return follow(name, path, AutoConstants.DRIVE_TIMEOUT_S);
    }

    public static AutoStep follow(String name, PathFactory path, double timeoutS) {
        return new FollowStep(name, path, timeoutS);
    }

    /** Drive straight to a pose, turning steadily to its heading on the way. */
    public static AutoStep driveTo(String name, Function<AutoContext, Pose> target) {
        return follow(name, (ctx, from) -> {
            Pose to = target.apply(ctx);
            return line(from, to).linear(from.heading(), to.heading());
        });
    }

    // ---- Shooting ----

    /** Spin up, open the blocker once at speed, feed for SHOOT_FEED_S, close. */
    public static AutoStep shoot() {
        return shoot(AutoConstants.SHOOT_FEED_S);
    }

    public static AutoStep shoot(double feedS) {
        return new ShootStep(feedS);
    }

    // ---- FLOWERS ----

    /**
     * Collect from a FLOWER: drive up to it, lower the arm, run the intake, shift side to
     * side for FLOWER_COLLECT_S, then raise the arm. The intake is left running.
     *
     * @param approach where the robot stops, facing the FLOWER
     */
    public static AutoStep collectFromFlower(String name, Function<AutoContext, Pose> approach) {
        return sequence(name,
                intake(Intake.Mode.INTAKE),
                driveTo("to " + name, approach),
                flowerArm(true),
                shiftSideToSide(AutoConstants.FLOWER_COLLECT_S),
                flowerArm(false));
    }

    /** Shift side to side in place (see IntakeConstants.FLOWER_SHIFT_IN). */
    public static AutoStep shiftSideToSide(double seconds) {
        return new ShiftStep(seconds);
    }

    // ---- Background jobs (finish at once; the job keeps running) ----

    /** Hold the shooter at speed in the background, e.g. while driving to a shot. */
    public static AutoStep spinUp(boolean on) {
        return instant("spin up " + (on ? "on" : "off"), ctx -> ctx.spinUp = on);
    }

    public static AutoStep intake(Intake.Mode mode) {
        return instant("intake " + mode, ctx -> ctx.intakeMode = mode);
    }

    public static AutoStep flowerArm(boolean down) {
        return instant("flower arm " + (down ? "down" : "up"), ctx -> ctx.flowerDown = down);
    }

    // ---- Timing and choices ----

    public static AutoStep waitSeconds(double seconds) {
        return new WaitStep(seconds);
    }

    /** Run some steps in order, as one step. */
    public static AutoStep sequence(String name, AutoStep... steps) {
        return new SequenceStep(name, Arrays.asList(steps));
    }

    /** Run {@code steps} only if at least {@code seconds} of the auto are left. */
    public static AutoStep ifTimeLeft(double seconds, AutoStep... steps) {
        return when("if " + seconds + " s left", ctx -> ctx.timeLeft() >= seconds,
                sequence("", steps), null);
    }

    /**
     * Do one thing if our LEFT CELL is up and another if our RIGHT one is (as our drivers
     * see it). Decided when this step starts, from the latest Limelight view, the
     * driver's choice, or the start default.
     */
    public static AutoStep byUpSide(AutoStep leftUp, AutoStep rightUp) {
        return when("by up CELL", ctx -> ctx.upSide == CellSide.LEFT, leftUp, rightUp);
    }

    /** Choose between two steps when this step starts. Either may be null (skip). */
    public static AutoStep when(String name, Predicate<AutoContext> test, AutoStep ifTrue,
                                AutoStep ifFalse) {
        return new ChoiceStep(name, test, ifTrue, ifFalse);
    }

    /** A step that does something once and finishes immediately. */
    public static AutoStep instant(String name, Consumer<AutoContext> action) {
        return new AutoStep() {
            @Override
            public void start(AutoContext ctx) {
                action.accept(ctx);
            }

            @Override
            public boolean update(AutoContext ctx) {
                return true;
            }

            @Override
            public String name() {
                return name;
            }
        };
    }

    // ---- Implementations ----

    private static final class FollowStep implements AutoStep {
        private final String name;
        private final PathFactory path;
        private final double timeoutS;
        private double startS;

        FollowStep(String name, PathFactory path, double timeoutS) {
            this.name = name;
            this.path = path;
            this.timeoutS = timeoutS;
        }

        @Override
        public void start(AutoContext ctx) {
            startS = ctx.now();
            ctx.robot.drivetrain.follow(path.build(ctx, ctx.robot.drivetrain.getPose()));
        }

        @Override
        public boolean update(AutoContext ctx) {
            return !ctx.robot.drivetrain.isBusy() || ctx.now() - startS > timeoutS;
        }

        @Override
        public String name() {
            return name;
        }
    }

    /**
     * Shoots for feedS seconds of actual feeding. If the HIVE tips part-way (the up CELL
     * changes), feeding stops at once so no ball goes into the CELL that is now facing
     * down, the turret swings to the new up CELL, and feeding resumes for the time left.
     */
    private static final class ShootStep implements AutoStep {
        private final double feedS;
        private double startS;
        private double fedS;
        private double lastS;
        private double pausedUntilS;
        private CellSide aimedAt;

        ShootStep(double feedS) {
            this.feedS = feedS;
        }

        @Override
        public void start(AutoContext ctx) {
            startS = ctx.now();
            lastS = startS;
            fedS = 0;
            pausedUntilS = -1;
            aimedAt = ctx.upSide;
            ctx.robot.shootSequence.start(ctx.now());
        }

        @Override
        public boolean update(AutoContext ctx) {
            double now = ctx.now();
            double dt = now - lastS;
            lastS = now;

            // The HIVE tipped: stop feeding and retarget.
            if (ctx.upSide != aimedAt) {
                aimedAt = ctx.upSide;
                ctx.robot.shootSequence.cancel();
                pausedUntilS = now + AutoConstants.SHOOT_RETARGET_PAUSE_S;
            }
            if (pausedUntilS >= 0) {
                if (now < pausedUntilS) {
                    return false;
                }
                pausedUntilS = -1;
                ctx.robot.shootSequence.start(now);
            }

            if (ctx.robot.shootSequence.isFeeding()) {
                fedS += dt;
                return fedS >= feedS;
            }
            // Still spinning up: give up rather than burn the auto.
            return now - startS > AutoConstants.SHOOT_TIMEOUT_S + AutoConstants.SHOOT_RETARGET_PAUSE_S;
        }

        @Override
        public void end(AutoContext ctx) {
            ctx.robot.shootSequence.cancel();
            if (fedS > 0) {
                ctx.recordShot(AutoConstants.BALLS_PER_SHOT);
            }
        }

        @Override
        public String name() {
            return "shoot";
        }
    }

    private static final class ShiftStep implements AutoStep {
        private final double seconds;
        private double startS;
        private Pose center;

        ShiftStep(double seconds) {
            this.seconds = seconds;
        }

        @Override
        public void start(AutoContext ctx) {
            startS = ctx.now();
            center = ctx.robot.drivetrain.getPose();
        }

        @Override
        public boolean update(AutoContext ctx) {
            double elapsed = ctx.now() - startS;
            ctx.robot.drivetrain.shiftAround(center, IntakeConstants.FLOWER_SHIFT_IN,
                    IntakeConstants.FLOWER_SHIFT_PERIOD_S, elapsed);
            return elapsed >= seconds;
        }

        @Override
        public void end(AutoContext ctx) {
            ctx.robot.drivetrain.endShift();
            ctx.robot.drivetrain.hold(center);
        }

        @Override
        public String name() {
            return "shift side to side";
        }
    }

    private static final class WaitStep implements AutoStep {
        private final double seconds;
        private double startS;

        WaitStep(double seconds) {
            this.seconds = seconds;
        }

        @Override
        public void start(AutoContext ctx) {
            startS = ctx.now();
        }

        @Override
        public boolean update(AutoContext ctx) {
            return ctx.now() - startS >= seconds;
        }

        @Override
        public String name() {
            return "wait " + seconds + " s";
        }
    }

    /** Runs a list of steps in order. Also what a whole routine is. */
    static class SequenceStep implements AutoStep {
        private final String name;
        private final List<AutoStep> steps;
        private int index;
        private boolean started;

        SequenceStep(String name, List<AutoStep> steps) {
            this.name = name;
            this.steps = steps;
        }

        @Override
        public void start(AutoContext ctx) {
            index = 0;
            started = false;
        }

        @Override
        public boolean update(AutoContext ctx) {
            // Several instant steps may finish in the same loop.
            while (index < steps.size()) {
                AutoStep step = steps.get(index);
                if (!started) {
                    step.start(ctx);
                    started = true;
                }
                if (!step.update(ctx)) {
                    return false;
                }
                step.end(ctx);
                index++;
                started = false;
            }
            return true;
        }

        @Override
        public void end(AutoContext ctx) {
            if (started && index < steps.size()) {
                steps.get(index).end(ctx);
            }
        }

        @Override
        public String name() {
            if (index < steps.size() && started) {
                String inner = steps.get(index).name();
                return name.isEmpty() ? inner : name + " > " + inner;
            }
            return name;
        }
    }

    private static final class ChoiceStep implements AutoStep {
        private final String name;
        private final Predicate<AutoContext> test;
        private final AutoStep ifTrue;
        private final AutoStep ifFalse;
        private AutoStep chosen;

        ChoiceStep(String name, Predicate<AutoContext> test, AutoStep ifTrue, AutoStep ifFalse) {
            this.name = name;
            this.test = test;
            this.ifTrue = ifTrue;
            this.ifFalse = ifFalse;
        }

        @Override
        public void start(AutoContext ctx) {
            chosen = test.test(ctx) ? ifTrue : ifFalse;
            if (chosen != null) {
                chosen.start(ctx);
            }
        }

        @Override
        public boolean update(AutoContext ctx) {
            return chosen == null || chosen.update(ctx);
        }

        @Override
        public void end(AutoContext ctx) {
            if (chosen != null) {
                chosen.end(ctx);
            }
        }

        @Override
        public String name() {
            return chosen == null ? name + " (skipped)" : chosen.name();
        }
    }
}
