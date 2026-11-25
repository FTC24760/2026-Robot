package org.firstinspires.ftc.teamcode.sequences;

import org.firstinspires.ftc.teamcode.constants.Configurables;
import org.firstinspires.ftc.teamcode.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import java.util.function.BooleanSupplier;

/**
 * Shooting, the one sequence used by both TeleOp and Auto:
 *
 *   1. spin the wheel up to shooting speed (with BLOCKER_OPEN_EARLY, the slow blocker
 *      starts opening now, so its travel overlaps spin-up),
 *   2. once RPM is stable, open the blocker if it is not already open,
 *   3. once the gate has had BLOCKER_OPEN_SETTLE_S to open, feed until stopped, or until
 *      the burst's feed limit if one was given (Auto),
 *   4. close the blocker.
 *
 * Balls are only fed while the fire gate is open: wheel at speed AND aim ready (turret
 * settled on the target, robot slow enough; Robot.aimReady()). If either drops mid-burst,
 * feeding pauses and resumes by itself.
 *
 * This class only decides what should happen. Each loop the caller asks
 * wantsShooter() and isFeeding() and passes them to Robot.runShooter() and
 * Robot.runIntake(), which do the driving.
 */
public class ShootSequence {

    public enum Stage {
        IDLE,
        SPINNING_UP,
        OPENING_GATE,
        FEEDING
    }

    private final Shooter shooter;
    private final Intake intake;
    private final BooleanSupplier aimReady;

    private Stage stage = Stage.IDLE;
    private double stageStartS;
    private double readySinceS = -1;
    private double blockerS = -1;
    // Stop by itself after this long in FEEDING; NaN = only when told to.
    private double maxFeedS = Double.NaN;

    /**
     * @param aimReady true when the aim allows a shot right now (see Robot.aimReady());
     *                 checked before every ball, together with the wheel speed
     */
    public ShootSequence(Shooter shooter, Intake intake, BooleanSupplier aimReady) {
        this.shooter = shooter;
        this.intake = intake;
        this.aimReady = aimReady;
    }

    /** Begin a burst that feeds until cancelled. Does nothing if one is already running. */
    public void start(double nowS) {
        start(nowS, Double.NaN);
    }

    /**
     * Begin a burst that stops by itself after feeding for maxFeedS (a backstop for Auto,
     * whose shoot step normally ends it sooner). Does nothing if one is already running.
     */
    public void start(double nowS, double maxFeedS) {
        if (stage != Stage.IDLE) {
            return;
        }
        this.maxFeedS = maxFeedS;
        readySinceS = -1;
        enter(Stage.SPINNING_UP, nowS);
        if (Configurables.BLOCKER_OPEN_EARLY) {
            openBlocker(nowS);
        } else {
            intake.setBlockerOpen(false);
        }
    }

    /** Stop now and close the blocker. Safe to call at any time. */
    public void cancel() {
        intake.setBlockerOpen(false);
        stage = Stage.IDLE;
    }

    /** The shoot button: start a burst, or stop the one in progress. */
    public void toggle(double nowS) {
        if (stage == Stage.IDLE) {
            start(nowS);
        } else {
            cancel();
        }
    }

    /** Advance the sequence. Call once per loop, after Robot.runShooter(). */
    public void update(double nowS) {
        switch (stage) {
            case SPINNING_UP:
                if (wheelSettled(nowS)) {
                    openBlocker(nowS);
                    enter(Stage.OPENING_GATE, nowS);
                }
                break;

            case OPENING_GATE:
                if (!shooter.isReadyToFeed()) {
                    if (Configurables.BLOCKER_OPEN_EARLY) {
                        // Wait for the wheel again, gate left open (transfer is off).
                        readySinceS = -1;
                        enter(Stage.SPINNING_UP, nowS);
                    } else {
                        double limit = maxFeedS;
                        cancel();
                        start(nowS, limit);
                    }
                } else if (nowS - blockerS >= ShooterConstants.BLOCKER_OPEN_SETTLE_S) {
                    enter(Stage.FEEDING, nowS);
                }
                break;

            case FEEDING:
                if (!Double.isNaN(maxFeedS) && nowS - stageStartS >= maxFeedS) {
                    cancel();
                }
                break;

            case IDLE:
            default:
                break;
        }
    }

    public Stage getStage() {
        return stage;
    }

    public boolean isActive() {
        return stage != Stage.IDLE;
    }

    /** True in the feeding stage, whether or not the fire gate lets a ball go right now. */
    public boolean inFeedingStage() {
        return stage == Stage.FEEDING;
    }

    /** True while the wheel should be held at shooting speed. */
    public boolean wantsShooter() {
        return stage != Stage.IDLE;
    }

    /** True while balls should be pushed into the wheel: feeding, wheel ready, aim ready. */
    public boolean isFeeding() {
        return stage == Stage.FEEDING && shooter.isReadyToFeed() && aimReady.getAsBoolean();
    }

    // The wheel has to stay at speed for a moment, not just touch it once.
    private boolean wheelSettled(double nowS) {
        if (!shooter.isReadyToFeed()) {
            readySinceS = -1;
            return false;
        }
        if (readySinceS < 0) {
            readySinceS = nowS;
        }
        return nowS - readySinceS >= ShooterConstants.BURST_READY_SETTLE_S;
    }

    // Open the blocker, starting its travel clock only if it was not already open.
    private void openBlocker(double nowS) {
        if (!intake.isBlockerOpen()) {
            intake.setBlockerOpen(true);
            blockerS = nowS;
        }
    }

    private void enter(Stage next, double nowS) {
        stage = next;
        stageStartS = nowS;
    }
}
