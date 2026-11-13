package org.firstinspires.ftc.teamcode.sequences;

import org.firstinspires.ftc.teamcode.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

/**
 * One-click shooting, shared by teleop and autonomous:
 *
 *   1. spin the wheel up to shooting speed,
 *   2. once it has held that speed for a moment, open the blocker,
 *   3. feed until stopped or until BURST_FEED_S has passed,
 *   4. close the blocker.
 *
 * This class only decides what should happen. Each loop the caller asks
 * wantsShooter() and isFeeding() and passes them to Robot.runShooter() and
 * Robot.runIntake(), which do the driving (including feed gating between balls).
 */
public class ShootSequence {

    public enum Stage {
        IDLE,
        SPINNING_UP,
        FEEDING
    }

    private final Shooter shooter;
    private final Intake intake;

    private Stage stage = Stage.IDLE;
    private double stageStartS;
    private double readySinceS = -1;

    public ShootSequence(Shooter shooter, Intake intake) {
        this.shooter = shooter;
        this.intake = intake;
    }

    /** Begin a burst. Does nothing if one is already running. */
    public void start(double nowS) {
        if (stage != Stage.IDLE) {
            return;
        }
        readySinceS = -1;
        enter(Stage.SPINNING_UP, nowS);
    }

    /** Stop now and close the blocker. Safe to call at any time. */
    public void cancel() {
        if (stage == Stage.FEEDING) {
            intake.setBlockerOpen(false);
        }
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

    /** Advance the sequence. Call once per loop, before Robot.runShooter(). */
    public void update(double nowS) {
        switch (stage) {
            case SPINNING_UP:
                boolean timedOut = nowS - stageStartS > ShooterConstants.BURST_SPIN_UP_TIMEOUT_S;
                if (wheelSettled(nowS) || timedOut) {
                    intake.setBlockerOpen(true);
                    enter(Stage.FEEDING, nowS);
                }
                break;

            case FEEDING:
                if (nowS - stageStartS > ShooterConstants.BURST_FEED_S) {
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

    /** True while the wheel should be held at shooting speed. */
    public boolean wantsShooter() {
        return stage != Stage.IDLE;
    }

    /** True while balls should be pushed into the wheel. */
    public boolean isFeeding() {
        return stage == Stage.FEEDING;
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

    private void enter(Stage next, double nowS) {
        stage = next;
        stageStartS = nowS;
    }
}
