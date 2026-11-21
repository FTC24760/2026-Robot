package org.firstinspires.ftc.teamcode.sequences;

import org.firstinspires.ftc.teamcode.constants.Configurables;
import org.firstinspires.ftc.teamcode.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

/**
 * One-click shooting, shared by teleop and autonomous:
 *
 *   1. spin the wheel up to shooting speed,
 *   2. once RPM is stable, open the blocker,
 *   3. wait for the gate to open, then feed until stopped or BURST_FEED_S passes,
 *   4. close the blocker.
 *
 * This class only decides what should happen. Each loop the caller asks
 * wantsShooter() and isFeeding() and passes them to Robot.runShooter() and
 * Robot.runIntake(), which do the driving (including feed gating between balls).
 */
public class AutoShootSequence {

    public enum Stage {
        IDLE,
        SPINNING_UP,
        OPENING_GATE,
        FEEDING
    }

    private final Shooter shooter;
    private final Intake intake;

    private Stage stage = Stage.IDLE;
    private double stageStartS;
    private double readySinceS = -1;

    public AutoShootSequence(Shooter shooter, Intake intake) {
        this.shooter = shooter;
        this.intake = intake;
    }

    /** Begin a burst. Does nothing if one is already running. */
    public void start(double nowS) {
        if (stage != Stage.IDLE) {
            return;
        }
        intake.setBlockerOpen(false);
        readySinceS = -1;
        enter(Stage.SPINNING_UP, nowS);
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
                if (nowS - stageStartS >= ShooterConstants.BURST_SPIN_UP_TIMEOUT_S) {
                    cancel();
                } else if (wheelSettled(nowS)) {
                    intake.setBlockerOpen(true);
                    enter(Stage.OPENING_GATE, nowS);
                }
                break;

            case OPENING_GATE:
                if (!shooter.isReadyToFeed()) {
                    cancel();
                    start(nowS);
                } else if (nowS - stageStartS >= ShooterConstants.BLOCKER_OPEN_SETTLE_S) {
                    enter(Stage.FEEDING, nowS);
                }
                break;

            case FEEDING:
                if (nowS - stageStartS >= Configurables.SHOOT_FEED_SECONDS) {
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

    /** True while feeding is active and the current wheel speed permits transfer. */
    public boolean isFeeding() {
        return stage == Stage.FEEDING && shooter.isReadyToFeed();
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