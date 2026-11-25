package org.firstinspires.ftc.teamcode.sequences;

import org.firstinspires.ftc.teamcode.constants.Configurables;
import org.firstinspires.ftc.teamcode.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

/**
 * One-click shooting, shared by teleop and autonomous:
 *
 *   1. spin the wheel up to shooting speed (with BLOCKER_OPEN_EARLY, the slow blocker
 *      starts opening now, so its travel overlaps spin-up),
 *   2. once RPM is stable, open the blocker if it is not already open,
 *   3. once the gate has had BLOCKER_OPEN_SETTLE_S to open, feed until stopped,
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
        OPENING_GATE,
        FEEDING
    }

    private final Shooter shooter;
    private final Intake intake;

    private Stage stage = Stage.IDLE;
    private double stageStartS;
    private double readySinceS = -1;
    private double blockerS = -1;

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
                        cancel();
                        start(nowS);
                    }
                } else if (nowS - blockerS >= ShooterConstants.BLOCKER_OPEN_SETTLE_S) {
                    enter(Stage.FEEDING, nowS);
                }
                break;

            case FEEDING:
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
