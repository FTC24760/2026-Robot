package org.firstinspires.ftc.teamcode.sequences;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.constants.ShootGateConstants;
import org.firstinspires.ftc.teamcode.utils.ShotZoneGeometry;

/**
 * The TeleOp shooting zone and the "ready to fire" signal. Pure decision logic plus the
 * driver feedback (rumble, gamepad LED); it owns no hardware. Robot feeds it the facts each
 * loop (distance, approach angle, aim and wheel readiness, shoot-sequence state).
 *
 * In zone = target distance inside the shot table's measured range AND the shot within
 * ShootGateConstants.ZONE_MAX_OFF_AXIS_DEG of the CELL's opening axis, both with
 * hysteresis. Pre-spin is wanted in the zone unless the stop button latched it off or a
 * shot has just been fed and the robot has not yet reloaded.
 */
public class ShootZone {
    private final Gamepad driver;
    private final Gamepad operator;

    private boolean inZone;
    private boolean readyToFire;
    private boolean wasReadyToFire;
    private boolean ledSet;
    private double lastRumbleS = -1e9;
    // Set by the stop button: no pre-spin until the robot has left the zone.
    private boolean prespinSuppressed;
    // After a shot that fed balls: no pre-spin and not ready until the robot has reloaded
    // (intake ran REARM_INTAKE_S) or left the zone. No ball sensor, so intake time stands in.
    private boolean reloadDisarmed;
    private boolean wasShootActive;
    private boolean fedThisShot;
    private double intakeSinceShotS;

    public ShootZone(Gamepad driver, Gamepad operator) {
        this.driver = driver;
        this.operator = operator;
    }

    public boolean isInZone() {
        return inZone;
    }

    public boolean isReadyToFire() {
        return readyToFire;
    }

    /** True once the driver feedback has started (TeleOp only; Auto never updates it). */
    public boolean isFeedbackShown() {
        return ledSet;
    }

    /** True while disarmed after a shot, waiting for the robot to reload or leave the zone. */
    public boolean isReloading() {
        return reloadDisarmed;
    }

    /** True if the wheel should be kept at the table speed ready for the shoot button. */
    public boolean wantsPrespin() {
        return ShootGateConstants.PRESPIN_IN_ZONE && inZone && !prespinSuppressed
                && !reloadDisarmed;
    }

    /** The stop button: no pre-spin until the robot has left the zone. */
    public void onStopPressed() {
        prespinSuppressed = true;
    }

    /**
     * Work out whether the robot is in the zone and ready to fire, and tell the drive team
     * when it becomes ready. Call after the turret has been aimed this loop.
     *
     * @param distanceIn     distance to the target, NaN with none
     * @param offAxisDeg     degrees the shot is off the CELL's opening axis, NaN with none
     * @param aimReady       Robot.aimReady()
     * @param wheelAtSpeed   Shooter.isReadyToFeed()
     * @param shootActive    a shoot sequence is running (no rumble then)
     */
    public void update(double distanceIn, double offAxisDeg, boolean aimReady,
                       boolean wheelAtSpeed, boolean shootActive, double nowS) {
        // Hysteresis: easier to stay in than to get in.
        inZone = ShotZoneGeometry.inZone(distanceIn, offAxisDeg, inZone);
        if (!inZone) {
            prespinSuppressed = false;
            reloadDisarmed = false;
        }

        boolean wheelOk = !ShootGateConstants.PRESPIN_IN_ZONE || prespinSuppressed
                || wheelAtSpeed;
        readyToFire = inZone && !reloadDisarmed && aimReady && wheelOk;

        // Rumble on becoming ready, but not again within READY_RUMBLE_COOLDOWN_S, so a
        // readiness that flickers (e.g. speed near the limit) does not buzz constantly.
        if (readyToFire && !wasReadyToFire && !shootActive
                && ShootGateConstants.READY_RUMBLE_MS > 0
                && nowS - lastRumbleS >= ShootGateConstants.READY_RUMBLE_COOLDOWN_S) {
            driver.rumble(ShootGateConstants.READY_RUMBLE_MS);
            operator.rumble(ShootGateConstants.READY_RUMBLE_MS);
            lastRumbleS = nowS;
        }
        if (ShootGateConstants.READY_LED && (readyToFire != wasReadyToFire || !ledSet)) {
            double red = readyToFire ? 0 : 1;
            double green = readyToFire ? 1 : 0;
            driver.setLedColor(red, green, 0, Gamepad.LED_DURATION_CONTINUOUS);
            operator.setLedColor(red, green, 0, Gamepad.LED_DURATION_CONTINUOUS);
            ledSet = true;
        }
        wasReadyToFire = readyToFire;
    }

    /**
     * A shot that fed balls disarms the zone until the robot has reloaded: the intake has
     * run REARM_INTAKE_S since the shot (it may pick balls up without leaving the zone), or
     * the robot leaves the zone (update). The stop button has its own latch.
     */
    public void updateReloadArming(boolean shootActive, boolean feeding, boolean stopShot,
                                   boolean intaking, double loopSeconds) {
        fedThisShot |= feeding;
        if (wasShootActive && !shootActive) {
            if (fedThisShot && !stopShot && ShootGateConstants.REARM_INTAKE_S > 0) {
                reloadDisarmed = true;
                intakeSinceShotS = 0;
            }
            fedThisShot = false;
        }
        wasShootActive = shootActive;
        if (reloadDisarmed && !shootActive && intaking) {
            intakeSinceShotS += loopSeconds;
            if (intakeSinceShotS >= ShootGateConstants.REARM_INTAKE_S) {
                reloadDisarmed = false;
            }
        }
    }
}
