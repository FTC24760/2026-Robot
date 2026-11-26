package org.firstinspires.ftc.teamcode.auto;

import org.firstinspires.ftc.teamcode.constants.AutoConstants;

/**
 * What the drive team chose on the init menu. Static, so the choices survive from one
 * match to the next while the Robot Controller app keeps running; they reset when the
 * app restarts or new code is deployed.
 */
public final class AutoSelection {
    private AutoSelection() {
    }

    /** Our slot, as agreed with the alliance partner before the match. */
    public enum Slot {
        /** Partner runs no auto, or only parks: shoot at once. */
        SOLO,
        /** Partner shoots too, and we agreed to go first: shoot at once. */
        FIRST,
        /** Partner shoots too, and goes first: wait startDelayS before shooting. */
        SECOND;

        public Slot next() {
            Slot[] all = values();
            return all[(ordinal() + 1) % all.length];
        }
    }

    /**
     * What we and the partner agreed happens if the FIRST robot's pre-loads do not tip the
     * HIVE. Both robots must pick the same one.
     */
    public enum NoTipPlan {
        /** FIRST parks; SECOND drives to the up CELL and shoots when its delay ends. */
        LEAVE,
        /** FIRST keeps cycling until the HIVE tips; SECOND waits for the tip. */
        STAY;

        public NoTipPlan next() {
            return this == LEAVE ? STAY : LEAVE;
        }
    }

    public static StartPreset preset = StartPreset.A;
    public static Slot slot = Slot.SOLO;
    /** Only used in the SECOND slot. */
    public static double startDelayS = AutoConstants.DEFAULT_START_DELAY_S;
    public static int cycles = AutoConstants.DEFAULT_CYCLES;
    /** Only used in the FIRST and SECOND slots. */
    public static NoTipPlan noTipPlan = AutoConstants.DEFAULT_NO_TIP_PLAN;

    /** FIRST, after the pre-loads, and the HIVE did not tip: keep cycling? */
    public static boolean firstStaysUntilTip() {
        return slot == Slot.FIRST && noTipPlan == NoTipPlan.STAY;
    }

    /** SECOND, delay over, no tip yet: keep waiting for the partner to tip it? */
    public static boolean secondWaitsForTip() {
        return slot == Slot.SECOND && noTipPlan == NoTipPlan.STAY;
    }

    /** How long to wait before the pre-loads, given the slot. */
    public static double effectiveDelayS() {
        return slot == Slot.SECOND ? startDelayS : 0;
    }
}
