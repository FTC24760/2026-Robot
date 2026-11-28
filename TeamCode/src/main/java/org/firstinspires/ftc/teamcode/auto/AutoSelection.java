package org.firstinspires.ftc.teamcode.auto;

import org.firstinspires.ftc.teamcode.constants.AutoConstants;
import org.firstinspires.ftc.teamcode.utils.Alliance;
import org.firstinspires.ftc.teamcode.utils.CellSide;
import org.firstinspires.ftc.teamcode.utils.HiveSide;

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
        /** Partner runs no auto, or only parks: we have the whole HIVE (start A). */
        SOLO,
        /**
         * Partner shoots too. Each robot takes care of its own zone only: we shoot only our
         * own CELL (start A: Zone A, start B: Zone B), collect only our side's sources, hold
         * fire while our CELL is down, and never drive into the partner's zone. Start picked
         * by hand.
         */
        PAIR;

        public Slot next() {
            Slot[] all = values();
            return all[(ordinal() + 1) % all.length];
        }
    }

    public static StartPreset preset = StartPreset.A;
    public static Slot slot = Slot.SOLO;
    public static int cycles = AutoConstants.DEFAULT_CYCLES;

    /** The start that goes with the slot: SOLO at A; PAIR either (the driver picks it). */
    public static StartPreset presetForSlot() {
        return slot == Slot.SOLO ? StartPreset.A : preset;
    }

    /** PAIR: the partner shoots into the same HIVE, and we keep to our own zone. */
    public static boolean pair() {
        return slot == Slot.PAIR;
    }

    /**
     * The CELL nearest this start, which a robot in a pair shoots into: preset A (audience
     * wall) the AUDIENCE CELL, B (far wall) the FAR CELL.
     */
    public static CellSide ownSide(Alliance alliance) {
        return CellSide.of(alliance, preset == StartPreset.A ? HiveSide.AUDIENCE : HiveSide.FAR);
    }
}
