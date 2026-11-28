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

    public static StartPreset preset = StartPreset.A;
    public static Slot slot = Slot.SOLO;
    /** Only used in the SECOND slot. */
    public static double startDelayS = AutoConstants.DEFAULT_START_DELAY_S;
    public static int cycles = AutoConstants.DEFAULT_CYCLES;

    /**
     * The start that goes with the slot: SOLO and FIRST (the stronger robot) at A, SECOND
     * at B. The driver can still change it by hand (the init screen then warns).
     */
    public static StartPreset presetForSlot() {
        return slot == Slot.SECOND ? StartPreset.B : StartPreset.A;
    }

    /** FIRST or SECOND: the partner shoots into the same HIVE. */
    public static boolean pair() {
        return slot != Slot.SOLO;
    }

    /**
     * The CELL nearest this start, which a robot in a pair shoots into: preset A (audience
     * wall) the AUDIENCE CELL, B (far wall) the FAR CELL.
     */
    public static CellSide ownSide(Alliance alliance) {
        return CellSide.of(alliance, preset == StartPreset.A ? HiveSide.AUDIENCE : HiveSide.FAR);
    }

    /** How long to wait before the pre-loads, given the slot. */
    public static double effectiveDelayS() {
        return slot == Slot.SECOND ? startDelayS : 0;
    }
}
