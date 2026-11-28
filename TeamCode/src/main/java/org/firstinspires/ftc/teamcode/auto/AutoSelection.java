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
        SECOND,
        /**
         * Both robots strong, goal three tips: each shoots only its own CELL (start A: Zone A,
         * start B: Zone B), collects only its side's sources, and holds fire while its CELL
         * is down. Start chosen by hand; at B it waits the delay like SECOND.
         */
        EQUAL;

        public Slot next() {
            Slot[] all = values();
            return all[(ordinal() + 1) % all.length];
        }
    }

    /**
     * FIRST only: where the partner starts, as its team told us before the match. We do not
     * know the partner's auto, so this is all we have to stay out of its way.
     */
    public enum PartnerStart {
        /** Nobody asked: FIRST shoots only Zone A and never drives into Zone B. */
        UNKNOWN,
        /** At start B (far wall): FIRST may use Zone B, PARTNER_CLEAR_IN from there. */
        FAR_WALL,
        /** Along our alliance wall: FIRST may use Zone B, PARTNER_CLEAR_IN from that wall. */
        ALLIANCE_WALL;

        public PartnerStart next() {
            PartnerStart[] all = values();
            return all[(ordinal() + 1) % all.length];
        }
    }

    public static StartPreset preset = StartPreset.A;
    /** FIRST only. Default UNKNOWN, so a robot whose drive team did not ask plays safe. */
    public static PartnerStart partnerStart = PartnerStart.UNKNOWN;
    public static Slot slot = Slot.SOLO;
    /** Only used in the SECOND slot. */
    public static double startDelayS = AutoConstants.DEFAULT_START_DELAY_S;
    public static int cycles = AutoConstants.DEFAULT_CYCLES;

    /**
     * The start that goes with the slot: SOLO and FIRST (the stronger robot) at A, SECOND
     * at B, EQUAL either. The driver can still change it by hand (the init screen then warns).
     */
    public static StartPreset presetForSlot() {
        switch (slot) {
            case SECOND:
                return StartPreset.B;
            case EQUAL:
                return preset; // either start; the driver picks it
            default:
                return StartPreset.A;
        }
    }

    /**
     * We let the partner shoot first: SECOND, or EQUAL from start B (our CELL is down at the
     * start, so the partner's pre-loads must tip it first).
     */
    public static boolean shootsAfterPartner() {
        return slot == Slot.SECOND || (slot == Slot.EQUAL && preset == StartPreset.B);
    }

    /** We only ever shoot our own CELL (the one nearest our start) and hold fire while it is down. */
    public static boolean ownCellOnly() {
        return slot == Slot.SECOND || slot == Slot.EQUAL
                || (slot == Slot.FIRST && partnerStart == PartnerStart.UNKNOWN);
    }

    /** FIRST, SECOND or EQUAL: the partner shoots into the same HIVE. */
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
        return shootsAfterPartner() ? startDelayS : 0;
    }
}
