package org.firstinspires.ftc.teamcode.utils;

import org.firstinspires.ftc.teamcode.constants.ShotTableConstants;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Wheel speed and hood position for a distance, from the measured rows in
 * ShotTableConstants: linear between rows, the end row's values outside them. Replaces
 * ShotSolver's physics model, which a real flywheel does not follow closely enough.
 */
public final class ShotTable {
    private ShotTable() {
    }

    public static final class Shot {
        public final double distanceIn;
        public final double rpm;
        public final double hoodPosition;
        /** False when the distance is well outside the measured rows (a guess). */
        public final boolean inRange;

        Shot(double distanceIn, double rpm, double hoodPosition, boolean inRange) {
            this.distanceIn = distanceIn;
            this.rpm = rpm;
            this.hoodPosition = hoodPosition;
            this.inRange = inRange;
        }
    }

    private static final class Row implements Comparable<Row> {
        final double distanceIn, rpm, hood;

        Row(double distanceIn, double rpm, double hood) {
            this.distanceIn = distanceIn;
            this.rpm = rpm;
            this.hood = hood;
        }

        @Override
        public int compareTo(Row other) {
            return Double.compare(distanceIn, other.distanceIn);
        }
    }

    /** The shot for this distance, or null if the table is off, empty, or the distance is unknown. */
    public static Shot lookup(double distanceIn) {
        if (!ShotTableConstants.ENABLED || Double.isNaN(distanceIn)) {
            return null;
        }
        // Read every call: rows edited live in Panels take effect at once.
        List<Row> rows = rows();
        if (rows.isEmpty()) {
            return null;
        }
        return interpolate(rows, distanceIn, ShotTableConstants.OUT_OF_RANGE_MARGIN_IN);
    }

    static Shot interpolate(List<Row> rows, double d, double margin) {
        Row first = rows.get(0);
        Row last = rows.get(rows.size() - 1);
        boolean inRange = d >= first.distanceIn - margin && d <= last.distanceIn + margin;

        if (d <= first.distanceIn) {
            return new Shot(d, first.rpm, first.hood, inRange);
        }
        if (d >= last.distanceIn) {
            return new Shot(d, last.rpm, last.hood, inRange);
        }
        for (int i = 1; i < rows.size(); i++) {
            Row hi = rows.get(i);
            if (d <= hi.distanceIn) {
                Row lo = rows.get(i - 1);
                double t = (d - lo.distanceIn) / (hi.distanceIn - lo.distanceIn);
                return new Shot(d,
                        lo.rpm + t * (hi.rpm - lo.rpm),
                        lo.hood + t * (hi.hood - lo.hood),
                        true);
            }
        }
        return new Shot(d, last.rpm, last.hood, inRange); // not reached
    }

    private static List<Row> rows() {
        List<Row> rows = new ArrayList<>(6);
        add(rows, ShotTableConstants.DIST_IN_1, ShotTableConstants.RPM_1, ShotTableConstants.HOOD_1);
        add(rows, ShotTableConstants.DIST_IN_2, ShotTableConstants.RPM_2, ShotTableConstants.HOOD_2);
        add(rows, ShotTableConstants.DIST_IN_3, ShotTableConstants.RPM_3, ShotTableConstants.HOOD_3);
        add(rows, ShotTableConstants.DIST_IN_4, ShotTableConstants.RPM_4, ShotTableConstants.HOOD_4);
        add(rows, ShotTableConstants.DIST_IN_5, ShotTableConstants.RPM_5, ShotTableConstants.HOOD_5);
        add(rows, ShotTableConstants.DIST_IN_6, ShotTableConstants.RPM_6, ShotTableConstants.HOOD_6);
        Collections.sort(rows);
        // Two rows at the same distance would divide by zero; keep the first.
        for (int i = rows.size() - 1; i > 0; i--) {
            if (rows.get(i).distanceIn == rows.get(i - 1).distanceIn) {
                rows.remove(i);
            }
        }
        return rows;
    }

    private static void add(List<Row> rows, double distanceIn, double rpm, double hood) {
        if (distanceIn > 0 && rpm > 0) {
            rows.add(new Row(distanceIn, rpm, hood));
        }
    }
}
