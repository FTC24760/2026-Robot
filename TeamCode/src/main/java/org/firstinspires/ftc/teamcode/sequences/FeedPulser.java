package org.firstinspires.ftc.teamcode.sequences;

/**
 * Feeds the shooter one ball at a time at full transfer power.
 *
 * The transfer runs in short pulses. A pulse starts only when the caller allows it,
 * then always runs to its full length, so a noisy speed reading cannot cut it short.
 * After each pulse the transfer rests for at least the off time. Each pulse should be
 * just long enough to push one ball into the wheel.
 *
 * The caller decides the style through what it passes in:
 *   staggered - allow a pulse only while the wheel reads ready (wait for each recovery)
 *   rapid     - allow every pulse after the first (a fixed beat, no waiting)
 *
 * Compared with running the transfer slowly, every ball (including the last one, which
 * has no ball behind it) gets the transfer's full push.
 */
public class FeedPulser {
    private boolean pulsing;
    private double pulseStartS;
    private double pulseEndS = Double.NEGATIVE_INFINITY;
    private int pulseCount;

    /**
     * Call once per loop while the driver or sequence wants to feed.
     *
     * @param nowS     loop timestamp in seconds
     * @param mayStart true if a new pulse is allowed to begin now
     * @param onMs     length of one pulse
     * @param minOffMs shortest rest between pulses
     * @return true if the transfer should run this loop
     */
    public boolean update(double nowS, boolean mayStart, double onMs, double minOffMs) {
        if (pulsing && nowS - pulseStartS >= onMs / 1000.0) {
            pulsing = false;
            pulseEndS = nowS;
        }

        boolean rested = nowS - pulseEndS >= minOffMs / 1000.0;
        if (!pulsing && mayStart && rested) {
            pulsing = true;
            pulseStartS = nowS;
            pulseCount++;
        }

        return pulsing;
    }

    /** Call when feeding stops, so the next burst starts with a fresh pulse. */
    public void reset() {
        pulsing = false;
        pulseEndS = Double.NEGATIVE_INFINITY;
        pulseCount = 0;
    }

    /** True while a pulse is running. */
    public boolean isPulsing() {
        return pulsing;
    }

    /** Pulses started since the last reset: roughly the number of balls fed. */
    public int getPulseCount() {
        return pulseCount;
    }
}
