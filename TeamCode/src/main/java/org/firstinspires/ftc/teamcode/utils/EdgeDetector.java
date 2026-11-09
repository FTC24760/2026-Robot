package org.firstinspires.ftc.teamcode.utils;

/** Turns a held button into a single "just pressed" event. Call {@link #rose} once per loop. */
public class EdgeDetector {
    private boolean last;

    /** @return true only on the loop where the input goes from false to true. */
    public boolean rose(boolean current) {
        boolean rose = current && !last;
        last = current;
        return rose;
    }
}
