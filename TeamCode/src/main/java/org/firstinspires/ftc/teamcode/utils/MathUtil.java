package org.firstinspires.ftc.teamcode.utils;

public final class MathUtil {
    private MathUtil() {
    }

    public static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    public static double rpmToTicksPerSecond(double rpm, double ticksPerRev) {
        return rpm * ticksPerRev / 60.0;
    }

    public static double ticksPerSecondToRpm(double ticksPerSecond, double ticksPerRev) {
        return ticksPerSecond * 60.0 / ticksPerRev;
    }

    public static long msToNs(double ms) {
        return (long) (ms * 1_000_000.0);
    }
}
