package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.CameraPanConstants;
import org.firstinspires.ftc.teamcode.constants.VisionConstants;
import org.firstinspires.ftc.teamcode.utils.MathUtil;

/**
 * Turns the Limelight about a vertical axis to keep it facing a point on the field (our
 * HIVE) from the odometry pose. The servo has no feedback, so the angle is the commanded
 * one, and {@link #angleAtNs} gives the angle at the time a camera frame was captured
 * (with a lag) for Vision to apply.
 *
 * The pan angle is relative to VisionConstants.CAMERA_YAW_DEG: 0 means the camera looks
 * where the fixed mount did.
 */
public class CameraPan implements Subsystem {
    private static final int HISTORY = 128;

    private final Servo servo;
    private final long[] historyNs = new long[HISTORY];
    private final double[] historyDeg = new double[HISTORY];
    private int historyCount;
    private int historyNext;

    private double angleDeg;      // commanded pan angle
    private double wantedDeg;     // last wanted angle, for telemetry
    private boolean reachable = true;
    private long lastNs;

    public CameraPan(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, CameraPanConstants.SERVO_NAME);
    }

    private static double halfRangeDeg() {
        return CameraPanConstants.DEGREES_PER_FULL_TRAVEL / 2;
    }

    /**
     * Point the camera at a field point. Call once per loop.
     *
     * @param pose     current odometry pose (inches, radians)
     * @param velocity odometry velocity, or null
     */
    public void pointAt(long nowNs, Pose pose, Velocity velocity, double targetX, double targetY) {
        double x = pose.x();
        double y = pose.y();
        double h = pose.heading();
        double lead = CameraPanConstants.LEAD_S;
        if (velocity != null && lead > 0) {
            x += velocity.vx * lead;
            y += velocity.vy * lead;
            h += velocity.omega * lead;
        }

        // Pan axis on the field.
        double af = CameraPanConstants.AXIS_FORWARD_M / 0.0254;
        double al = CameraPanConstants.AXIS_LEFT_M / 0.0254;
        double ax = x + af * Math.cos(h) - al * Math.sin(h);
        double ay = y + af * Math.sin(h) + al * Math.cos(h);

        double bearing = Math.toDegrees(MathUtil.normalizeRad(Math.atan2(targetY - ay, targetX - ax) - h));
        double wanted = normalizeDeg(bearing - VisionConstants.CAMERA_YAW_DEG);
        reachable = Math.abs(wanted) <= halfRangeDeg();
        wanted = MathUtil.clamp(wanted, -halfRangeDeg(), halfRangeDeg());
        wantedDeg = wanted;

        double dt = lastNs == 0 ? 0 : Math.min((nowNs - lastNs) * 1e-9, 0.1);
        lastNs = nowNs;
        double error = wanted - angleDeg;
        if (Math.abs(error) >= CameraPanConstants.DEADBAND_DEG) {
            double maxStep = CameraPanConstants.MAX_RATE_DEG_PER_S * dt;
            angleDeg += MathUtil.clamp(error, -maxStep, maxStep);
        }
        apply(nowNs);
    }

    private void apply(long nowNs) {
        double sign = CameraPanConstants.INCREASING_POSITION_TURNS_LEFT ? 1 : -1;
        servo.setPosition(MathUtil.clamp(CameraPanConstants.CENTER_POSITION
                + sign * angleDeg / CameraPanConstants.DEGREES_PER_FULL_TRAVEL, 0, 1));

        historyNs[historyNext] = nowNs;
        historyDeg[historyNext] = angleDeg;
        historyNext = (historyNext + 1) % HISTORY;
        historyCount = Math.min(historyCount + 1, HISTORY);
    }

    /** Commanded pan angle now, degrees left of the fixed-mount camera direction. */
    public double getAngleDeg() {
        return angleDeg;
    }

    /**
     * Best estimate of the real pan angle when a frame captured at {@code captureNs} was
     * taken: the commanded angle SERVO_LAG_S earlier.
     */
    public double angleAtNs(long captureNs) {
        if (historyCount == 0) {
            return angleDeg;
        }
        long t = captureNs - MathUtil.msToNs(CameraPanConstants.SERVO_LAG_S * 1000);
        int newest = (historyNext + HISTORY - 1) % HISTORY;
        for (int i = 0; i < historyCount; i++) {
            int idx = (newest + HISTORY - i) % HISTORY;
            if (historyNs[idx] <= t) {
                return historyDeg[idx];
            }
        }
        return historyDeg[(historyNext + HISTORY - historyCount) % HISTORY];
    }

    /** True if the camera cannot turn far enough to face the target (out of travel). */
    public boolean isOutOfRange() {
        return !reachable;
    }

    private static double normalizeDeg(double deg) {
        return Math.toDegrees(MathUtil.normalizeRad(Math.toRadians(deg)));
    }

    @Override
    public void stop() {
        // Holds its last position.
    }

    @Override
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Camera pan", "%.0f deg (wanted %.0f)%s", angleDeg, wantedDeg,
                reachable ? "" : "  OUT OF TRAVEL");
    }
}
