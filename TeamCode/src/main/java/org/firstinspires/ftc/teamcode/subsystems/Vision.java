package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.constants.VisionConstants;
import org.firstinspires.ftc.teamcode.utils.Alliance;
import org.firstinspires.ftc.teamcode.utils.HiveCell;
import org.firstinspires.ftc.teamcode.utils.MathUtil;

import java.util.EnumMap;
import java.util.Map;

/**
 * Finds the HIVE with the Limelight. The camera looks up at the AprilTag clusters on the
 * underside of each CELL and this class turns them into three answers:
 *
 *   a) how far away the CELL is (horizontal distance from the robot centre),
 *   b) which way it is (bearing, degrees to the left of robot forward), and
 *   c) whether that CELL is the upward-facing one (the one to launch into).
 *
 * Robot frame used throughout: X forward, Y left, Z up, origin on the floor under the
 * robot centre, in metres.
 */
public class Vision implements Subsystem {

    /** One CELL as seen this frame, in the robot frame. */
    public static class CellSighting {
        public final HiveCell cell;
        /** Centre of the visible tags: forward, left, and height above the tiles. */
        public final double forwardM, leftM, heightM;
        public final int tagCount;

        CellSighting(HiveCell cell, double forwardM, double leftM, double heightM, int tagCount) {
            this.cell = cell;
            this.forwardM = forwardM;
            this.leftM = leftM;
            this.heightM = heightM;
            this.tagCount = tagCount;
        }

        public double distanceM() {
            return Math.hypot(forwardM, leftM);
        }

        /** Degrees to the left of robot forward; negative is to the right. */
        public double bearingDeg() {
            return Math.toDegrees(Math.atan2(leftM, forwardM));
        }
    }

    /** The CELL to aim at, smoothed over recent frames. */
    public static class HiveTarget {
        public final HiveCell cell;
        public final double distanceM;
        public final double bearingDeg;
        /** Height of the tag cluster above the tiles. */
        public final double tagHeightM;
        /** True if this CELL is the upward-facing one. */
        public final boolean cellUp;
        public final int tagCount;

        HiveTarget(HiveCell cell, double distanceM, double bearingDeg, double tagHeightM,
                   boolean cellUp, int tagCount) {
            this.cell = cell;
            this.distanceM = distanceM;
            this.bearingDeg = bearingDeg;
            this.tagHeightM = tagHeightM;
            this.cellUp = cellUp;
            this.tagCount = tagCount;
        }
    }

    private final Limelight3A limelight;
    private final Map<HiveCell, CellSighting> sightings = new EnumMap<>(HiveCell.class);

    private Alliance alliance;
    private HiveTarget target;
    private long targetNs;

    /** @param alliance whose HIVE to aim at, or null to take whichever CELL is nearest */
    public Vision(HardwareMap hardwareMap, Alliance alliance) {
        this.alliance = alliance;

        limelight = hardwareMap.get(Limelight3A.class, VisionConstants.LIMELIGHT_NAME);
        limelight.setPollRateHz(VisionConstants.POLL_RATE_HZ);
        limelight.pipelineSwitch(VisionConstants.PIPELINE);
        limelight.start();
    }

    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
    }

    /** Read the newest Limelight frame. Call once per loop. */
    public void update(long nowNs) {
        sightings.clear();

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            readSightings(result);
        }

        CellSighting best = chooseCell();
        if (best == null) {
            return; // keep the last target until it times out
        }

        boolean up = isUp(best);
        double distance = best.distanceM();
        double bearing = best.bearingDeg();
        double height = best.heightM;

        // Smooth against the previous frame, but only while it is the same CELL.
        if (hasTarget(nowNs) && target.cell == best.cell) {
            double a = MathUtil.clamp(VisionConstants.SMOOTHING, 0, 0.95);
            distance = a * target.distanceM + (1 - a) * distance;
            bearing = a * target.bearingDeg + (1 - a) * bearing;
            height = a * target.tagHeightM + (1 - a) * height;
        }

        target = new HiveTarget(best.cell, distance, bearing, height, up, best.tagCount);
        targetNs = nowNs;
    }

    /** @return the current target, or null if no HIVE tag has been seen recently. */
    public HiveTarget getTarget(long nowNs) {
        return hasTarget(nowNs) ? target : null;
    }

    public boolean hasTarget(long nowNs) {
        return target != null
                && nowNs - targetNs <= MathUtil.msToNs(VisionConstants.TARGET_TIMEOUT_MS);
    }

    /** Every CELL visible in the latest frame, for calibration and debugging. */
    public Map<HiveCell, CellSighting> getSightings() {
        return sightings;
    }

    public boolean isConnected() {
        return limelight.isConnected();
    }

    @Override
    public void stop() {
        limelight.stop();
    }

    @Override
    public void addTelemetry(Telemetry telemetry) {
        if (target == null) {
            telemetry.addData("Hive", limelight.isConnected() ? "not seen" : "LIMELIGHT NOT CONNECTED");
            return;
        }

        telemetry.addData("Hive cell", "%s (%s, %d tags)",
                target.cell, target.cellUp ? "UP" : "DOWN", target.tagCount);
        telemetry.addData("Hive distance m", "%.2f", target.distanceM);
        telemetry.addData("Hive bearing deg", "%.1f (%s)",
                Math.abs(target.bearingDeg), target.bearingDeg >= 0 ? "left" : "right");
        telemetry.addData("Tag height m", "%.2f", target.tagHeightM);
    }

    // Average the visible tags of each CELL into one point per CELL.
    private void readSightings(LLResult result) {
        Map<HiveCell, double[]> sums = new EnumMap<>(HiveCell.class);

        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            HiveCell cell = HiveCell.fromTagId(tag.getFiducialId());
            if (cell == null || tag.getTargetPoseCameraSpace() == null) {
                continue;
            }

            Position p = tag.getTargetPoseCameraSpace().getPosition().toUnit(DistanceUnit.METER);
            double[] robot = cameraToRobot(p.x, p.y, p.z);

            double[] sum = sums.get(cell);
            if (sum == null) {
                sum = new double[4];
                sums.put(cell, sum);
            }
            sum[0] += robot[0];
            sum[1] += robot[1];
            sum[2] += robot[2];
            sum[3] += 1;
        }

        for (Map.Entry<HiveCell, double[]> entry : sums.entrySet()) {
            double[] sum = entry.getValue();
            double n = sum[3];
            sightings.put(entry.getKey(), new CellSighting(
                    entry.getKey(), sum[0] / n, sum[1] / n, sum[2] / n, (int) n));
        }
    }

    /**
     * Limelight camera space (x right, y down, z out of the lens) to the robot frame
     * (X forward, Y left, Z up), using the mounting in VisionConstants.
     */
    static double[] cameraToRobot(double x, double y, double z) {
        if (VisionConstants.CAMERA_UPSIDE_DOWN) {
            x = -x;
            y = -y;
        }

        // Relative to the camera body: along the lens, to its left, and out of its top.
        double along = z;
        double left = -x;
        double top = -y;

        // Tilt the lens up by the pitch angle.
        double pitch = Math.toRadians(VisionConstants.CAMERA_PITCH_DEG);
        double forward = along * Math.cos(pitch) - top * Math.sin(pitch);
        double up = along * Math.sin(pitch) + top * Math.cos(pitch);

        // Turn the camera to its yaw on the robot.
        double yaw = Math.toRadians(VisionConstants.CAMERA_YAW_DEG);
        double robotForward = forward * Math.cos(yaw) - left * Math.sin(yaw);
        double robotLeft = forward * Math.sin(yaw) + left * Math.cos(yaw);

        return new double[]{
                robotForward + VisionConstants.CAMERA_FORWARD_M,
                robotLeft + VisionConstants.CAMERA_LEFT_M,
                up + VisionConstants.CAMERA_UP_M
        };
    }

    // Prefer the upward-facing CELL of our alliance; otherwise the nearest one we can see.
    private CellSighting chooseCell() {
        CellSighting best = null;

        for (CellSighting sighting : sightings.values()) {
            if (alliance != null && sighting.cell.alliance != alliance) {
                continue;
            }

            if (best == null) {
                best = sighting;
                continue;
            }

            boolean sightingUp = isUp(sighting);
            boolean bestUp = isUp(best);
            if (sightingUp != bestUp) {
                if (sightingUp) {
                    best = sighting;
                }
            } else if (sighting.distanceM() < best.distanceM()) {
                best = sighting;
            }
        }

        return best;
    }

    // The two CELLS of a HIVE are rigidly joined, so when both are visible the higher one
    // is the one facing up. With only one visible, fall back to a height threshold.
    private boolean isUp(CellSighting sighting) {
        for (CellSighting other : sightings.values()) {
            if (other != sighting && other.cell.alliance == sighting.cell.alliance) {
                return sighting.heightM > other.heightM;
            }
        }
        return sighting.heightM >= VisionConstants.CELL_UP_MIN_TAG_HEIGHT_M;
    }
}
