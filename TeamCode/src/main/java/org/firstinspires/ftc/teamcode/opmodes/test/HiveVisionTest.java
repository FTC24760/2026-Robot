package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.utils.HiveCell;
import org.firstinspires.ftc.teamcode.utils.ShotSolver;

/**
 * Limelight only: shows every HIVE CELL in view with its distance, bearing and tag
 * height, and the shot the solver would pick. Nothing moves, so the hood and shooter
 * do not need to be connected.
 *
 * Use it to calibrate VisionConstants:
 *   - Put the robot a tape-measured distance from a CELL and compare "distance".
 *   - Compare "tag height" with the real height of the tags above the tiles.
 *     If it is negative or wildly off, check CAMERA_UPSIDE_DOWN and CAMERA_PITCH_DEG.
 *   - Tip the HIVE each way and note "tag height" for the same CELL up and down, then
 *     set CELL_UP_MIN_TAG_HEIGHT_M halfway between.
 */
@TeleOp(name = "Test: Hive Vision", group = "Test")
public class HiveVisionTest extends OpMode {
    private Vision vision;

    @Override
    public void init() {
        // No alliance: report whichever CELLS are in view.
        vision = new Vision(hardwareMap, null);
    }

    @Override
    public void loop() {
        long nowNs = System.nanoTime();
        vision.update(nowNs);

        telemetry.addData("Limelight", vision.isConnected() ? "connected" : "NOT CONNECTED");

        if (vision.getSightings().isEmpty()) {
            telemetry.addLine("No HIVE tags in view (IDs 30-45)");
        }

        for (HiveCell cell : HiveCell.values()) {
            Vision.CellSighting s = vision.getSightings().get(cell);
            if (s == null) {
                continue;
            }

            telemetry.addLine("--- " + cell + " (" + s.tagCount + " tags) ---");
            telemetry.addData("distance m", "%.2f", s.distanceM());
            telemetry.addData("bearing deg", "%.1f (%s)",
                    Math.abs(s.bearingDeg()), s.bearingDeg() >= 0 ? "left" : "right");
            telemetry.addData("tag height m", "%.2f", s.heightM);
            telemetry.addData("forward / left m", "%.2f / %.2f", s.forwardM, s.leftM);
        }

        Vision.HiveTarget target = vision.getTarget(nowNs);
        if (target != null) {
            ShotSolver.Solution shot = ShotSolver.solveForCell(target.distanceM, target.tagHeightM);

            telemetry.addLine("=== Chosen target ===");
            telemetry.addData("cell", "%s, %s", target.cell, target.cellUp ? "UP" : "DOWN");
            telemetry.addData("shot", shot.reachable
                    ? String.format("%.0f RPM, %.1f deg, hood %.3f",
                            shot.rpm, shot.launchAngleDeg, shot.hoodPosition)
                    : "OUT OF RANGE");
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        if (vision != null) {
            vision.stop();
        }
    }
}
