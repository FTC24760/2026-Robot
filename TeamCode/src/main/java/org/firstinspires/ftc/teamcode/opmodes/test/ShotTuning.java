package org.firstinspires.ftc.teamcode.opmodes.test;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.constants.HoodConstants;
import org.firstinspires.ftc.teamcode.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.utils.Alliance;
import org.firstinspires.ftc.teamcode.utils.EdgeDetector;
import org.firstinspires.ftc.teamcode.utils.MathUtil;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

/**
 * Collects (distance, RPM, hood) points for the Desmos fits that ShotSolver uses.
 *
 * Runs the full TeleOp (drive, intake, shoot, turret auto-aim at the nearest CELL), but
 * the wheel speed and hood come from the operator instead of the fits:
 *   operator d-pad up / down     RPM up / down by RPM_STEP
 *   operator d-pad right / left  hood up / down by HOOD_STEP
 *   operator A                   save a point: distance, RPM, hood
 * Everything else is the normal button map (see Controls).
 *
 * How to use: start against the wall at the auto start pose (or press operator X there).
 * Drive to a spot, shoot, adjust RPM and hood until shots go in, press A. Repeat at many
 * distances, and at a few RPMs per distance so the hood fit can separate the two.
 *
 * The points are on the Driver Station screen and in
 * shot_tuning/shot_points_<time>.csv under the app's external files folder (the same
 * place the shooter RPM logs went). In Desmos paste them as a table with columns
 * d_1, r_1, h_1, then fit
 *   r_1 ~ m d_1 + b              -> ShooterConstants.RPM_SLOPE = m, RPM_INTERCEPT = b
 *   h_1 ~ a + c d_1 + e r_1      -> HoodConstants.HOOD_INTERCEPT = a,
 *                                   HOOD_DISTANCE_COEF = c, HOOD_RPM_COEF = e
 */
@Configurable
@TeleOp(name = "Test: Shot Tuning", group = "Test")
public class ShotTuning extends OpMode {
    public static Alliance ALLIANCE = Alliance.RED;
    // One step of the hub's velocity reading (20 ticks/s), so targets stay on a step.
    public static double RPM_STEP = 20 * 60 / ShooterConstants.TICKS_PER_REV;
    public static double HOOD_STEP = 0.01;

    private Robot robot;
    private double rpm;
    private double hood;

    private final EdgeDetector rpmUp = new EdgeDetector();
    private final EdgeDetector rpmDown = new EdgeDetector();
    private final EdgeDetector hoodUp = new EdgeDetector();
    private final EdgeDetector hoodDown = new EdgeDetector();
    private final EdgeDetector save = new EdgeDetector();

    private final List<String> points = new ArrayList<>();
    private FileWriter csv;
    private String fileStatus;

    @Override
    public void init() {
        robot = new Robot(this, ALLIANCE);
        rpm = ShooterConstants.MANUAL_RPM;
        hood = HoodConstants.HOOD_UP_POSITION;
        openCsv();
    }

    @Override
    public void start() {
        robot.start();
    }

    @Override
    public void loop() {
        if (rpmUp.rose(gamepad2.dpad_up)) rpm += RPM_STEP;
        if (rpmDown.rose(gamepad2.dpad_down)) rpm -= RPM_STEP;
        rpm = MathUtil.clamp(rpm, 0, ShooterConstants.MAX_RPM);

        double low = Math.min(HoodConstants.HOOD_DOWN_POSITION, HoodConstants.HOOD_UP_POSITION);
        double high = Math.max(HoodConstants.HOOD_DOWN_POSITION, HoodConstants.HOOD_UP_POSITION);
        if (hoodUp.rose(gamepad2.dpad_right)) hood += HOOD_STEP;
        if (hoodDown.rose(gamepad2.dpad_left)) hood -= HOOD_STEP;
        hood = MathUtil.clamp(hood, low, high);

        robot.shotSolver.rpmOverride = rpm;
        robot.shotSolver.hoodOverride = hood;

        if (save.rose(gamepad2.a)) {
            savePoint(robot.shotSolver.distanceIn, rpm, hood);
        }

        telemetry.addData("Tuning RPM", "%.0f   (actual %.0f)", rpm, robot.shooter.getActualRpm());
        telemetry.addData("Tuning hood", "%.3f", hood);
        telemetry.addData("Distance in", "%.1f", robot.shotSolver.distanceIn);
        telemetry.addData("CSV", fileStatus);
        telemetry.addLine("distance, rpm, hood:");
        for (String point : points) {
            telemetry.addLine(point);
        }

        // Last: Robot.update() sends the telemetry.
        robot.update();
    }

    @Override
    public void stop() {
        if (robot != null) {
            robot.stop();
        }
        if (csv != null) {
            try {
                csv.close();
            } catch (IOException ignored) {
            }
        }
    }

    private void savePoint(double distance, double rpm, double hood) {
        String line = String.format("%.1f, %.0f, %.3f", distance, rpm, hood);
        points.add(line);
        if (csv != null) {
            try {
                csv.write(String.format("%.2f,%.1f,%.4f%n", distance, rpm, hood));
                csv.flush();  // keep every point even if the OpMode is killed
            } catch (IOException e) {
                fileStatus = "WRITE FAILED: " + e.getMessage();
            }
        }
    }

    private void openCsv() {
        try {
            File folder = new File(hardwareMap.appContext.getExternalFilesDir(null), "shot_tuning");
            if (!folder.isDirectory() && !folder.mkdirs()) {
                throw new IOException("can't create " + folder);
            }
            File file = new File(folder, "shot_points_" + System.currentTimeMillis() + ".csv");
            csv = new FileWriter(file);
            csv.write("distance_in,rpm,hood\n");
            csv.flush();
            fileStatus = file.getAbsolutePath();
        } catch (IOException e) {
            csv = null;
            fileStatus = "NOT SAVING: " + e.getMessage();
        }
    }
}
