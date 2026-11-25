package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.constants.Configurables;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.utils.Alliance;
import org.firstinspires.ftc.teamcode.utils.EdgeDetector;
import org.firstinspires.ftc.teamcode.utils.MathUtil;
import org.firstinspires.ftc.teamcode.utils.ShotTable;

import java.util.ArrayList;
import java.util.List;

/**
 * Fill ShotTableConstants with measured shots. For each distance:
 *
 *   1. Drive (gamepad 1 sticks) to a spot in the shooting area with the up CELL in the
 *      Limelight's view; the turret auto-aims and the distance shows on telemetry.
 *   2. Hold right bumper to shoot (spin up, open gate, feed); hold left trigger to intake.
 *   3. Adjust until shots score: d-pad up/down = RPM -/+ 25 (hold B for 100),
 *      d-pad left/right = hood -/+ 0.01.
 *   4. Press Y to record the row. Rows are listed on telemetry and in the Robot log
 *      ("SHOT ROW"); copy them into ShotTableConstants.
 *
 * X resets RPM and hood to what the current table says for this distance.
 * Aims at the RED HIVE unless ALLIANCE is changed (Panels / here).
 */
@TeleOp(name = "Test: Shot Calibration", group = "Test")
public class ShotCalibration extends OpMode {
    public static Alliance ALLIANCE = Alliance.RED;

    private Robot robot;
    private double rpm = Configurables.SHOOTER_RPM;
    private double hood = Configurables.HOOD_POSITION;
    private final List<String> rows = new ArrayList<>();

    private final EdgeDetector rpmUp = new EdgeDetector();
    private final EdgeDetector rpmDown = new EdgeDetector();
    private final EdgeDetector hoodUp = new EdgeDetector();
    private final EdgeDetector hoodDown = new EdgeDetector();
    private final EdgeDetector record = new EdgeDetector();
    private final EdgeDetector fromTable = new EdgeDetector();
    private final EdgeDetector shoot = new EdgeDetector();

    @Override
    public void init() {
        robot = new Robot(this).enableShooterAssist().enableAutoAim(ALLIANCE);
    }

    @Override
    public void start() {
        robot.start();
    }

    @Override
    public void loop() {
        robot.beginLoop(); // auto-aim: turret target and table values for this distance
        double distanceIn = robot.targetDistanceIn();

        double step = gamepad1.b ? 100 : 25;
        if (rpmUp.rose(gamepad1.dpad_up)) {
            rpm += step;
        }
        if (rpmDown.rose(gamepad1.dpad_down)) {
            rpm -= step;
        }
        if (hoodUp.rose(gamepad1.dpad_right)) {
            hood += 0.01;
        }
        if (hoodDown.rose(gamepad1.dpad_left)) {
            hood -= 0.01;
        }
        rpm = MathUtil.clamp(rpm, 500, 6000);
        hood = MathUtil.clamp(hood, 0, 1);

        ShotTable.Shot table = ShotTable.lookup(distanceIn);
        if (fromTable.rose(gamepad1.x) && table != null) {
            rpm = table.rpm;
            hood = table.hoodPosition;
        }
        if (record.rose(gamepad1.y)) {
            String row = Double.isNaN(distanceIn)
                    ? "no target: not recorded"
                    : String.format("DIST_IN = %.1f, RPM = %.0f, HOOD = %.3f", distanceIn, rpm, hood);
            rows.add(row);
            RobotLog.ii("SHOT ROW", row);
        }

        // Our values, not the table's.
        robot.shooter.setShootRpm(rpm);
        robot.hood.setPosition(hood);

        robot.drivetrain.driveFieldCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x,
                gamepad1.right_stick_x);
        robot.runTurret(0);

        if (shoot.rose(gamepad1.right_bumper)) {
            robot.shootSequence.start(robot.nowSeconds());
        } else if (!gamepad1.right_bumper) {
            robot.shootSequence.cancel();
        }
        robot.runShooter(robot.shootSequence.wantsShooter());
        robot.shootSequence.update(robot.nowSeconds());
        if (robot.shootSequence.isFeeding()) {
            robot.runIntake(Intake.Mode.INTAKE);
        } else if (robot.shootSequence.isActive()) {
            robot.runIntake(Intake.Mode.OFF);
        } else {
            robot.runIntake(gamepad1.left_trigger > 0.2 ? Intake.Mode.INTAKE : Intake.Mode.OFF);
        }

        telemetry.addData("Distance to up CELL", Double.isNaN(distanceIn)
                ? "NO TARGET (put the up CELL in view)" : String.format("%.1f in", distanceIn));
        telemetry.addData("Trying", "%.0f RPM, hood %.3f", rpm, hood);
        telemetry.addData("Table says", table == null ? "-"
                : String.format("%.0f RPM, hood %.3f%s", table.rpm, table.hoodPosition,
                table.inRange ? "" : " (outside measured rows)"));
        telemetry.addLine("RB shoot | LT intake | dpad RPM/hood | B big step | X from table | Y record");
        for (int i = 0; i < rows.size(); i++) {
            telemetry.addData("Row " + (i + 1), rows.get(i));
        }
        robot.endLoop();
    }

    @Override
    public void stop() {
        if (robot != null) {
            robot.stop();
        }
    }
}
