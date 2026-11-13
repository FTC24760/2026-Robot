package org.firstinspires.ftc.teamcode.opmodes.auto;

import static com.pedropathing.api.Paths.line;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.constants.AutoConstants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.utils.Alliance;

/**
 * Sample autonomous: shoot the 4 pre-loaded POLLEN as a rapid-fire burst, then (if
 * there is time) collect 4 more and shoot those too.
 *
 * The shooter spins up while the robot drives. Shooting uses the same ShootSequence as
 * the driver's shoot button: no ball is fed until the wheel has held shooting speed
 * for a moment, and feed gating holds each following ball until the wheel has
 * recovered. With auto-aim available, the turret, wheel speed and hood follow the
 * Limelight; without it the shooter uses the fixed TARGET_RPM.
 *
 * All positions are in AutoConstants and are placeholders until measured on a field.
 */
public abstract class RapidFireAuto extends OpMode {

    private enum Step {
        DRIVE_TO_SHOOT,
        SHOOT,
        DRIVE_TO_POLLEN,
        COLLECT,
        DONE
    }

    private final Alliance alliance;
    private Robot robot;
    private Step step = Step.DRIVE_TO_SHOOT;
    private int burstsFired;

    protected RapidFireAuto(Alliance alliance) {
        this.alliance = alliance;
    }

    @Override
    public void init() {
        robot = new Robot(this).enableShooterAssist();
        if (AutoConstants.USE_AUTO_AIM) {
            robot.enableAutoAim(alliance);
        }

        robot.drivetrain.setPose(new Pose(0, 0, 0));

        telemetry.addLine("Rapid Fire auto ready (" + alliance + ")");
        telemetry.addLine(robot.hasAutoAim() ? "Auto-aim on" : "Auto-aim off: fixed RPM, manual turret");
        telemetry.update();
    }

    @Override
    public void start() {
        resetRuntime();
        robot.start();
        robot.drivetrain.setPose(new Pose(0, 0, 0));
        robot.intake.setBlockerOpen(false);
        robot.flowerArm.setDown(false);

        driveTo(shootPose());
        step = Step.DRIVE_TO_SHOOT;
    }

    @Override
    public void loop() {
        robot.beginLoop();
        robot.butterfly.setDown();
        robot.drivetrain.update();
        robot.shootSequence.update(robot.nowSeconds());

        boolean spinUp = false;
        boolean collecting = false;

        switch (step) {
            case DRIVE_TO_SHOOT:
                // Spin up on the way so the wheel is ready on arrival.
                spinUp = true;
                if (!robot.drivetrain.isBusy()) {
                    robot.shootSequence.start(robot.nowSeconds());
                    step = Step.SHOOT;
                }
                break;

            case SHOOT:
                // The sequence waits for speed, opens the blocker, feeds, and closes it.
                if (!robot.shootSequence.isActive()) {
                    burstsFired++;

                    if (shouldRunSecondCycle()) {
                        driveTo(collectStartPose());
                        step = Step.DRIVE_TO_POLLEN;
                    } else {
                        step = Step.DONE;
                    }
                }
                break;

            case DRIVE_TO_POLLEN:
                collecting = true;
                if (!robot.drivetrain.isBusy()) {
                    driveTo(collectEndPose());
                    step = Step.COLLECT;
                }
                break;

            case COLLECT:
                collecting = true;
                if (!robot.drivetrain.isBusy()) {
                    driveTo(shootPose());
                    step = Step.DRIVE_TO_SHOOT;
                }
                break;

            case DONE:
            default:
                break;
        }

        if (AutoConstants.COLLECT_WITH_FLOWER_ARM) {
            robot.flowerArm.setDown(collecting);
        }

        // Shooter first, then intake, so feed gating sees this loop's wheel speed.
        robot.runShooter(spinUp || robot.shootSequence.wantsShooter());
        robot.runTurret(0);
        robot.runIntake(collecting || robot.shootSequence.isFeeding()
                ? Intake.Mode.INTAKE
                : Intake.Mode.OFF);

        telemetry.addData("Auto step", step);
        telemetry.addData("Bursts fired", burstsFired);
        telemetry.addData("Time left s", "%.1f", timeLeft());
        robot.endLoop();
    }

    @Override
    public void stop() {
        if (robot != null) {
            robot.stop();
        }
    }

    private boolean shouldRunSecondCycle() {
        return AutoConstants.SECOND_CYCLE_ENABLED
                && burstsFired < 2
                && timeLeft() >= AutoConstants.SECOND_CYCLE_MIN_TIME_LEFT_S;
    }

    private void driveTo(Pose target) {
        Pose from = robot.drivetrain.getPose();
        robot.drivetrain.follow(line(from, target).linear(from.heading(), target.heading()));
    }

    private double timeLeft() {
        return AutoConstants.AUTO_LENGTH_S - getRuntime();
    }

    private Pose shootPose() {
        return pose(AutoConstants.SHOOT_X, AutoConstants.SHOOT_Y, AutoConstants.SHOOT_HEADING_DEG);
    }

    private Pose collectStartPose() {
        return pose(AutoConstants.COLLECT_START_X, AutoConstants.COLLECT_START_Y,
                AutoConstants.COLLECT_HEADING_DEG);
    }

    private Pose collectEndPose() {
        return pose(AutoConstants.COLLECT_END_X, AutoConstants.COLLECT_END_Y,
                AutoConstants.COLLECT_HEADING_DEG);
    }

    // Constants are written for red; blue mirrors left and right.
    private Pose pose(double x, double y, double headingDeg) {
        double side = alliance == Alliance.BLUE ? -1 : 1;
        return new Pose(x, side * y, Math.toRadians(side * headingDeg));
    }
}
