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
 * there is time) collect the 4 from our GARDEN and shoot those too.
 *
 * The shooter spins up while the robot drives, and no ball is fed until the wheel has
 * held shooting speed for a moment. Feed gating then holds each following ball until
 * the wheel has recovered. With auto-aim on, speed and hood follow the Limelight.
 *
 * All positions are in AutoConstants and are placeholders until measured on a field.
 */
public abstract class RapidFireAuto extends OpMode {

    private enum Step {
        DRIVE_TO_SHOOT,
        WAIT_FOR_SPEED,
        FIRE,
        DRIVE_TO_POLLEN,
        COLLECT,
        DONE
    }

    private final Alliance alliance;
    private Robot robot;
    private Step step = Step.DRIVE_TO_SHOOT;
    private double stepStartS;
    private double readySinceS = -1;
    private int burstsFired;
    private String aimNote = "";

    protected RapidFireAuto(Alliance alliance) {
        this.alliance = alliance;
    }

    @Override
    public void init() {
        robot = new Robot(this).enableShooterAssist();

        if (AutoConstants.USE_AUTO_AIM) {
            try {
                robot.enableAutoAim(alliance);
            } catch (RuntimeException e) {
                // No Limelight or hood servo in the configuration: shoot at the fixed RPM.
                aimNote = "Auto-aim unavailable, using fixed RPM";
            }
        }

        robot.drivetrain.setPose(new Pose(0, 0, 0));
        telemetry.addLine("Rapid Fire auto ready (" + alliance + ")");
        telemetry.addLine(aimNote);
        telemetry.update();
    }

    @Override
    public void start() {
        resetRuntime();
        robot.start();
        robot.drivetrain.setPose(new Pose(0, 0, 0));
        robot.intake.setBlockerOpen(false);
        driveTo(shootPose());
        enter(Step.DRIVE_TO_SHOOT);
    }

    @Override
    public void loop() {
        robot.beginLoop();
        robot.butterfly.setDown();
        robot.drivetrain.update();

        boolean shoot = false;
        Intake.Mode intakeMode = Intake.Mode.OFF;

        switch (step) {
            case DRIVE_TO_SHOOT:
                // Spin up on the way so the wheel is ready on arrival.
                shoot = true;
                if (!robot.drivetrain.isBusy()) {
                    readySinceS = -1;
                    enter(Step.WAIT_FOR_SPEED);
                }
                break;

            case WAIT_FOR_SPEED:
                shoot = true;
                if (wheelSettled() || stepTime() > AutoConstants.SPIN_UP_TIMEOUT_S) {
                    robot.intake.setBlockerOpen(true);
                    enter(Step.FIRE);
                }
                break;

            case FIRE:
                shoot = true;
                // Intake roller and transfer together, so all 4 balls reach the wheel.
                intakeMode = Intake.Mode.INTAKE;
                if (stepTime() > AutoConstants.FIRE_S) {
                    robot.intake.setBlockerOpen(false);
                    burstsFired++;

                    if (shouldRunSecondCycle()) {
                        driveTo(collectStartPose());
                        enter(Step.DRIVE_TO_POLLEN);
                    } else {
                        enter(Step.DONE);
                    }
                }
                break;

            case DRIVE_TO_POLLEN:
                intakeMode = Intake.Mode.INTAKE;
                if (!robot.drivetrain.isBusy()) {
                    driveTo(collectEndPose());
                    enter(Step.COLLECT);
                }
                break;

            case COLLECT:
                intakeMode = Intake.Mode.INTAKE;
                if (!robot.drivetrain.isBusy()) {
                    driveTo(shootPose());
                    enter(Step.DRIVE_TO_SHOOT);
                }
                break;

            case DONE:
            default:
                break;
        }

        // Shooter first, then intake, so feed gating sees this loop's wheel speed.
        robot.runShooter(shoot);
        robot.runIntake(intakeMode);

        telemetry.addData("Auto step", step);
        telemetry.addData("Bursts fired", burstsFired);
        telemetry.addData("Time left s", "%.1f", timeLeft());
        if (!aimNote.isEmpty()) {
            telemetry.addLine(aimNote);
        }
        robot.endLoop();
    }

    @Override
    public void stop() {
        if (robot != null) {
            robot.stop();
        }
    }

    // The wheel has to stay at speed for a moment, not just touch it once.
    private boolean wheelSettled() {
        if (!robot.shooter.isReadyToFeed()) {
            readySinceS = -1;
            return false;
        }
        if (readySinceS < 0) {
            readySinceS = getRuntime();
        }
        return getRuntime() - readySinceS >= AutoConstants.READY_SETTLE_S;
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

    private void enter(Step next) {
        step = next;
        stepStartS = getRuntime();
    }

    private double stepTime() {
        return getRuntime() - stepStartS;
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
