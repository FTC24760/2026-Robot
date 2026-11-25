package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.ButterflyConstants;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * Competition drive: mecanum through the Pedro follower, field-centric off the
 * localizer heading, holding position when the sticks are released.
 * Motor names, localizer and follower tuning live in pedro/Constants.
 */
public class Drivetrain implements Subsystem {
    private final Follower follower;
    private Pose pose = new Pose(0, 0, 0);
    private Velocity velocity = Velocity.zero();
    // System.nanoTime() when pose was last read from the localizer.
    private long poseNs = System.nanoTime();
    // Field heading tank drive turns to, radians CCW. Kept when the stick is released.
    public double tankTargetHeading;

    public Drivetrain(HardwareMap hardwareMap, Follower follower) {
        this.follower = follower;
    }

    /**
     * Drive relative to the field, and update the localizer. Call once per loop.
     *
     * @param forward positive = away from the driver
     * @param strafe  positive = to the driver's right
     * @param rotate  positive = clockwise
     */
    public void driveFieldCentric(double forward, double strafe, double rotate) {
        // Pedro uses left-positive strafe and counter-clockwise-positive rotation.
        DrivePowers powers = ManualDrive.fieldCentric(
                forward,
                -strafe,
                -rotate,
                follower.pose().heading()
        );
        follower.manual(powers);
    }

    /**
     * Field-oriented tank drive, for when the butterfly pods are down and the robot can't
     * strafe. The robot turns to face along the stick (or directly away from it) using
     * Pedro's tuned heading controller, and drives with the part of the stick that lies
     * along its forward axis, so it slows while it's still turning. Call
     * {@link #resetTankHeading()} when tank drive starts.
     *
     * @param forward    stick toward the far side of the field, as in driveFieldCentric
     * @param strafe     stick to the driver's right, as in driveFieldCentric
     * @param facingAway false: face the stick direction and drive forward;
     *                   true: face away from it and drive backward
     */
    public void driveTank(double forward, double strafe, boolean facingAway) {
        double heading = follower.pose().heading();
        // Field direction of the stick in Pedro's frame (left-positive, CCW angles).
        double fieldLeft = -strafe;
        double magnitude = Math.min(1.0, Math.hypot(forward, fieldLeft));
        double stickAngle = Math.atan2(fieldLeft, forward);

        if (magnitude > ButterflyConstants.TANK_STICK_DEADBAND) {
            tankTargetHeading = facingAway ? stickAngle + Math.PI : stickAngle;
        } else {
            magnitude = 0;
        }

        // The stick vector projected onto the robot's forward axis. Negative when facing
        // away, so the robot backs toward the stick direction.
        double drive = magnitude * Math.cos(stickAngle - heading);
        DrivePowers powers = ManualDrive.headingLock(
                Constants.foresightConfig,
                follower,
                new DrivePowers(drive, 0, 0),
                tankTargetHeading,
                ButterflyConstants.TANK_HEADING_STRENGTH
        );
        follower.manual(powers);
    }

    /** Hold the current heading until the stick says otherwise. */
    public void resetTankHeading() {
        tankTargetHeading = follower.pose().heading();
    }

    // ---- Autonomous ----

    /** Tell the localizer where the robot is. Call once at the start of an auto. */
    public void setPose(Pose pose) {
        moveOrigin(pose);
    }

    /** Run the follower and localizer for one loop of an auto. */
    public void update() {
        follower.update();
    }

    // Tell the localizer it is now at newPose. If the follower is holding position, move
    // the hold target too: it is stored in the old coordinates, so leaving it would make
    // the robot drive and spin back to "where it was" in the new frame.
    private void moveOrigin(Pose newPose) {
        follower.setPose(newPose);
        if (follower.holding()) {
            follower.hold(newPose);
        }
    }
    @Override
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Robot X", pose.x());
        telemetry.addData("Robot Y", pose.y());
        telemetry.addData("Robot Heading", Math.toDegrees(pose.heading()));
    }
    @Override
    public void stop() {
    }
}
