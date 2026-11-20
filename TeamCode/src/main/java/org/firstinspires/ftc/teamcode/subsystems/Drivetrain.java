package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * Competition drive: mecanum through the Pedro follower, field-centric off the
 * localizer heading, holding position when the sticks are released.
 * Motor names, localizer and follower tuning live in pedro/Constants.
 */
public class Drivetrain implements Subsystem {
    private final Follower follower;
    private Pose pose = new Pose(0, 0, 0);
    // +1 or -1 while shifting side to side, 0 when not.
    private int shiftSide;
    private Velocity velocity = Velocity.zero();
    // System.nanoTime() when pose was last read from the localizer.
    private long poseNs = System.nanoTime();

    public Drivetrain(HardwareMap hardwareMap) {
        follower = Constants.create(hardwareMap);
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
        ManualDrive.driveOrHold(follower, powers);
        follower.update();
        readPose();
    }

    // ---- Autonomous ----

    /** Tell the localizer where the robot is. Call once at the start of an auto. */
    public void setPose(Pose pose) {
        moveOrigin(pose);
        driveFieldCentric(0.02, 0.02, 0.02);
    }

    /** Start following a path. Keep calling update() every loop until isBusy() is false. */
    public void follow(Path path) {
        follower.follow(path);
    }

    /** Drive to a pose and hold it against being pushed. */
    public void hold(Pose pose) {
        follower.hold(pose);
    }

    /**
     * Shift side to side around {@code center}, robot-relative: half of {@code totalIn}
     * to the left, then half to the right, switching every {@code periodS}. Call every
     * loop; the follower runs from update() or driveFieldCentric()'s loop.
     *
     * @param elapsedS seconds since the shifting began
     */
    public void shiftAround(Pose center, double totalIn, double periodS, double elapsedS) {
        int side = ((int) Math.floor(elapsedS / Math.max(periodS, 0.05))) % 2 == 0 ? 1 : -1;
        if (side == shiftSide) {
            return;
        }
        shiftSide = side;

        double half = side * totalIn / 2.0;
        double h = center.heading();
        // Robot-relative left is 90 degrees counter-clockwise from its heading.
        follower.hold(new Pose(
                center.x() - Math.sin(h) * half,
                center.y() + Math.cos(h) * half,
                h));
    }

    /** Stop shifting; the next shiftAround() starts again from its first side. */
    public void endShift() {
        shiftSide = 0;
    }

    /** Run the follower for one loop without new stick input (holding or shifting). */
    public void updateHold() {
        follower.update();
        readPose();
    }

    /** True while a path is still being followed. */
    public boolean isBusy() {
        return follower.isBusy();
    }

    /** Run the follower and localizer for one loop of an auto. */
    public void update() {
        follower.update();
        readPose();
    }

    /** Make the robot's current position and heading the field origin. */
    public void resetPose() {
        moveOrigin(new Pose(0, 0, 0));
    }

    /** Pose as of the last driveFieldCentric or update call. */
    public Pose getPose() {
        return pose;
    }

    /** When {@link #getPose()} was read, in System.nanoTime() terms. */
    public long getPoseTimeNs() {
        return poseNs;
    }

    /** Field-frame velocity: inches per second and radians per second (CCW positive). */
    public Velocity getVelocity() {
        return velocity;
    }

    // Tell the localizer it is now at newPose. If the follower is holding position, move
    // the hold target too: it is stored in the old coordinates, so leaving it would make
    // the robot drive and spin back to "where it was" in the new frame.
    private void moveOrigin(Pose newPose) {
        follower.setPose(newPose);
        if (follower.holding()) {
            follower.hold(newPose);
        }
        pose = newPose;
        poseNs = System.nanoTime();
    }

    private void readPose() {
        pose = follower.pose();
        velocity = follower.velocity();
        poseNs = System.nanoTime();
    }

    @Override
    public void stop() {
        // The SDK cuts motor power when the OpMode stops.
    }

    @Override
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Robot X", pose.x());
        telemetry.addData("Robot Y", pose.y());
        telemetry.addData("Robot Heading", Math.toDegrees(pose.heading()));
    }
}
