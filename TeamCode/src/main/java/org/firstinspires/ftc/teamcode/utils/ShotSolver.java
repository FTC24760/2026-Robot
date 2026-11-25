package org.firstinspires.ftc.teamcode.utils;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.constants.HoodConstants;
import org.firstinspires.ftc.teamcode.constants.ShooterConstants;

/**
 * Works out how to hit the HIVE from where the robot is: turret bearing, wheel speed and
 * hood. ShotController calls update() each loop and reads the public fields.
 *
 * Placeholder for now: update() aims straight ahead at MANUAL_RPM with the hood up, and
 * never reports a shot as achievable.
 */
public class ShotSolver {
    // Hood servo position (not an angle yet).
    public double hoodAngle;
    public double shooterTargetRPM;
    // Turret bearing to aim at, radians to the left of robot forward.
    public double turretAngle;
    // True when the target is in range and the solution can be trusted. Not checked yet.
    public boolean shotAchievable;
    Follower follower;
    HiveTracker hiveTracker;
    // Read the alliance as robot.alliance when needed; don't copy it here, because
    // MatchTeleOp sets it after the Robot (and this solver) is built.
    Robot robot;
    public ShotSolver(Robot robot) {
        this.robot = robot;
        follower = robot.follower;
    }

    /** Robot turn rate from odometry, rad/s counter-clockwise; used as turret feedforward. */
    public double getAngularVelocity() {
        return follower.velocity().omega;
    }

    public void update() {
        hoodAngle = HoodConstants.HOOD_UP_POSITION;
        shooterTargetRPM = ShooterConstants.MANUAL_RPM;
        turretAngle = 0;
        shotAchievable = false;
    }

}
