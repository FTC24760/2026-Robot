package org.firstinspires.ftc.teamcode.utils;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.constants.HoodConstants;
import org.firstinspires.ftc.teamcode.constants.ShooterConstants;

public class ShotSolver {
    public double hoodAngle;
    public double shooterTargetRPM;
    // Turret bearing to aim at, radians to the left of robot forward.
    public double turretAngle;
    public boolean shotAchievable;
    Follower follower;
    Alliance alliance;
    HiveTracker hiveTracker;
    public ShotSolver(Robot robot) {
        follower = robot.follower;
        alliance = robot.alliance;
    }

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
