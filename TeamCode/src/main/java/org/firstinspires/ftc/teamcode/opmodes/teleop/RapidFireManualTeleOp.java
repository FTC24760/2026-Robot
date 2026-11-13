package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

/**
 * "TeleOp (Shooter Assist, manual)" with rapid fire in place of the staggered feed:
 * balls go on a fixed beat, without waiting for the wheel to recover after each one. Faster and less forgiving; tune the
 * RAPID_ values in ShooterAssistConstants.
 */
@TeleOp(name = "TeleOp (Rapid Fire, manual)", group = "Competition")
public class RapidFireManualTeleOp extends RobotTeleOp {
    @Override
    protected Robot createRobot() {
        return new Robot(this)
                .enableShooterAssist()
                .enableRapidFire()
                .enableShooterLogging()
                .useManualControls();
    }
}
