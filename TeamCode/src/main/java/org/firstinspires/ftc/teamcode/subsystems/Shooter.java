package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.opmodes.teleop.JerryRiggedPidfTeleOp;
import org.firstinspires.ftc.teamcode.utils.SpeedController;
import org.firstinspires.ftc.teamcode.ShooterConstants;

@Configurable
public class Shooter {
    private long lastLoopNs;
    double hoodAngle, flywheelRPM;
    DcMotorEx shooterMotor;
    private final SpeedController speedController;

    public enum State {
        SHOOT,
        IDLE,
        DISABLED
    }

    public Shooter(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, ShooterConstants.shooterName);
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        speedController = new SpeedController(hardwareMap);
    }

    public void Update(State state, double voltageScale) {
        long now = System.nanoTime();
        double loopSeconds = lastLoopNs == 0 ? 0 : Math.min((now - lastLoopNs) * 1e-9, 0.1);
        lastLoopNs = now;

        double shooterPower = 0;
        double actualTicksPerSecond = shooterMotor.getVelocity();

        double TICKS_PER_REV = 28;
        double targetTicksPerSecond;

        switch (state) {
            case SHOOT:
                targetTicksPerSecond = ShooterConstants.TARGET_RPM * TICKS_PER_REV / 60.0;
                shooterPower = speedController.calculate(targetTicksPerSecond, actualTicksPerSecond, voltageScale, now);
                break;
            case IDLE:
                targetTicksPerSecond = ShooterConstants.IDLE_RPM * TICKS_PER_REV / 60.0;
                shooterPower = speedController.calculate(targetTicksPerSecond, actualTicksPerSecond, voltageScale, now);
                break;
            case DISABLED:
                speedController.reset();
                shooterPower = 0.0;
                break;
        }
        shooterMotor.setPower(shooterPower);
    }


}
