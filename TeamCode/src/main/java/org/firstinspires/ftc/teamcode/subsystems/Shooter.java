package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.SpeedController;
import org.firstinspires.ftc.teamcode.ShooterConstants;

@Configurable
public class Shooter {
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
