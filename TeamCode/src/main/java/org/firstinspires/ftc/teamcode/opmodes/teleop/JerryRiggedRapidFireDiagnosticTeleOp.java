
package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.ButterflyController;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.List;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicInteger;

@TeleOp(
        name = "Jerry Rigged PIDF Baseline Logger",
        group = "Robot"
)
public class JerryRiggedRapidFireDiagnosticTeleOp extends OpMode {

    // ===================================
    // BASELINE SHOOTER CONFIGURATION
    // ===================================

    public static double intakePower = 0.6;
    public static double transferPower = 0.8;

    // goBILDA 5000 Series bare motor encoder.
    // Motor to flywheel belt ratio: 1:1.
    private static final double TICKS_PER_REV = 28.0;

    public static double TARGET_RPM = 1410.0;
    public static double RPM_STEP = 20.0;

    // Original conservative software PIDF.
    // Gains use ticks/second, not hub PIDF units.
    public static double kF = 0.0006825;
    public static double kS = 0.069;
    public static double kP = 0.0015;
    public static double kI = 0.00002;
    public static double kD = 0.00005;
    public static double kA = 0.0;

    public static double MAX_INTEGRAL = 5000.0;
    public static double NOMINAL_VOLTAGE = 12.0;
    public static double MAX_ACCEL = 6000.0;
    public static double VELOCITY_ALPHA = 0.5;
    public static double KS_DEADBAND = 50.0;

    public static double TURRET_RATE = 0.25;

    // ===================================
    // HARDWARE
    // ===================================

    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backLeftDrive;
    private DcMotor backRightDrive;

    private DcMotor intakeMotor;
    private DcMotor transferMotor;
    private DcMotorEx shooterMotor;

    private Servo leftTurretServo;
    private Servo rightTurretServo;
    private Servo blockerServo;

    private IMU imu;
    private ButterflyController butterfly;

    private List<LynxModule> hubs;
    private VoltageSensor voltageSensor;

    // ===================================
    // TURRET AND BLOCKER
    // ===================================

    private double leftTurretPosition = 0.5;
    private double rightTurretPosition = 0.5;

    private static final double BLOCKER_OPEN = 0.032;
    private static final double BLOCKER_CLOSED = 0.0;

    private boolean blockerOpen = false;
    private boolean lastA = false;

    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;

    // ===================================
    // PIDF
    // ===================================

    private final SpeedController speedController =
            new SpeedController();

    private long lastLoopNs = 0;
    private long lastVoltageNs = 0;
    private long lastTelemetryNs = 0;
    private long recordingStartNs = 0;

    private double voltage = Double.NaN;
    private double voltageScale = 1.0;

    // ===================================
    // CSV LOGGER
    // ===================================

    private final AsyncCsvLogger logger =
            new AsyncCsvLogger();

    // ===================================
    // BURST DIAGNOSTICS
    // ===================================

    private boolean wasFeeding = false;
    private int burstNumber = 0;

    private double burstStartRpm = Double.NaN;
    private double burstMinRpm = Double.NaN;
    private double burstMaxRpm = Double.NaN;

    private double burstMaxPower = 0.0;
    private double burstMinVoltage = Double.NaN;

    private double lastBurstStartRpm = Double.NaN;
    private double lastBurstMinRpm = Double.NaN;
    private double lastBurstMaxRpm = Double.NaN;

    // ===================================
    // INIT
    // ===================================

    @Override
    public void init() {

        frontLeftDrive =
                hardwareMap.get(DcMotor.class, "lf");

        frontRightDrive =
                hardwareMap.get(DcMotor.class, "rf");

        backLeftDrive =
                hardwareMap.get(DcMotor.class, "lb");

        backRightDrive =
                hardwareMap.get(DcMotor.class, "rb");

        intakeMotor =
                hardwareMap.get(DcMotor.class, "intake");

        transferMotor =
                hardwareMap.get(DcMotor.class, "transfer");

        shooterMotor =
                hardwareMap.get(DcMotorEx.class, "shooter");

        leftTurretServo =
                hardwareMap.get(Servo.class, "leftturret");

        rightTurretServo =
                hardwareMap.get(Servo.class, "rightturret");

        blockerServo =
                hardwareMap.get(Servo.class, "blocker");

        frontLeftDrive.setDirection(
                DcMotor.Direction.REVERSE
        );

        backLeftDrive.setDirection(
                DcMotor.Direction.REVERSE
        );

        transferMotor.setDirection(
                DcMotor.Direction.REVERSE
        );

        frontLeftDrive.setMode(
                DcMotor.RunMode.RUN_USING_ENCODER
        );

        frontRightDrive.setMode(
                DcMotor.RunMode.RUN_USING_ENCODER
        );

        backLeftDrive.setMode(
                DcMotor.RunMode.RUN_USING_ENCODER
        );

        backRightDrive.setMode(
                DcMotor.RunMode.RUN_USING_ENCODER
        );

        shooterMotor.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );

        shooterMotor.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.FLOAT
        );

        hubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(
                    LynxModule.BulkCachingMode.MANUAL
            );
        }

        voltageSensor =
                hardwareMap.voltageSensor.iterator().hasNext()
                        ? hardwareMap.voltageSensor.iterator().next()
                        : null;

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot orientation =
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                );

        imu.initialize(
                new IMU.Parameters(orientation)
        );

        butterfly =
                new ButterflyController(hardwareMap);

        blockerServo.setPosition(BLOCKER_CLOSED);
    }

    // ===================================
    // START
    // ===================================

    @Override
    public void start() {

        lastLoopNs = 0;
        lastVoltageNs = 0;
        lastTelemetryNs = 0;
        recordingStartNs = 0;

        wasFeeding = false;
        burstNumber = 0;

        speedController.reset();

        logger.start(
                hardwareMap.appContext.getExternalFilesDir(null)
        );
    }

    // ===================================
    // MAIN LOOP
    // ===================================

    @Override
    public void loop() {

        long now = System.nanoTime();

        double dt = lastLoopNs == 0
                ? 0.01
                : Math.max(
                0.0001,
                Math.min(
                        (now - lastLoopNs) * 1e-9,
                        0.1
                )
        );

        lastLoopNs = now;

        // Exactly one fresh bulk snapshot per loop.
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }

        double measuredTicks =
                shooterMotor.getVelocity();

        double rawRpm =
                measuredTicks * 60.0 / TICKS_PER_REV;

        // ===================================
        // TARGET ADJUSTMENT
        // ===================================

        if (gamepad1.dpad_up && !lastDpadUp) {
            TARGET_RPM = clamp(
                    TARGET_RPM + RPM_STEP,
                    1100.0,
                    1700.0
            );
        }

        if (gamepad1.dpad_down && !lastDpadDown) {
            TARGET_RPM = clamp(
                    TARGET_RPM - RPM_STEP,
                    1100.0,
                    1700.0
            );
        }

        lastDpadUp = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;

        // ===================================
        // INPUTS
        // ===================================

        boolean shooterEnabled =
                gamepad1.right_trigger > 0.2;

        boolean feeding =
                gamepad1.left_trigger > 0.2
                        || gamepad1.x;

        // ===================================
        // VOLTAGE
        // ===================================

        if (lastVoltageNs == 0
                || now - lastVoltageNs >= 250_000_000L) {

            double v = voltageSensor == null
                    ? 0.0
                    : voltageSensor.getVoltage();

            voltage = v > 0.1
                    ? v
                    : Double.NaN;

            voltageScale = v > 0.1
                    ? NOMINAL_VOLTAGE / v
                    : 1.0;

            lastVoltageNs = now;
        }

        // ===================================
        // ORIGINAL PIDF CONTROL
        // ===================================

        double targetTicks = shooterEnabled
                ? TARGET_RPM * TICKS_PER_REV / 60.0
                : 0.0;

        double shooterPower;

        if (shooterEnabled) {

            shooterPower = speedController.calculate(
                    targetTicks,
                    measuredTicks,
                    voltageScale,
                    now
            );

        } else {

            speedController.reset();
            shooterPower = 0.0;
        }

        shooterMotor.setPower(shooterPower);

        // ===================================
        // BURST STATISTICS
        // ===================================

        if (feeding && !wasFeeding) {

            burstNumber++;

            burstStartRpm = rawRpm;
            burstMinRpm = rawRpm;
            burstMaxRpm = rawRpm;

            burstMaxPower = 0.0;
            burstMinVoltage = Double.NaN;
        }

        if (feeding) {

            burstMinRpm = Math.min(
                    burstMinRpm,
                    rawRpm
            );

            burstMaxRpm = Math.max(
                    burstMaxRpm,
                    rawRpm
            );

            burstMaxPower = Math.max(
                    burstMaxPower,
                    shooterPower
            );

            if (!Double.isNaN(voltage)) {

                burstMinVoltage =
                        Double.isNaN(burstMinVoltage)
                                ? voltage
                                : Math.min(
                                burstMinVoltage,
                                voltage
                        );
            }
        }

        if (!feeding && wasFeeding) {

            lastBurstStartRpm = burstStartRpm;
            lastBurstMinRpm = burstMinRpm;
            lastBurstMaxRpm = burstMaxRpm;
        }

        wasFeeding = feeding;

        // ===================================
        // CONTINUOUS INTAKE / TRANSFER
        // ===================================

        if (gamepad1.left_trigger > 0.2) {

            intakeMotor.setPower(intakePower);
            transferMotor.setPower(transferPower);

        } else if (gamepad1.x) {

            intakeMotor.setPower(0.0);
            transferMotor.setPower(transferPower);

        } else {

            intakeMotor.setPower(0.0);
            transferMotor.setPower(0.0);
        }

        // ===================================
        // DRIVING
        // ===================================

        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        if (gamepad1.b) {
            imu.resetYaw();
        }

        butterfly.update(
                forward,
                strafe,
                rotate,
                gamepad1.right_bumper
        );

        driveFieldRelative(
                forward,
                strafe,
                rotate
        );

        // ===================================
        // TURRET
        // ===================================

        if (gamepad1.right_bumper) {

            rightTurretPosition = Math.min(
                    1.0,
                    rightTurretPosition + TURRET_RATE * dt
            );

            leftTurretPosition = Math.min(
                    1.0,
                    leftTurretPosition + TURRET_RATE * dt
            );

        } else if (gamepad1.left_bumper) {

            rightTurretPosition = Math.max(
                    0.0,
                    rightTurretPosition - TURRET_RATE * dt
            );

            leftTurretPosition = Math.max(
                    0.0,
                    leftTurretPosition - TURRET_RATE * dt
            );
        }

        leftTurretServo.setPosition(leftTurretPosition);
        rightTurretServo.setPosition(rightTurretPosition);

        // ===================================
        // BLOCKER
        // ===================================

        if (gamepad1.a && !lastA) {
            blockerOpen = !blockerOpen;
        }

        lastA = gamepad1.a;

        blockerServo.setPosition(
                blockerOpen
                        ? BLOCKER_OPEN
                        : BLOCKER_CLOSED
        );

        // ===================================
        // ASYNCHRONOUS CSV LOGGING
        // ===================================

        if (recordingStartNs == 0) {
            recordingStartNs = now;
        }

        CsvSample sample = new CsvSample();

        sample.timeSeconds =
                (now - recordingStartNs) * 1e-9;

        sample.rawRpm = rawRpm;

        sample.filteredRpm =
                speedController.filteredVel
                        * 60.0 / TICKS_PER_REV;

        sample.targetRpm =
                targetTicks * 60.0 / TICKS_PER_REV;

        sample.profiledRpm =
                speedController.profiledTarget
                        * 60.0 / TICKS_PER_REV;

        sample.errorRpm =
                (speedController.profiledTarget
                        - speedController.filteredVel)
                        * 60.0 / TICKS_PER_REV;

        sample.motorPower = shooterPower;

        sample.batteryVolts = voltage;
        sample.loopMs = dt * 1000.0;

        sample.shooterEnabled =
                shooterEnabled ? 1 : 0;

        sample.feeding = feeding ? 1 : 0;

        sample.rightTrigger =
                gamepad1.right_trigger;

        sample.leftTrigger =
                gamepad1.left_trigger;

        sample.xPressed =
                gamepad1.x ? 1 : 0;

        sample.burstId =
                feeding ? burstNumber : 0;

        logger.record(sample);

        // ===================================
        // TELEMETRY
        // ===================================

        if (now - lastTelemetryNs >= 200_000_000L) {

            telemetry.addData(
                    "Target RPM", "%.0f", TARGET_RPM
            );

            telemetry.addData(
                    "Actual RPM", "%.1f", rawRpm
            );

            telemetry.addData(
                    "Filtered RPM",
                    "%.1f",
                    sample.filteredRpm
            );

            telemetry.addData(
                    "Profiled RPM",
                    "%.1f",
                    sample.profiledRpm
            );

            telemetry.addData(
                    "RPM Error",
                    "%.1f",
                    sample.errorRpm
            );

            telemetry.addData(
                    "Shooter Power",
                    "%.3f",
                    shooterPower
            );

            telemetry.addData(
                    "Battery",
                    "%.2f V",
                    voltage
            );

            telemetry.addData(
                    "Loop Time",
                    "%.1f ms",
                    dt * 1000.0
            );

            telemetry.addData(
                    "Shooter Trigger",
                    "%.2f",
                    gamepad1.right_trigger
            );

            telemetry.addData(
                    "Shooter Enabled",
                    shooterEnabled ? "YES" : "NO"
            );

            telemetry.addData(
                    "Feeding",
                    feeding ? "YES" : "NO"
            );

            telemetry.addData(
                    "Burst #",
                    burstNumber
            );

            telemetry.addData(
                    "Burst Start RPM",
                    "%.1f",
                    burstStartRpm
            );

            telemetry.addData(
                    "Burst Min RPM",
                    "%.1f",
                    burstMinRpm
            );

            telemetry.addData(
                    "Burst Max RPM",
                    "%.1f",
                    burstMaxRpm
            );

            telemetry.addData(
                    "Burst Max Power",
                    "%.3f",
                    burstMaxPower
            );

            telemetry.addData(
                    "Last Burst Start",
                    "%.1f",
                    lastBurstStartRpm
            );

            telemetry.addData(
                    "Last Burst Min",
                    "%.1f",
                    lastBurstMinRpm
            );

            telemetry.addData(
                    "Last Burst Max",
                    "%.1f",
                    lastBurstMaxRpm
            );

            telemetry.addData(
                    "CSV Status",
                    logger.status
            );

            telemetry.addData(
                    "Samples Written",
                    logger.writtenSamples
            );

            telemetry.addData(
                    "Samples Dropped",
                    logger.droppedSamples.get()
            );

            telemetry.addData(
                    "Queue Size",
                    logger.queueSize()
            );

            if (logger.error != null) {

                telemetry.addData(
                        "CSV Error",
                        logger.error
                );
            }

            telemetry.addLine(
                    "D-pad UP/DOWN: +/-20 RPM"
            );

            telemetry.addLine(
                    "Continuous feed: no recovery boost"
            );

            telemetry.update();

            lastTelemetryNs = now;
        }
    }

    // ===================================
    // FIELD-RELATIVE DRIVING
    // ===================================

    private void driveFieldRelative(
            double forward,
            double right,
            double rotate
    ) {

        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        theta = AngleUnit.normalizeRadians(
                theta - imu.getRobotYawPitchRollAngles()
                        .getYaw(AngleUnit.RADIANS)
        );

        drive(
                r * Math.sin(theta),
                r * Math.cos(theta),
                rotate
        );
    }

    private void drive(
            double forward,
            double right,
            double rotate
    ) {

        double fl = forward + right + rotate;
        double fr = forward - right - rotate;
        double br = forward + right - rotate;
        double bl = forward - right + rotate;

        double max = Math.max(
                1.0,
                Math.max(
                        Math.max(Math.abs(fl), Math.abs(fr)),
                        Math.max(Math.abs(bl), Math.abs(br))
                )
        );

        frontLeftDrive.setPower(fl / max);
        frontRightDrive.setPower(fr / max);
        backLeftDrive.setPower(bl / max);
        backRightDrive.setPower(br / max);
    }

    // ===================================
    // STOP
    // ===================================

    @Override
    public void stop() {

        // Stop hardware immediately.
        shooterMotor.setPower(0.0);
        intakeMotor.setPower(0.0);
        transferMotor.setPower(0.0);

        frontLeftDrive.setPower(0.0);
        frontRightDrive.setPower(0.0);
        backLeftDrive.setPower(0.0);
        backRightDrive.setPower(0.0);

        speedController.reset();

        // Signal the CSV thread without waiting
        // for thousands of disk writes.
        logger.stopAsync();

        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(
                    LynxModule.BulkCachingMode.AUTO
            );
        }
    }

    // ===================================
    // ORIGINAL SOFTWARE PIDF
    // ===================================

    public static class SpeedController {

        private double integral = 0.0;
        private double filteredVel = 0.0;
        private double profiledTarget = 0.0;

        private long lastTimeNs = -1;

        double calculate(
                double target,
                double measured,
                double voltageScale,
                long now
        ) {

            if (lastTimeNs < 0) {

                lastTimeNs = now;

                filteredVel = measured;
                profiledTarget = measured;

                return 0.0;
            }

            double dt =
                    (now - lastTimeNs) * 1e-9;

            lastTimeNs = now;

            if (dt <= 0.0) {
                return 0.0;
            }

            double previousVel = filteredVel;

            double alpha = clamp(
                    VELOCITY_ALPHA,
                    0.0,
                    1.0
            );

            filteredVel =
                    alpha * filteredVel
                            + (1.0 - alpha) * measured;

            double previousTarget = profiledTarget;

            double maxDelta =
                    Math.max(0.0, MAX_ACCEL) * dt;

            profiledTarget += clamp(
                    target - profiledTarget,
                    -maxDelta,
                    maxDelta
            );

            double accel =
                    (profiledTarget - previousTarget) / dt;

            double error =
                    profiledTarget - filteredVel;

            double derivative =
                    -(filteredVel - previousVel) / dt;

            // Original integral accumulation rule.
            if (Math.abs(kP * error) < 1.0) {

                double limit =
                        Math.abs(MAX_INTEGRAL);

                integral = clamp(
                        integral + error * dt,
                        -limit,
                        limit
                );
            }

            double staticTerm =
                    Math.abs(profiledTarget) > KS_DEADBAND
                            ? Math.copySign(kS, profiledTarget)
                            : 0.0;

            double output =
                    (
                            kF * profiledTarget
                                    + staticTerm
                                    + kA * accel
                                    + kP * error
                                    + kI * integral
                                    + kD * derivative
                    ) * voltageScale;

            return clamp(
                    output,
                    -1.0,
                    1.0
            );
        }

        void reset() {

            integral = 0.0;
            filteredVel = 0.0;
            profiledTarget = 0.0;

            lastTimeNs = -1;
        }
    }

    // ===================================
    // CSV SAMPLE
    // ===================================

    private static class CsvSample {

        double timeSeconds;

        double rawRpm;
        double filteredRpm;

        double targetRpm;
        double profiledRpm;
        double errorRpm;

        double motorPower;
        double batteryVolts;
        double loopMs;

        double rightTrigger;
        double leftTrigger;

        int xPressed;
        int shooterEnabled;
        int feeding;
        int burstId;
    }

    // ===================================
    // NONBLOCKING CSV LOGGER
    // ===================================

    private static class AsyncCsvLogger {

        private static final int QUEUE_CAPACITY = 4096;

        private final ArrayBlockingQueue<CsvSample> queue =
                new ArrayBlockingQueue<>(QUEUE_CAPACITY);

        final AtomicInteger droppedSamples =
                new AtomicInteger(0);

        volatile long writtenSamples = 0;

        volatile String status = "NOT STARTED";
        volatile String error = null;

        private volatile boolean stopping = false;
        private volatile boolean accepting = false;

        private Thread writerThread;

        void start(File baseDirectory) {

            if (writerThread != null) {
                return;
            }

            stopping = false;
            accepting = true;

            status = "STARTING";

            writerThread = new Thread(() -> {

                try {

                    if (baseDirectory == null) {
                        throw new IOException(
                                "External storage unavailable"
                        );
                    }

                    File folder = new File(
                            baseDirectory,
                            "shooter_diagnostics"
                    );

                    if (!folder.exists()
                            && !folder.mkdirs()) {

                        throw new IOException(
                                "Could not create directory"
                        );
                    }

                    File file = new File(
                            folder,
                            "rapidfire_baseline_"
                                    + System.currentTimeMillis()
                                    + ".csv"
                    );

                    try (BufferedWriter writer =
                                 new BufferedWriter(
                                         new FileWriter(file),
                                         32768
                                 )) {

                        writer.write(
                                "time_s,raw_rpm,filtered_rpm,"
                                        + "target_rpm,profiled_rpm,"
                                        + "error_rpm,motor_power,"
                                        + "battery_v,loop_ms,"
                                        + "right_trigger,left_trigger,"
                                        + "x_pressed,shooter_enabled,"
                                        + "feeding,burst_id\n"
                        );

                        writer.flush();

                        status = "RECORDING";

                        StringBuilder row =
                                new StringBuilder(256);

                        long lastFlushNs =
                                System.nanoTime();

                        while (!stopping || !queue.isEmpty()) {

                            CsvSample s;

                            try {

                                s = queue.poll(
                                        100,
                                        TimeUnit.MILLISECONDS
                                );

                            } catch (InterruptedException e) {
                                continue;
                            }

                            if (s != null) {

                                row.setLength(0);

                                row.append(s.timeSeconds).append(',');
                                row.append(s.rawRpm).append(',');
                                row.append(s.filteredRpm).append(',');
                                row.append(s.targetRpm).append(',');
                                row.append(s.profiledRpm).append(',');
                                row.append(s.errorRpm).append(',');
                                row.append(s.motorPower).append(',');
                                row.append(s.batteryVolts).append(',');
                                row.append(s.loopMs).append(',');
                                row.append(s.rightTrigger).append(',');
                                row.append(s.leftTrigger).append(',');
                                row.append(s.xPressed).append(',');
                                row.append(s.shooterEnabled).append(',');
                                row.append(s.feeding).append(',');
                                row.append(s.burstId).append('\n');

                                writer.write(row.toString());

                                writtenSamples++;
                            }

                            long currentNs =
                                    System.nanoTime();

                            if (currentNs - lastFlushNs
                                    >= 500_000_000L) {

                                writer.flush();
                                lastFlushNs = currentNs;
                            }
                        }

                        status = "FINALIZING";
                        writer.flush();
                    }

                    status = "SAVED";

                    android.util.Log.i(
                            "RapidFireBaseline",
                            "Saved CSV: " + file.getAbsolutePath()
                    );

                } catch (IOException e) {

                    error = e.toString();
                    status = "ERROR";

                    android.util.Log.e(
                            "RapidFireBaseline",
                            "CSV write failed",
                            e
                    );

                } catch (RuntimeException e) {

                    error = e.toString();
                    status = "ERROR";

                    android.util.Log.e(
                            "RapidFireBaseline",
                            "Logger error",
                            e
                    );

                } finally {

                    accepting = false;
                }

            }, "RapidFireBaselineWriter");

            writerThread.setDaemon(true);
            writerThread.start();
        }

        void record(CsvSample sample) {

            if (!accepting || stopping) {
                return;
            }

            // No disk I/O on control loop.
            if (!queue.offer(sample)) {
                droppedSamples.incrementAndGet();
            }
        }

        int queueSize() {
            return queue.size();
        }

        void stopAsync() {

            accepting = false;
            stopping = true;

            // No join or flush on FTC thread.
            status = "STOP REQUESTED";
        }
    }

    // ===================================
    // UTILITIES
    // ===================================

    private static double clamp(
            double value,
            double min,
            double max
    ) {

        return Math.max(
                min,
                Math.min(max, value)
        );
    }
}
