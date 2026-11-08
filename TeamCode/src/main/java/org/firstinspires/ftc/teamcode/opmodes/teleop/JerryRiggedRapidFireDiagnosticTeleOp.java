
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
        name = "Jerry Rigged Rapid Fire DIAGNOSTIC V4.1",
        group = "Robot"
)
public class JerryRiggedRapidFireDiagnosticTeleOp extends OpMode {

    // ========================================
    // SHOOTER SETTINGS
    // ========================================

    public static double intakePower = 0.6;
    public static double transferPower = 0.8;

    private static final double TICKS_PER_REV = 28.0;

    public static double TARGET_RPM = 1410.0;
    public static double RPM_STEP = 20.0;

    public static double kF = 0.0006825;
    public static double kS = 0.069;

    public static double kP = 0.0015;
    public static double kI = 0.00002;
    public static double kD = 0.0;

    public static double RECOVERY_KP = 0.0030;
    public static double MAX_RECOVERY_BOOST = 0.12;

    public static double RECOVERY_ON_RPM = 35.0;
    public static double RECOVERY_OFF_RPM = 15.0;
    public static double FULL_RECOVERY_RPM = 120.0;

    public static double FAST_TAU = 0.010;
    public static double NORMAL_TAU = 0.035;

    public static double NOMINAL_VOLTAGE = 12.0;
    public static double MAX_ACCEL = 6000.0;
    public static double MAX_INTEGRAL = 5000.0;

    public static double READY_FRACTION = 0.98;
    public static double TURRET_RATE = 0.25;

    // ========================================
    // HARDWARE
    // ========================================

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

    // ========================================
    // TURRET AND BLOCKER
    // ========================================

    private double leftTurretPosition = 0.5;
    private double rightTurretPosition = 0.5;

    private static final double BLOCKER_OPEN = 0.032;
    private static final double BLOCKER_CLOSED = 0.0;

    private boolean blockerOpen = false;
    private boolean lastA = false;

    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;

    // ========================================
    // CONTROLLER
    // ========================================

    private final ShooterController controller =
            new ShooterController();

    private long lastLoopNs = 0;
    private long lastVoltageNs = 0;
    private long lastTelemetryNs = 0;

    private double voltage = Double.NaN;
    private double voltageScale = 1.0;

    // ========================================
    // ASYNCHRONOUS CSV LOGGER
    // ========================================

    private final AsyncCsvLogger csvLogger =
            new AsyncCsvLogger();

    private long recordingStartNs = 0;

    // ========================================
    // BURST STATISTICS
    // ========================================

    private boolean wasFeeding = false;

    private int burstNumber = 0;

    private double burstStartRpm = Double.NaN;
    private double burstMinRpm = Double.NaN;
    private double burstMaxRpm = Double.NaN;

    private double burstMaxPower = 0.0;
    private double burstMaxRecovery = 0.0;

    private long burstStartNs = 0;

    // ========================================
    // INITIALIZATION
    // ========================================

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

    // ========================================
    // START
    // ========================================

    @Override
    public void start() {

        super.start();

        lastLoopNs = 0;
        lastVoltageNs = 0;
        lastTelemetryNs = 0;

        recordingStartNs = 0;

        wasFeeding = false;
        burstNumber = 0;

        controller.reset();

        // Creates and writes the CSV on another thread.
        // No disk writing occurs here.

        csvLogger.start(
                hardwareMap.appContext.getExternalFilesDir(null)
        );
    }

    // ========================================
    // MAIN LOOP
    // ========================================

    @Override
    public void loop() {

        long now = System.nanoTime();

        double dt = lastLoopNs == 0
                ? 0.01
                : clamp(
                (now - lastLoopNs) * 1e-9,
                0.0001,
                0.1
        );

        lastLoopNs = now;

        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }

        double measuredTicks =
                shooterMotor.getVelocity();

        double rawRpm =
                measuredTicks * 60.0 / TICKS_PER_REV;

        // ====================================
        // TARGET RPM ADJUSTMENT
        // ====================================

        if (gamepad1.dpad_up && !lastDpadUp) {

            TARGET_RPM = clamp(
                    TARGET_RPM + RPM_STEP,
                    1100,
                    1700
            );
        }

        if (gamepad1.dpad_down && !lastDpadDown) {

            TARGET_RPM = clamp(
                    TARGET_RPM - RPM_STEP,
                    1100,
                    1700
            );
        }

        lastDpadUp = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;

        // ====================================
        // FEED INPUT
        // ====================================

        boolean feeding =
                gamepad1.left_trigger > 0.2
                        || gamepad1.x;

        boolean shooterEnabled =
                gamepad1.right_trigger > 0.2;

        // ====================================
        // BATTERY
        // ====================================

        long voltageInterval = feeding
                ? 100_000_000L
                : 250_000_000L;

        if (lastVoltageNs == 0
                || now - lastVoltageNs >= voltageInterval) {

            double v = voltageSensor == null
                    ? 0.0
                    : voltageSensor.getVoltage();

            voltage = v > 1.0
                    ? v
                    : Double.NaN;

            voltageScale = v > 1.0
                    ? NOMINAL_VOLTAGE / v
                    : 1.0;

            lastVoltageNs = now;
        }

        // ====================================
        // SHOOTER PIDF
        // ====================================

        double targetTicks = shooterEnabled
                ? TARGET_RPM * TICKS_PER_REV / 60.0
                : 0.0;

        double shooterPower;

        if (shooterEnabled) {

            shooterPower = controller.calculate(
                    targetTicks,
                    measuredTicks,
                    voltageScale,
                    now
            );

        } else {

            controller.reset();
            shooterPower = 0.0;
        }

        shooterMotor.setPower(shooterPower);

        // ====================================
        // BURST STATISTICS
        // ====================================

        if (feeding && !wasFeeding) {

            burstNumber++;
            burstStartNs = now;

            burstStartRpm = rawRpm;
            burstMinRpm = rawRpm;
            burstMaxRpm = rawRpm;

            burstMaxPower = 0.0;
            burstMaxRecovery = 0.0;
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

            burstMaxRecovery = Math.max(
                    burstMaxRecovery,
                    controller.recoveryPower
            );
        }

        wasFeeding = feeding;

        // ====================================
        // CONTINUOUS INTAKE / TRANSFER
        // ====================================

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

        // ====================================
        // DRIVING
        // ====================================

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

        // ====================================
        // TURRET
        // ====================================

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

        // ====================================
        // BLOCKER
        // ====================================

        if (gamepad1.a && !lastA) {
            blockerOpen = !blockerOpen;
        }

        lastA = gamepad1.a;

        blockerServo.setPosition(
                blockerOpen ? BLOCKER_OPEN : BLOCKER_CLOSED
        );

        // ====================================
        // NONBLOCKING CSV SAMPLE
        // ====================================

        if (recordingStartNs == 0) {
            recordingStartNs = now;
        }

        CsvSample sample = new CsvSample();

        sample.timeSeconds =
                (now - recordingStartNs) * 1e-9;

        sample.rawRpm = rawRpm;

        sample.fastRpm =
                controller.fastVelocity
                        * 60.0 / TICKS_PER_REV;

        sample.normalRpm =
                controller.normalVelocity
                        * 60.0 / TICKS_PER_REV;

        sample.targetRpm =
                targetTicks * 60.0 / TICKS_PER_REV;

        sample.profiledRpm =
                controller.profiledTarget
                        * 60.0 / TICKS_PER_REV;

        sample.motorPower = shooterPower;

        sample.basePower = controller.basePower;
        sample.pidPower = controller.pidPower;
        sample.recoveryPower = controller.recoveryPower;
        sample.effectiveKp = controller.effectiveKp;

        sample.batteryVolts = voltage;
        sample.loopMs = dt * 1000.0;

        sample.feeding = feeding ? 1 : 0;
        sample.shooterEnabled = shooterEnabled ? 1 : 0;
        sample.recoveryActive =
                controller.recoveryActive ? 1 : 0;

        sample.burstId = feeding ? burstNumber : 0;

        // offer() does not wait for disk writes.
        csvLogger.record(sample);

        // ====================================
        // DRIVER STATION TELEMETRY
        // ====================================

        if (now - lastTelemetryNs >= 200_000_000L) {

            telemetry.addData(
                    "Target RPM",
                    "%.0f",
                    TARGET_RPM
            );

            telemetry.addData(
                    "Actual RPM",
                    "%.1f",
                    rawRpm
            );

            telemetry.addData(
                    "Fast RPM",
                    "%.1f",
                    sample.fastRpm
            );

            telemetry.addData(
                    "Normal RPM",
                    "%.1f",
                    sample.normalRpm
            );

            telemetry.addData(
                    "Base Power",
                    "%.3f",
                    controller.basePower
            );

            telemetry.addData(
                    "PID Power",
                    "%.3f",
                    controller.pidPower
            );

            telemetry.addData(
                    "Recovery Power",
                    "%.3f",
                    controller.recoveryPower
            );

            telemetry.addData(
                    "Final Power",
                    "%.3f",
                    shooterPower
            );

            telemetry.addData(
                    "Effective kP",
                    "%.5f",
                    controller.effectiveKp
            );

            telemetry.addData(
                    "Recovery",
                    controller.recoveryActive
                            ? "ACTIVE" : "off"
            );

            telemetry.addData(
                    "Battery",
                    "%.2f V",
                    voltage
            );

            telemetry.addData(
                    "Loop",
                    "%.1f ms",
                    dt * 1000.0
            );

            telemetry.addData(
                    "Shooter trigger",
                    "%.2f",
                    gamepad1.right_trigger
            );

            telemetry.addData(
                    "Shooter enabled",
                    shooterEnabled ? "YES" : "NO"
            );

            telemetry.addData(
                    "Feeding",
                    feeding ? "YES" : "NO"
            );

            telemetry.addData(
                    "Burst",
                    burstNumber
            );

            telemetry.addData(
                    "Burst start RPM",
                    "%.1f",
                    burstStartRpm
            );

            telemetry.addData(
                    "Burst min RPM",
                    "%.1f",
                    burstMinRpm
            );

            telemetry.addData(
                    "Burst max RPM",
                    "%.1f",
                    burstMaxRpm
            );

            telemetry.addData(
                    "Burst max power",
                    "%.3f",
                    burstMaxPower
            );

            telemetry.addData(
                    "Burst max recovery",
                    "%.3f",
                    burstMaxRecovery
            );

            telemetry.addData(
                    "CSV status",
                    csvLogger.status
            );

            telemetry.addData(
                    "CSV written samples",
                    csvLogger.writtenSamples
            );

            telemetry.addData(
                    "CSV queued samples",
                    csvLogger.queueSize()
            );

            telemetry.addData(
                    "CSV dropped samples",
                    csvLogger.droppedSamples.get()
            );

            if (csvLogger.error != null) {

                telemetry.addData(
                        "CSV ERROR",
                        csvLogger.error
                );
            }

            telemetry.addLine(
                    "D-pad up/down: +/-20 RPM"
            );

            telemetry.addLine(
                    "STOP safely closes CSV in background"
            );

            telemetry.update();

            lastTelemetryNs = now;
        }
    }

    // ========================================
    // DRIVE METHODS
    // ========================================

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

    // ========================================
    // STOP: NO BLOCKING FILE I/O
    // ========================================

    @Override
    public void stop() {

        // Stop motors FIRST.

        shooterMotor.setPower(0.0);
        intakeMotor.setPower(0.0);
        transferMotor.setPower(0.0);

        frontLeftDrive.setPower(0.0);
        frontRightDrive.setPower(0.0);
        backLeftDrive.setPower(0.0);
        backRightDrive.setPower(0.0);

        controller.reset();

        // Only signals the background writer.
        // Does NOT wait for the file to finish.

        csvLogger.stopAsync();

        for (LynxModule hub : hubs) {

            hub.setBulkCachingMode(
                    LynxModule.BulkCachingMode.AUTO
            );
        }
    }

    // ========================================
    // CSV SAMPLE OBJECT
    // ========================================

    private static class CsvSample {

        double timeSeconds;

        double rawRpm;
        double fastRpm;
        double normalRpm;

        double targetRpm;
        double profiledRpm;

        double motorPower;

        double basePower;
        double pidPower;
        double recoveryPower;

        double effectiveKp;

        double batteryVolts;
        double loopMs;

        int feeding;
        int shooterEnabled;
        int recoveryActive;
        int burstId;
    }

    // ========================================
    // BACKGROUND CSV LOGGER
    // ========================================

    private static class AsyncCsvLogger {

        private static final int QUEUE_CAPACITY = 4096;

        private static final long FLUSH_INTERVAL_NS =
                500_000_000L;

        private final ArrayBlockingQueue<CsvSample> queue =
                new ArrayBlockingQueue<>(QUEUE_CAPACITY);

        final AtomicInteger droppedSamples =
                new AtomicInteger(0);

        volatile long writtenSamples = 0;

        volatile String status = "NOT STARTED";
        volatile String error = null;
        volatile String savedPath = null;

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
                                "Cannot create CSV directory"
                        );
                    }

                    File file = new File(
                            folder,
                            "rapidfire_v4_1_"
                                    + System.currentTimeMillis()
                                    + ".csv"
                    );

                    savedPath = file.getAbsolutePath();

                    try (BufferedWriter writer =
                                 new BufferedWriter(
                                         new FileWriter(file),
                                         32768
                                 )) {

                        writer.write(
                                "time_s,raw_rpm,fast_rpm,"
                                        + "normal_rpm,target_rpm,"
                                        + "profiled_rpm,motor_power,"
                                        + "base_power,pid_power,"
                                        + "recovery_power,effective_kp,"
                                        + "battery_v,loop_ms,feeding,"
                                        + "shooter_enabled,recovery_active,"
                                        + "burst_id\n"
                        );

                        writer.flush();

                        status = "RECORDING";

                        long lastFlush = System.nanoTime();

                        // Reuse builder outside the motor loop.

                        StringBuilder row =
                                new StringBuilder(256);

                        // Drain remaining samples after STOP.
                        while (!stopping || !queue.isEmpty()) {

                            CsvSample s;

                            try {

                                s = queue.poll(
                                        100,
                                        TimeUnit.MILLISECONDS
                                );

                            } catch (InterruptedException e) {

                                // A shutdown interrupt must
                                // not discard queued samples.
                                continue;
                            }

                            if (s != null) {

                                row.setLength(0);

                                row.append(s.timeSeconds).append(',');
                                row.append(s.rawRpm).append(',');
                                row.append(s.fastRpm).append(',');
                                row.append(s.normalRpm).append(',');

                                row.append(s.targetRpm).append(',');
                                row.append(s.profiledRpm).append(',');

                                row.append(s.motorPower).append(',');

                                row.append(s.basePower).append(',');
                                row.append(s.pidPower).append(',');
                                row.append(s.recoveryPower).append(',');

                                row.append(s.effectiveKp).append(',');
                                row.append(s.batteryVolts).append(',');
                                row.append(s.loopMs).append(',');

                                row.append(s.feeding).append(',');
                                row.append(s.shooterEnabled).append(',');
                                row.append(s.recoveryActive).append(',');
                                row.append(s.burstId).append('\n');

                                writer.write(row.toString());

                                writtenSamples++;
                            }

                            long now = System.nanoTime();

                            if (now - lastFlush >= FLUSH_INTERVAL_NS) {

                                writer.flush();
                                lastFlush = now;
                            }
                        }

                        // Final flush is done off the
                        // FTC OpMode thread.

                        status = "FINALIZING";

                        writer.flush();
                    }

                    status = "SAVED";

                    android.util.Log.i(
                            "RapidFireV4",
                            "CSV saved: " + savedPath
                    );

                } catch (IOException e) {

                    error = e.toString();
                    status = "ERROR";

                    android.util.Log.e(
                            "RapidFireV4",
                            "CSV writer failed",
                            e
                    );

                } catch (RuntimeException e) {

                    error = e.toString();
                    status = "ERROR";

                    android.util.Log.e(
                            "RapidFireV4",
                            "Unexpected CSV writer error",
                            e
                    );

                } finally {

                    accepting = false;
                }

            }, "RapidFireCsvWriter");

            // This prevents the diagnostic thread from
            // keeping the Robot Controller app alive.

            writerThread.setDaemon(true);

            writerThread.start();
        }

        void record(CsvSample sample) {

            if (!accepting || stopping) {
                droppedSamples.incrementAndGet();
                return;
            }

            // offer() returns immediately.
            // Never use put() here because it blocks.

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

            // No join(), no disk flush here.
            // The background thread drains and exits.

            status = "STOP REQUESTED";
        }
    }

    // ========================================
    // V4 SHOOTER RECOVERY CONTROLLER
    // ========================================

    public static class ShooterController {

        private double fastVelocity = 0.0;
        private double normalVelocity = 0.0;

        private double profiledTarget = 0.0;

        private double integral = 0.0;

        private double basePower = 0.0;
        private double pidPower = 0.0;
        private double recoveryPower = 0.0;

        private double effectiveKp = kP;

        private boolean recoveryActive = false;

        private long lastTimeNs = -1;

        double calculate(
                double target,
                double measured,
                double voltageScale,
                long now
        ) {

            if (lastTimeNs < 0) {

                lastTimeNs = now;

                fastVelocity = measured;
                normalVelocity = measured;

                profiledTarget = measured;

                return 0.0;
            }

            double dt = clamp(
                    (now - lastTimeNs) * 1e-9,
                    0.0001,
                    0.1
            );

            lastTimeNs = now;

            // =================================
            // FAST AND NORMAL VELOCITY FILTERS
            // =================================

            double fastAlpha = Math.exp(
                    -dt / Math.max(0.001, FAST_TAU)
            );

            double normalAlpha = Math.exp(
                    -dt / Math.max(0.001, NORMAL_TAU)
            );

            double previousNormal = normalVelocity;

            fastVelocity =
                    fastAlpha * fastVelocity
                            + (1.0 - fastAlpha) * measured;

            normalVelocity =
                    normalAlpha * normalVelocity
                            + (1.0 - normalAlpha) * measured;

            // =================================
            // PROFILE TARGET
            // =================================

            double maxDelta =
                    Math.max(0.0, MAX_ACCEL) * dt;

            profiledTarget += clamp(
                    target - profiledTarget,
                    -maxDelta,
                    maxDelta
            );

            // =================================
            // VELOCITY ERRORS
            // =================================

            double normalError =
                    profiledTarget - normalVelocity;

            double fastError =
                    target - fastVelocity;

            double normalErrorRpm =
                    normalError * 60.0 / TICKS_PER_REV;

            double fastErrorRpm =
                    fastError * 60.0 / TICKS_PER_REV;

            // =================================
            // VOLTAGE-COMPENSATED FEEDFORWARD
            // =================================

            double staticPower =
                    profiledTarget > 50.0 ? kS : 0.0;

            basePower = (
                    kF * profiledTarget + staticPower
            ) * voltageScale;

            // =================================
            // RECOVERY HYSTERESIS
            // =================================

            boolean atTarget =
                    target > 0.0
                            && profiledTarget
                            >= target * READY_FRACTION;

            if (!atTarget) {

                recoveryActive = false;

            } else if (
                    !recoveryActive
                            && fastErrorRpm >= RECOVERY_ON_RPM
            ) {

                recoveryActive = true;

            } else if (
                    recoveryActive
                            && fastErrorRpm <= RECOVERY_OFF_RPM
            ) {

                recoveryActive = false;
            }

            // =================================
            // ADAPTIVE KP
            // =================================

            double recoveryFraction = 0.0;

            if (recoveryActive) {

                recoveryFraction = clamp(
                        (normalErrorRpm - RECOVERY_OFF_RPM)
                                / Math.max(
                                1.0,
                                FULL_RECOVERY_RPM - RECOVERY_OFF_RPM
                        ),
                        0.0,
                        1.0
                );
            }

            effectiveKp =
                    kP
                            + Math.max(
                            0.0,
                            RECOVERY_KP - kP
                    ) * recoveryFraction;

            // =================================
            // BOUNDED RECOVERY BOOST
            // =================================

            if (recoveryActive) {

                recoveryPower =
                        MAX_RECOVERY_BOOST * clamp(
                                (fastErrorRpm - RECOVERY_OFF_RPM)
                                        / Math.max(
                                        1.0,
                                        FULL_RECOVERY_RPM
                                                - RECOVERY_OFF_RPM
                                ),
                                0.0,
                                1.0
                        );

            } else {

                recoveryPower = 0.0;
            }

            // =================================
            // PID FEEDBACK
            // =================================

            double derivative =
                    -(normalVelocity - previousNormal) / dt;

            double feedbackWithoutI =
                    effectiveKp * normalError
                            + kD * derivative;

            // =================================
            // CONDITIONAL ANTI-WINDUP
            // =================================

            double candidateIntegral = clamp(
                    integral + normalError * dt,
                    -Math.abs(MAX_INTEGRAL),
                    Math.abs(MAX_INTEGRAL)
            );

            double candidatePower =
                    basePower
                            + feedbackWithoutI
                            + kI * candidateIntegral
                            + recoveryPower;

            boolean saturatingHigh =
                    candidatePower > 1.0
                            && normalError > 0.0;

            boolean saturatingLow =
                    candidatePower < 0.0
                            && normalError < 0.0;

            if (!saturatingHigh && !saturatingLow) {
                integral = candidateIntegral;
            }

            pidPower =
                    feedbackWithoutI
                            + kI * integral;

            return clamp(
                    basePower + pidPower + recoveryPower,
                    0.0,
                    1.0
            );
        }

        void reset() {

            fastVelocity = 0.0;
            normalVelocity = 0.0;

            profiledTarget = 0.0;

            integral = 0.0;

            basePower = 0.0;
            pidPower = 0.0;
            recoveryPower = 0.0;

            effectiveKp = kP;
            recoveryActive = false;

            lastTimeNs = -1;
        }
    }

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
