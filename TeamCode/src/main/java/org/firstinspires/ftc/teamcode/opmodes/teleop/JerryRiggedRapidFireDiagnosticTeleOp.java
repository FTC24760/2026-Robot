
package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.List;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import android.util.Log;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.ButterflyController;

@TeleOp(name = "Jerry Rigged PIDF Baseline Logger", group = "Robot")
public class JerryRiggedRapidFireDiagnosticTeleOp extends OpMode {

    public static double intakePower = 0.6;
    public static double transferPower = 0.8;

    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backLeftDrive;
    private DcMotor backRightDrive;
    private DcMotor intakeMotor;
    private DcMotor transferMotor;

    private IMU imu;
    private ButterflyController butterfly;

    Servo leftTurretServo;
    Servo rightTurretServo;
    Servo blockerServo;

    double leftTurretServoPosition = 0.5;
    double rightTurretServoPosition = 0.5;
    double blockerOpenPosition = 0.032;
    double blockerClosedPosition = 0.0;
    private boolean blockerOpen = false;
    private boolean lastAPressed = false;

    // SHOOTER
    private DcMotorEx shooterMotor;

    // goBILDA 5000 Series (no gearbox)
    private static final double TICKS_PER_REV = 28.0;

    // Original working PIDF settings
    public static double TARGET_RPM = 1550;

    public static double kF = 0.00089;
    public static double kS = 0.069;
    public static double kP = 0.0051;
    public static double kI = 0.00002;
    public static double kD = 0.00005;
    public static double kA = 0.0;

    public static double MAX_INTEGRAL = 5000;
    public static double NOMINAL_VOLTAGE = 12.0;
    public static double MAX_ACCEL = 6000.0;
    public static double VELOCITY_ALPHA = 0.7;
    public static double KS_DEADBAND = 50.0;
    public static double TURRET_RATE = 0.25;

    private List<LynxModule> hubs;
    private VoltageSensor voltageSensor;
    private long lastVoltageReadNs;
    private long lastLoopNs;
    private double voltageScale = 1.0;

    private final SpeedController speedController =
            new SpeedController();

    // =========================================
    // MINIMAL ASYNC RPM LOGGING
    // =========================================

    // Approximately 50 samples per second.
    // No file I/O in the robot control loop.
    private static final long RPM_LOG_PERIOD_NS = 20_000_000L;
    private static final long TELEMETRY_PERIOD_NS = 200_000_000L;

    private long lastRpmLoggedNs = 0;
    private long lastTelemetryNs = 0;
    private long recordingStartNs = 0;

    private final AsyncRpmLogger rpmLogger =
            new AsyncRpmLogger();

    @Override
    public void init() {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "lf");
        frontRightDrive = hardwareMap.get(DcMotor.class, "rf");
        backLeftDrive = hardwareMap.get(DcMotor.class, "lb");
        backRightDrive = hardwareMap.get(DcMotor.class, "rb");

        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        transferMotor = hardwareMap.get(DcMotor.class, "transfer");

        leftTurretServo = hardwareMap.get(Servo.class, "leftturret");
        rightTurretServo = hardwareMap.get(Servo.class, "rightturret");
        blockerServo = hardwareMap.get(Servo.class, "blocker");

        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        transferMotor.setDirection(DcMotor.Direction.REVERSE);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");

        // Software PIDF, not the Hub velocity controller.
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        voltageSensor = hardwareMap.voltageSensor.iterator().hasNext()
                ? hardwareMap.voltageSensor.iterator().next()
                : null;

        shooterMotor.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.FLOAT
        );

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot orientationOnRobot =
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                );

        imu.initialize(new IMU.Parameters(orientationOnRobot));

        butterfly = new ButterflyController(hardwareMap);

        blockerServo.setPosition(blockerClosedPosition);
    }

    @Override
    public void start() {
        recordingStartNs = System.nanoTime();
        lastRpmLoggedNs = 0;
        lastTelemetryNs = 0;

        rpmLogger.start(
                hardwareMap.appContext.getExternalFilesDir(null)
        );
    }

    @Override
    public void loop() {

        long now = System.nanoTime();

        double loopSeconds = lastLoopNs == 0
                ? 0
                : Math.min((now - lastLoopNs) * 1e-9, 0.1);

        lastLoopNs = now;

        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }

        // Exactly one shooter velocity read per loop.
        // Reused for both PIDF and logging.
        double actualTicksPerSecond = shooterMotor.getVelocity();

        boolean shooterEnabled =
                gamepad1.right_trigger > 0.2;

        double targetTicksPerSecond = shooterEnabled
                ? TARGET_RPM * TICKS_PER_REV / 60.0
                : 0.0;

        if (lastVoltageReadNs == 0
                || now - lastVoltageReadNs >= 250_000_000L) {

            double voltage = voltageSensor == null
                    ? 0.0
                    : voltageSensor.getVoltage();

            voltageScale = voltage > 0.1
                    ? NOMINAL_VOLTAGE / voltage
                    : 1.0;

            lastVoltageReadNs = now;
        }

        // =====================================
        // ORIGINAL SOFTWARE PIDF
        // =====================================

        double shooterPower;

        if (shooterEnabled) {
            shooterPower = speedController.calculate(
                    targetTicksPerSecond,
                    actualTicksPerSecond,
                    voltageScale,
                    now
            );
        } else {
            speedController.reset();
            shooterPower = 0.0;
        }

        shooterMotor.setPower(shooterPower);

        // =====================================
        // ORIGINAL TURRET / DRIVE CONTROLS
        // =====================================

        leftTurretServo.setPosition(leftTurretServoPosition);
        rightTurretServo.setPosition(rightTurretServoPosition);

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

        driveFieldRelative(forward, strafe, rotate);

        if (gamepad1.right_bumper) {
            if (rightTurretServoPosition < 1.0) {
                rightTurretServoPosition = Math.min(
                        1.0,
                        rightTurretServoPosition
                                + TURRET_RATE * loopSeconds
                );

                leftTurretServoPosition = Math.min(
                        1.0,
                        leftTurretServoPosition
                                + TURRET_RATE * loopSeconds
                );
            }
        } else if (gamepad1.left_bumper) {
            if (rightTurretServoPosition > 0.0) {
                rightTurretServoPosition = Math.max(
                        0.0,
                        rightTurretServoPosition
                                - TURRET_RATE * loopSeconds
                );

                leftTurretServoPosition = Math.max(
                        0.0,
                        leftTurretServoPosition
                                - TURRET_RATE * loopSeconds
                );
            }
        }

        // =====================================
        // ORIGINAL CONTINUOUS FEED CONTROLS
        // =====================================

        if (gamepad1.left_trigger > 0.2) {
            intakeMotor.setPower(intakePower);
            transferMotor.setPower(transferPower);

        } else if (gamepad1.x) {
            intakeMotor.setPower(0);
            transferMotor.setPower(transferPower);

        } else {
            intakeMotor.setPower(0);
            transferMotor.setPower(0);
        }

        if (gamepad1.a && !lastAPressed) {
            blockerOpen = !blockerOpen;
        }

        lastAPressed = gamepad1.a;

        blockerServo.setPosition(
                blockerOpen
                        ? blockerOpenPosition
                        : blockerClosedPosition
        );

        // =====================================
        // MINIMAL RPM LOGGING
        // =====================================

        double actualRPM =
                actualTicksPerSecond * 60.0 / TICKS_PER_REV;

        double targetRPM =
                targetTicksPerSecond * 60.0 / TICKS_PER_REV;

        // No second encoder read, string formatting,
        // disk I/O, or locking inside this block.
        if (lastRpmLoggedNs == 0
                || now - lastRpmLoggedNs >= RPM_LOG_PERIOD_NS) {

            boolean feeding =
                    gamepad1.left_trigger > 0.2 || gamepad1.x;

            rpmLogger.record(
                    (now - recordingStartNs) * 1e-9,
                    actualRPM,
                    targetRPM,
                    feeding
            );

            lastRpmLoggedNs = now;
        }

        // Limit telemetry updates to reduce overhead.
        if (lastTelemetryNs == 0
                || now - lastTelemetryNs >= TELEMETRY_PERIOD_NS) {

            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("Actual RPM", "%.1f", actualRPM);

            telemetry.addData(
                    "Profiled RPM",
                    "%.1f",
                    speedController.profiledTarget
                            * 60.0 / TICKS_PER_REV
            );

            telemetry.addData(
                    "Shooter power",
                    "%.3f",
                    shooterPower
            );

            telemetry.addData(
                    "Loop ms",
                    "%.1f",
                    loopSeconds * 1000.0
            );

            telemetry.addData(
                    "Blocker open",
                    blockerOpen ? "yes" : "no"
            );

            telemetry.addData("RPM CSV", rpmLogger.status);
            telemetry.addData(
                    "Dropped RPM samples",
                    rpmLogger.droppedSamples
            );

            telemetry.update();
            lastTelemetryNs = now;
        }
    }

    // =========================================
    // ORIGINAL FIELD-CENTRIC DRIVE
    // =========================================

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

        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        drive(newForward, newRight, rotate);
    }

    private void drive(
            double forward,
            double right,
            double rotate
    ) {
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double maxPower = Math.max(
                1.0,
                Math.max(
                        Math.max(
                                Math.abs(frontLeftPower),
                                Math.abs(frontRightPower)
                        ),
                        Math.max(
                                Math.abs(backLeftPower),
                                Math.abs(backRightPower)
                        )
                )
        );

        frontLeftDrive.setPower(frontLeftPower / maxPower);
        frontRightDrive.setPower(frontRightPower / maxPower);
        backLeftDrive.setPower(backLeftPower / maxPower);
        backRightDrive.setPower(backRightPower / maxPower);
    }

    @Override
    public void stop() {
        shooterMotor.setPower(0);
        speedController.reset();

        // Signals background logger to finish writing.
        // Does not wait for disk I/O.
        rpmLogger.stopAsync();

        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    // =========================================
    // EXACT ORIGINAL PIDF CONTROLLER
    // =========================================

    public static class SpeedController {

        private double integral;
        private double filteredVel;
        private double profiledTarget;

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

            double dt = (now - lastTimeNs) * 1e-9;
            lastTimeNs = now;

            if (dt <= 0) {
                return 0.0;
            }

            double previousVel = filteredVel;

            double alpha = clamp(VELOCITY_ALPHA, 0, 1);

            filteredVel =
                    alpha * filteredVel
                            + (1 - alpha) * measured;

            double previousTarget = profiledTarget;

            double maxDelta = Math.max(0, MAX_ACCEL) * dt;

            profiledTarget += clamp(
                    target - profiledTarget,
                    -maxDelta,
                    maxDelta
            );

            double accel =
                    (profiledTarget - previousTarget) / dt;

            double error = profiledTarget - filteredVel;

            double derivative =
                    -(filteredVel - previousVel) / dt;

            if (Math.abs(kP * error) < 1.0) {
                double limit = Math.abs(MAX_INTEGRAL);

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

            return clamp(
                    (
                            kF * profiledTarget
                                    + staticTerm
                                    + kA * accel
                                    + kP * error
                                    + kI * integral
                                    + kD * derivative
                    ) * voltageScale,
                    -1,
                    1
            );
        }

        void reset() {
            integral = 0;
            filteredVel = 0;
            profiledTarget = 0;
            lastTimeNs = -1;
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

    // =========================================
    // ASYNC CSV RPM LOGGER
    // =========================================

    /**
     * Single-producer/single-consumer primitive ring buffer.
     *
     * The main OpMode loop never formats CSV rows,
     * waits for file I/O, or takes a lock.
     *
     * The writer runs on a daemon thread and
     * drains the buffer after STOP is pressed.
     */
    private static final class AsyncRpmLogger {

        private static final int CAPACITY = 2048;
        private static final int MASK = CAPACITY - 1;

        private static final String TAG = "ShooterRpmLogger";

        private final double[] timeSeconds =
                new double[CAPACITY];

        private final double[] measuredRpm =
                new double[CAPACITY];

        private final double[] requestedRpm =
                new double[CAPACITY];

        private final byte[] feeding =
                new byte[CAPACITY];

        private volatile long publishedIndex = 0;
        private volatile long consumedIndex = 0;

        private volatile boolean accepting = false;
        private volatile boolean stopping = false;

        volatile long droppedSamples = 0;
        volatile String status = "NOT STARTED";
        volatile String filename = "";

        private Thread writerThread;

        void start(final File rootDirectory) {

            if (writerThread != null) {
                return;
            }

            stopping = false;
            accepting = true;
            status = "STARTING";

            writerThread = new Thread(
                    () -> save(rootDirectory),
                    "ShooterRpmCsvWriter"
            );

            writerThread.setDaemon(true);
            writerThread.start();
        }

        // Called by OpMode thread about once every 20 ms.
        void record(
                double t,
                double rpm,
                double target,
                boolean isFeeding
        ) {
            if (!accepting || stopping) {
                return;
            }

            long write = publishedIndex;

            if (write - consumedIndex >= CAPACITY) {
                droppedSamples++;
                return;
            }

            int slot = (int) (write & MASK);

            timeSeconds[slot] = t;
            measuredRpm[slot] = rpm;
            requestedRpm[slot] = target;

            feeding[slot] =
                    (byte) (isFeeding ? 1 : 0);

            publishedIndex = write + 1;
        }

        private void save(File rootDirectory) {
            try {
                if (rootDirectory == null) {
                    throw new IOException(
                            "No external files directory"
                    );
                }

                File folder = new File(
                        rootDirectory,
                        "shooter_diagnostics"
                );

                if (!folder.isDirectory()
                        && !folder.mkdirs()
                        && !folder.isDirectory()) {

                    throw new IOException(
                            "Cannot create " + folder
                    );
                }

                File pending = File.createTempFile(
                        "rapidfire_original_pidf_rpm_"
                                + System.currentTimeMillis()
                                + "_",
                        ".csv.part",
                        folder
                );

                filename = pending.getName();

                File completed = new File(
                        folder,
                        filename.substring(
                                0,
                                filename.length()
                                        - ".part".length()
                        )
                );

                try (BufferedWriter writer =
                             new BufferedWriter(
                                     new FileWriter(pending),
                                     8192
                             )) {

                    writer.write(
                            "time_s,actual_rpm,target_rpm,feeding\n"
                    );

                    status = "RECORDING";

                    long read = 0;
                    long flushNs = System.nanoTime();

                    while (!stopping || read < publishedIndex) {

                        long available = publishedIndex;

                        if (read < available) {
                            int slot = (int) (read & MASK);

                            double t = timeSeconds[slot];
                            double actual = measuredRpm[slot];
                            double target = requestedRpm[slot];
                            int feed = feeding[slot];

                            consumedIndex = ++read;

                            writer.write(Double.toString(t));
                            writer.write(',');

                            writer.write(Double.toString(actual));
                            writer.write(',');

                            writer.write(Double.toString(target));
                            writer.write(',');

                            writer.write(Integer.toString(feed));
                            writer.write('\n');

                        } else {
                            try {
                                Thread.sleep(10);
                            } catch (InterruptedException e) {
                                // Continue draining samples.
                            }
                        }

                        if (System.nanoTime() - flushNs
                                >= 500_000_000L) {

                            writer.flush();
                            flushNs = System.nanoTime();
                        }
                    }

                    status = "FINALIZING";
                    writer.flush();
                }

                // Only mark complete after the file is closed.
                if (!pending.renameTo(completed)) {
                    throw new IOException(
                            "CSV closed but rename failed: " + pending
                    );
                }

                filename = completed.getName();
                status = "SAVED";

                Log.i(
                        TAG,
                        "Saved " + completed
                                + "; dropped=" + droppedSamples
                );

            } catch (IOException | RuntimeException e) {

                status = "ERROR";

                Log.e(
                        TAG,
                        "RPM CSV write failed",
                        e
                );

            } finally {
                accepting = false;
            }
        }

        void stopAsync() {
            accepting = false;
            stopping = true;

            // Return immediately.
            // Background writer closes and renames the file.
        }
    }
}
