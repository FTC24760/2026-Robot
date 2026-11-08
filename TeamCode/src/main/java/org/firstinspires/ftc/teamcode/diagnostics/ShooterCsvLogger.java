package org.firstinspires.ftc.teamcode.diagnostics;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicInteger;

public final class ShooterCsvLogger {
    public static class Sample {

        public double timeSeconds;

        public double rawRpm;
        public double filteredRpm;

        public double targetRpm;
        public double profiledRpm;
        public double errorRpm;

        public double motorPower;
        public double batteryVolts;
        public double loopMs;

        public double rightTrigger;
        public double leftTrigger;

        public int xPressed;
        public int shooterEnabled;
        public int feeding;
        public int burstId;
    }

    // ===================================
    // NONBLOCKING CSV LOGGER
    // ===================================




        private static final int QUEUE_CAPACITY = 4096;

        private final ArrayBlockingQueue<Sample> queue =
                new ArrayBlockingQueue<>(QUEUE_CAPACITY);

        public final AtomicInteger droppedSamples =
                new AtomicInteger(0);

        public volatile long writtenSamples = 0;

        public volatile String status = "NOT STARTED";
        public volatile String error = null;

        private volatile boolean stopping = false;
        private volatile boolean accepting = false;

        private Thread writerThread;

        public void start(File baseDirectory) {

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

                            Sample s;

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

        public void record(Sample sample) {

            if (!accepting || stopping) {
                return;
            }

            // No disk I/O on control loop.
            if (!queue.offer(sample)) {
                droppedSamples.incrementAndGet();
            }
        }

        public int queueSize() {
            return queue.size();
        }

        public void stopAsync() {

            accepting = false;
            stopping = true;

            // No join or flush on FTC thread.
            status = "STOP REQUESTED";
        }
    }

