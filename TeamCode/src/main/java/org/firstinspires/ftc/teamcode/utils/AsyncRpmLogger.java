package org.firstinspires.ftc.teamcode.utils;

import android.util.Log;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

/**
 * Writes shooter RPM samples to a CSV without slowing the robot loop.
 *
 * Single-producer/single-consumer primitive ring buffer: the OpMode loop never
 * formats CSV rows, waits for file I/O, or takes a lock. The writer runs on a
 * daemon thread and drains the buffer after stop is requested.
 *
 * Files land in [rootDirectory]/shooter_diagnostics/.
 */
public final class AsyncRpmLogger {

    private static final int CAPACITY = 2048; // must be a power of two
    private static final int MASK = CAPACITY - 1;
    private static final String TAG = "ShooterRpmLogger";
    private static final String FOLDER_NAME = "shooter_diagnostics";
    private static final String FILE_PREFIX = "rapidfire_original_pidf_rpm_";

    private final double[] timeSeconds = new double[CAPACITY];
    private final double[] measuredRpm = new double[CAPACITY];
    private final double[] requestedRpm = new double[CAPACITY];
    private final byte[] feeding = new byte[CAPACITY];

    private volatile long publishedIndex = 0;
    private volatile long consumedIndex = 0;

    private volatile boolean accepting = false;
    private volatile boolean stopping = false;

    private volatile long droppedSamples = 0;
    private volatile String status = "NOT STARTED";
    private volatile String filename = "";

    private Thread writerThread;

    public void start(final File rootDirectory) {
        if (writerThread != null) {
            return;
        }

        stopping = false;
        accepting = true;
        status = "STARTING";

        writerThread = new Thread(() -> save(rootDirectory), "ShooterRpmCsvWriter");
        writerThread.setDaemon(true);
        writerThread.start();
    }

    /** Called from the OpMode thread. Never blocks; drops the sample if the buffer is full. */
    public void record(double t, double rpm, double target, boolean isFeeding) {
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
        feeding[slot] = (byte) (isFeeding ? 1 : 0);

        publishedIndex = write + 1;
    }

    /** Returns immediately; the background writer finishes, closes, and renames the file. */
    public void stopAsync() {
        accepting = false;
        stopping = true;
    }

    public String getStatus() {
        return status;
    }

    public String getFilename() {
        return filename;
    }

    public long getDroppedSamples() {
        return droppedSamples;
    }

    private void save(File rootDirectory) {
        try {
            if (rootDirectory == null) {
                throw new IOException("No external files directory");
            }

            File folder = new File(rootDirectory, FOLDER_NAME);

            if (!folder.isDirectory() && !folder.mkdirs() && !folder.isDirectory()) {
                throw new IOException("Cannot create " + folder);
            }

            // Written as .csv.part and renamed once complete, so a .csv is never half-written.
            File pending = File.createTempFile(
                    FILE_PREFIX + System.currentTimeMillis() + "_",
                    ".csv.part",
                    folder
            );

            filename = pending.getName();

            File completed = new File(
                    folder,
                    filename.substring(0, filename.length() - ".part".length())
            );

            try (BufferedWriter writer = new BufferedWriter(new FileWriter(pending), 8192)) {
                writer.write("time_s,actual_rpm,target_rpm,feeding\n");
                status = "RECORDING";

                long read = 0;
                long flushNs = System.nanoTime();

                while (!stopping || read < publishedIndex) {
                    if (read < publishedIndex) {
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

                    if (System.nanoTime() - flushNs >= 500_000_000L) {
                        writer.flush();
                        flushNs = System.nanoTime();
                    }
                }

                status = "FINALIZING";
                writer.flush();
            }

            // Only mark complete after the file is closed.
            if (!pending.renameTo(completed)) {
                throw new IOException("CSV closed but rename failed: " + pending);
            }

            filename = completed.getName();
            status = "SAVED";
            Log.i(TAG, "Saved " + completed + "; dropped=" + droppedSamples);

        } catch (IOException | RuntimeException e) {
            status = "ERROR";
            Log.e(TAG, "RPM CSV write failed", e);
        } finally {
            accepting = false;
        }
    }
}
