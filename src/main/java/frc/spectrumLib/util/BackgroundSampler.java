package frc.spectrumLib.util;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.function.Supplier;

/**
 * Runs slow reads -- files under {@code /proc}, CAN bus status -- on one low-priority daemon
 * thread, so the robot loop only ever picks up the latest result.
 *
 * <p>On SystemCore the main thread runs at real-time priority, and a read that blocks for a
 * millisecond there (CANBus.getStatus() can) or walks sixty {@code /proc} files is a late loop. The
 * results are telemetry and alerts, which are outputs, so where they are computed does not matter
 * to AdvantageKit replay.
 *
 * <p>Start samplers during robot init: threads inherit the scheduling policy of the thread that
 * creates them, and the main thread becomes real-time at the end of init.
 */
public final class BackgroundSampler {
    private BackgroundSampler() {}

    private static ScheduledExecutorService executor;

    private static synchronized ScheduledExecutorService executor() {
        if (executor == null) {
            executor =
                    Executors.newSingleThreadScheduledExecutor(
                            runnable -> {
                                Thread t = new Thread(runnable, "BackgroundSampler");
                                t.setDaemon(true);
                                t.setPriority(Thread.MIN_PRIORITY);
                                return t;
                            });
        }
        return executor;
    }

    /**
     * The latest value a background read produced.
     *
     * @param <T> the value type
     */
    public static final class Latest<T> {
        private volatile T value;
        private volatile long sequence = 0;

        /** The newest value, or null before the first read finished. */
        public T get() {
            return value;
        }

        /** Increments each time a new value lands; compare to see whether one is new. */
        public long sequence() {
            return sequence;
        }

        void set(T v) {
            value = v;
            sequence++;
        }
    }

    /**
     * Runs {@code read} every {@code periodSeconds} on the sampler thread.
     *
     * @param periodSeconds time between reads
     * @param read the read; exceptions are swallowed so one bad read cannot stop the thread
     * @return where each result lands
     */
    public static <T> Latest<T> every(double periodSeconds, Supplier<T> read) {
        Latest<T> latest = new Latest<>();
        long periodMs = Math.max(1, (long) (periodSeconds * 1000));
        executor()
                .scheduleWithFixedDelay(
                        () -> {
                            try {
                                latest.set(read.get());
                            } catch (RuntimeException e) {
                                // keep sampling; the value simply does not update this time
                            }
                        },
                        0,
                        periodMs,
                        TimeUnit.MILLISECONDS);
        return latest;
    }
}
