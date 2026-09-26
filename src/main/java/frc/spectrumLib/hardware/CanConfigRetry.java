package frc.spectrumLib.hardware;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.ParentDevice;
import frc.spectrumLib.telemetry.Alert;
import frc.spectrumLib.telemetry.Telemetry;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.concurrent.Semaphore;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleFunction;
import java.util.function.LongSupplier;
import org.wpilib.driverstation.Alert.Level;

/**
 * Makes sure every CAN device configuration actually reaches its device, and keeps trying until it
 * does.
 *
 * <p>Until 2026-09-25 each device config was applied once at boot, with a 50 ms timeout. A failed
 * apply incremented {@code CANConfig/FailedCalls} and nothing else: the motor then ran on whatever
 * was in its flash -- for a hood motor swapped in from the spares shelf that is the factory config,
 * which means no soft limits, the wrong direction and a 1:1 sensor ratio, on a mechanism driven by
 * Motion Magic toward a setpoint in mechanism rotations. Nothing on the dashboard said so.
 *
 * <p>Now every device configuration goes through here, as an {@link Entry} per device and per
 * config (a mechanism TalonFX has its {@code TalonFXConfiguration} and, separately, its
 * status-signal rates):
 *
 * <ol>
 *   <li><b>At boot</b> ({@link #applyAtBoot}), on the main thread, it is retried up to {@link
 *       CanConfigBudget#maxAttempts()} times, each with {@link
 *       CanConfigBudget#CALL_TIMEOUT_SECONDS}, while the boot budget holds. Failures are still
 *       charged to {@link CanConfigBudget}, so a dead bus still boots in seconds.
 *   <li><b>After boot</b>, every entry that did not apply stays pending and is retried from one
 *       low-priority daemon thread ({@link #start()}), backing off from {@link
 *       #BACKOFF_MIN_SECONDS} to {@link #BACKOFF_MAX_SECONDS}, until it applies. A bus that comes
 *       up late, or a device plugged in after boot, gets its config within seconds. None of this
 *       blocks the real-time main loop, and none of it is charged to the boot budget.
 *   <li><b>Configuration changes at runtime</b> ({@link #request}: brake/coast, current limits) no
 *       longer block the main thread either: the caller snapshots the new config and hands it to
 *       the same thread, which applies it within one {@link #THREAD_PERIOD_SECONDS}.
 *   <li><b>A device that resets</b> (brownout, loose power lead, firmware fault) is noticed on that
 *       thread from Phoenix's startup frame ({@link ParentDevice#getResetOccurredChecker()}) and
 *       has every one of its entries re-applied. The config itself is persisted in the device's
 *       flash, so for it this is belt and braces; status-signal rates are not persisted, and this
 *       is what puts them back.
 * </ol>
 *
 * <p>Each entry that matters to a mechanism raises its own HIGH {@link Alert} ("Hood config not
 * applied (CAN id 15 on systemcore:2) -- retrying") while it is not applied, cleared the moment it
 * is, and every entry logs its status under {@code CANConfig/Devices/<name>/<config>/...}. Position
 * mechanisms with soft limits hold their output neutral while their config is missing ({@code
 * Mechanism.holdNeutralUntilConfigured}).
 *
 * <h2>Thread safety</h2>
 *
 * <p>Phoenix 6 allows a config apply to a device from any thread, concurrently with control
 * requests and signal refreshes from the main thread. What this class avoids is two applies to one
 * device at once: each entry has a lock that every apply of it (boot or retry) holds, and after
 * boot the main thread never applies at all -- {@link #request} only hands over a new config.
 * Callers pass a snapshot ({@code config.clone()}), never the live object the main thread keeps
 * editing. Alerts and logging happen only on the main thread ({@link #periodic()}), from volatile
 * fields the retry thread writes.
 *
 * <h2>Replay</h2>
 *
 * <p>This is hardware IO. In replay the thread is never started and nothing retries; whether a
 * mechanism's config applied reaches its logic only through its logged inputs ({@code
 * MotorInputs.configApplied}), which replay restores from the log.
 */
public final class CanConfigRetry {

    /** First retry delay after a failed apply. */
    public static final double BACKOFF_MIN_SECONDS = 0.5;

    /**
     * Longest delay between retries of one entry. Every failed apply prints a Phoenix error to the
     * Driver Station console, so a dead bus of twenty devices at this rate is four lines a second:
     * visible without being a flood, and a device that comes back is configured within five
     * seconds.
     */
    public static final double BACKOFF_MAX_SECONDS = 5.0;

    /** How often the retry thread wakes to look for due entries and device resets. */
    public static final double THREAD_PERIOD_SECONDS = 0.1;

    /** Label of a device's main configuration ({@code TalonFXConfiguration} and the like). */
    public static final String CONFIG = "Config";

    /** Label of a TalonFX's status-signal rates and bus optimisation. */
    public static final String SIGNALS = "Signals";

    /** The robot's tracker. */
    public static final CanConfigRetry INSTANCE =
            new CanConfigRetry(
                    System::nanoTime,
                    new BootPolicy() {
                        // Lambdas rather than method references so CanConfigBudget (which builds
                        // an Alert) is not initialised until the first boot apply.
                        @Override
                        public int maxAttempts() {
                            return CanConfigBudget.maxAttempts();
                        }

                        @Override
                        public StatusCode run(String name, DoubleFunction<StatusCode> call) {
                            return CanConfigBudget.run(name, call);
                        }
                    },
                    CanConfigBudget.CALL_TIMEOUT_SECONDS);

    /** How a boot apply is attempted: {@link CanConfigBudget} on the robot, a fake in tests. */
    public interface BootPolicy {
        /**
         * Attempts allowed for one config; re-read before every retry, so a budget spent part-way
         * through a device's retries stops them.
         *
         * @return the attempt count
         */
        int maxAttempts();

        /**
         * Runs one attempt.
         *
         * @param name the device, for alerts
         * @param call the apply, taking a timeout in seconds
         * @return its status
         */
        StatusCode run(String name, DoubleFunction<StatusCode> call);
    }

    private final LongSupplier clock;
    private final BootPolicy bootPolicy;
    private final double retryTimeoutSeconds;
    private final List<Entry> entries = new CopyOnWriteArrayList<>();
    private final Semaphore wake = new Semaphore(0);
    private volatile Thread thread;

    /**
     * Creates a tracker. The robot uses {@link #INSTANCE}; tests make their own.
     *
     * @param clock monotonic time in nanoseconds
     * @param bootPolicy how boot attempts are counted and charged
     * @param retryTimeoutSeconds timeout of each background attempt
     */
    CanConfigRetry(LongSupplier clock, BootPolicy bootPolicy, double retryTimeoutSeconds) {
        this.clock = clock;
        this.bootPolicy = bootPolicy;
        this.retryTimeoutSeconds = retryTimeoutSeconds;
    }

    // ── Registration and applying ──────────────────────────────────────────────

    /**
     * Applies a config during robot init, retrying while the boot budget allows, and leaves it for
     * the retry thread if it still has not applied.
     *
     * @param name the mechanism or device name, for alerts and log keys
     * @param device the device, for reset detection and to find the entry again; may be null
     * @param canId the device's CAN id
     * @param bus the bus name
     * @param what which config ({@link #CONFIG}, {@link #SIGNALS})
     * @param raisesAlert whether a missing config raises a HIGH alert
     * @param apply the apply, taking a timeout in seconds; must use a snapshot of the config
     * @return the entry
     */
    public Entry applyAtBoot(
            String name,
            ParentDevice device,
            int canId,
            String bus,
            String what,
            boolean raisesAlert,
            DoubleFunction<StatusCode> apply) {
        return applyAtBoot(name, device, canId, bus, what, raisesAlert, Integer.MAX_VALUE, apply);
    }

    /**
     * {@link #applyAtBoot(String, ParentDevice, int, String, String, boolean, DoubleFunction)} with
     * a cap on the attempts below the budget's own.
     *
     * @param name the mechanism or device name
     * @param device the device; may be null
     * @param canId the device's CAN id
     * @param bus the bus name
     * @param what which config
     * @param raisesAlert whether a missing config raises a HIGH alert
     * @param maxAttempts the most attempts to make now
     * @param apply the apply, taking a timeout in seconds
     * @return the entry
     */
    public Entry applyAtBoot(
            String name,
            ParentDevice device,
            int canId,
            String bus,
            String what,
            boolean raisesAlert,
            int maxAttempts,
            DoubleFunction<StatusCode> apply) {
        Entry entry = track(name, device, canId, bus, what, raisesAlert);
        entry.setConfig(apply, clock.getAsLong());
        for (int i = 0; i < Math.min(maxAttempts, bootPolicy.maxAttempts()); i++) {
            if (entry.attempt(fn -> bootPolicy.run(name, fn), clock).isOK() && entry.isUpToDate()) {
                break;
            }
        }
        return entry;
    }

    /**
     * Registers a config without attempting it, for the retry thread to apply. Used for calls that
     * are skipped at boot once the budget is spent.
     *
     * @param name the mechanism or device name
     * @param device the device; may be null
     * @param canId the device's CAN id
     * @param bus the bus name
     * @param what which config
     * @param raisesAlert whether a missing config raises a HIGH alert
     * @param apply the apply, taking a timeout in seconds
     * @return the entry
     */
    public Entry defer(
            String name,
            ParentDevice device,
            int canId,
            String bus,
            String what,
            boolean raisesAlert,
            DoubleFunction<StatusCode> apply) {
        Entry entry = track(name, device, canId, bus, what, raisesAlert);
        entry.setConfig(apply, clock.getAsLong());
        wake.release();
        return entry;
    }

    /**
     * Hands a changed config to the retry thread, which applies it within one {@link
     * #THREAD_PERIOD_SECONDS} and keeps retrying if it fails. Never blocks. For a device with no
     * entry yet, one is created (with an alert).
     *
     * @param name the mechanism or device name
     * @param device the device
     * @param canId the device's CAN id
     * @param bus the bus name
     * @param what which config
     * @param apply the apply, taking a timeout in seconds; must use a snapshot of the config
     * @return the entry
     */
    public Entry request(
            String name,
            ParentDevice device,
            int canId,
            String bus,
            String what,
            DoubleFunction<StatusCode> apply) {
        Entry entry = track(name, device, canId, bus, what, true);
        entry.setConfig(apply, clock.getAsLong());
        wake.release();
        return entry;
    }

    /**
     * The entry for a device's config, or null if it has none.
     *
     * @param device the device
     * @param what which config
     * @return the entry, or null
     */
    public Entry find(ParentDevice device, String what) {
        if (device == null) {
            return null;
        }
        for (Entry e : entries) {
            if (e.device == device && e.what.equals(what)) {
                return e;
            }
        }
        return null;
    }

    /**
     * @return every entry, in registration order
     */
    public List<Entry> entries() {
        return entries;
    }

    private synchronized Entry track(
            String name,
            ParentDevice device,
            int canId,
            String bus,
            String what,
            boolean raisesAlert) {
        Entry existing = find(device, what);
        if (existing != null) {
            return existing;
        }
        Entry entry =
                new Entry(
                        name,
                        canId,
                        bus,
                        what,
                        device,
                        device == null ? null : device.getResetOccurredChecker(),
                        raisesAlert);
        entries.add(entry);
        return entry;
    }

    // ── The retry thread ───────────────────────────────────────────────────────

    /**
     * Starts the retry thread. Call once, at the end of robot init and before the main thread is
     * made real-time (a thread inherits its creator's scheduling policy). Not in replay.
     */
    public synchronized void start() {
        if (thread != null) {
            return;
        }
        Thread t =
                new Thread(
                        () -> {
                            long periodMs = (long) (THREAD_PERIOD_SECONDS * 1000);
                            while (true) {
                                try {
                                    wake.tryAcquire(periodMs, TimeUnit.MILLISECONDS);
                                    wake.drainPermits();
                                    retryPass();
                                } catch (InterruptedException e) {
                                    return;
                                } catch (RuntimeException e) {
                                    // A throwing apply must not end retries for every device.
                                    // Plain stdout: Telemetry logs through AdvantageKit, which
                                    // is main-thread only.
                                    System.out.println("CanConfigRetry: " + e);
                                }
                            }
                        },
                        "CanConfigRetry");
        t.setDaemon(true);
        t.setPriority(Thread.MIN_PRIORITY);
        thread = t;
        t.start();
    }

    /**
     * One pass of the retry thread: notices device resets, then attempts every entry that is due.
     * Package-private so tests can drive it without the thread.
     */
    void retryPass() {
        for (Entry e : entries) {
            e.pollReset(clock.getAsLong());
        }
        for (Entry e : entries) {
            if (e.isDue(clock.getAsLong())) {
                e.attempt(fn -> fn.apply(retryTimeoutSeconds), clock);
            }
        }
    }

    // ── Main-thread reporting ──────────────────────────────────────────────────

    /** Main thread, once a second: raises or clears each entry's alert and logs its status. */
    public void periodic() {
        List<String> notApplied = new ArrayList<>();
        for (Entry e : entries) {
            boolean ok = e.isApplied();
            if (!ok) {
                notApplied.add(e.name + " " + e.what);
            }
            if (e.raisesAlert) {
                if (e.alert == null) {
                    String config = CONFIG.equals(e.what) ? "config" : e.what + " config";
                    e.alert =
                            new Alert(
                                    e.name
                                            + " "
                                            + config
                                            + " not applied (CAN id "
                                            + e.canId
                                            + " on "
                                            + e.bus
                                            + ") -- retrying",
                                    Level.HIGH);
                }
                e.alert.set(!ok);
            }
            Telemetry.log(e.keyPrefix + "Applied", ok);
            Telemetry.log(e.keyPrefix + "UpToDate", e.isUpToDate());
            Telemetry.log(e.keyPrefix + "Attempts", e.attempts);
            Telemetry.log(e.keyPrefix + "Resets", e.resets);
            Telemetry.log(e.keyPrefix + "LastStatus", e.lastStatus);
        }
        Telemetry.log("CANConfig/NotApplied", notApplied.toArray(new String[0]));
        Telemetry.log("CANConfig/NotAppliedCount", notApplied.size());
    }

    // ── Entry ──────────────────────────────────────────────────────────────────

    /** Retry delay after {@code failures} consecutive failures, in nanoseconds. */
    static long backoffNanos(int failures) {
        double seconds = BACKOFF_MIN_SECONDS * Math.pow(2, Math.max(0, failures - 1));
        return (long) (Math.min(seconds, BACKOFF_MAX_SECONDS) * 1e9);
    }

    /** One configuration of one device, and whether it has reached the device. */
    public static final class Entry {
        private final String name;
        private final int canId;
        private final String bus;
        private final String what;
        private final ParentDevice device;
        private final boolean raisesAlert;

        /** A clone of the device's reset signal; only ever called on the retry thread. */
        private final BooleanSupplier resetChecker;

        /** False until the retry thread has swallowed the checker's first report. */
        private boolean resetPrimed = false;

        private final ReentrantLock lock = new ReentrantLock();

        /** Bumped by every new config and every reset; what has applied is compared to it. */
        private final AtomicLong generation = new AtomicLong();

        /** The newest generation an attempt has applied. Written under {@link #lock}. */
        private volatile long appliedGeneration = 0;

        private volatile DoubleFunction<StatusCode> apply;
        private volatile boolean everApplied = false;
        private volatile int consecutiveFailures = 0;
        private volatile int attempts = 0;
        private volatile int resets = 0;
        private volatile long nextAttemptNanos = Long.MIN_VALUE;
        private volatile String lastStatus = "not attempted";

        // Main thread only.
        private Alert alert;
        private final String keyPrefix;

        Entry(
                String name,
                int canId,
                String bus,
                String what,
                ParentDevice device,
                BooleanSupplier resetChecker,
                boolean raisesAlert) {
            this.name = name;
            this.canId = canId;
            this.bus = bus;
            this.what = what;
            this.device = device;
            this.resetChecker = resetChecker;
            this.raisesAlert = raisesAlert;
            this.keyPrefix = "CANConfig/Devices/" + name + "/" + what + "/";
        }

        /**
         * Whether the device is running a config this code gave it: one has applied and nothing has
         * failed since. A newer config or a reset that is merely queued keeps this true; a failed
         * attempt, or a config that has never once applied, makes it false.
         *
         * @return true when the device can be trusted to have this config
         */
        public boolean isApplied() {
            return everApplied && consecutiveFailures == 0;
        }

        /**
         * Whether the newest config handed in has applied, with nothing queued.
         *
         * @return true when nothing is pending
         */
        public boolean isUpToDate() {
            return appliedGeneration == generation.get();
        }

        /**
         * @return the device or mechanism name
         */
        public String getName() {
            return name;
        }

        /**
         * @return which config this is ({@link #CONFIG}, {@link #SIGNALS}, ...)
         */
        public String getWhat() {
            return what;
        }

        /**
         * @return the device's CAN id
         */
        public int getCanId() {
            return canId;
        }

        /**
         * @return the bus name
         */
        public String getBus() {
            return bus;
        }

        /**
         * @return attempts made so far, boot included
         */
        public int getAttempts() {
            return attempts;
        }

        /**
         * @return device resets noticed so far
         */
        public int getResets() {
            return resets;
        }

        /**
         * @return the status of the last attempt
         */
        public String getLastStatus() {
            return lastStatus;
        }

        boolean isDue(long nowNanos) {
            return !isUpToDate() && nowNanos >= nextAttemptNanos;
        }

        /** Hands in a new config and makes it due now. */
        void setConfig(DoubleFunction<StatusCode> newApply, long nowNanos) {
            apply = newApply;
            generation.incrementAndGet();
            nextAttemptNanos = nowNanos;
        }

        /** Retry thread only: queues a re-apply if the device has booted since the last look. */
        void pollReset(long nowNanos) {
            if (resetChecker == null) {
                return;
            }
            boolean reset = resetChecker.getAsBoolean();
            if (!resetPrimed) {
                // The first look reports whatever startup frame arrived since construction,
                // including the boot this robot program already configured.
                resetPrimed = true;
                return;
            }
            if (reset) {
                resets++;
                generation.incrementAndGet();
                nextAttemptNanos = nowNanos;
            }
        }

        /**
         * Runs one attempt of the current config under the entry's lock and records the outcome.
         *
         * @param runner runs the apply (through the boot budget, or directly with a timeout)
         * @param clock for scheduling the next retry
         * @return the status
         */
        StatusCode attempt(
                java.util.function.Function<DoubleFunction<StatusCode>, StatusCode> runner,
                LongSupplier clock) {
            lock.lock();
            try {
                long gen = generation.get();
                DoubleFunction<StatusCode> fn = apply;
                StatusCode status = fn == null ? StatusCode.OK : runner.apply(fn);
                attempts++;
                lastStatus = status.getName();
                if (status.isOK()) {
                    everApplied = true;
                    consecutiveFailures = 0;
                    appliedGeneration = Math.max(appliedGeneration, gen);
                } else {
                    consecutiveFailures++;
                    nextAttemptNanos = clock.getAsLong() + backoffNanos(consecutiveFailures);
                }
                return status;
            } finally {
                lock.unlock();
            }
        }
    }
}
