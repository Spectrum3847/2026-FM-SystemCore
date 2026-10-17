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
 * Retries every CAN device config until it reaches the device. A motor left on its flash config may
 * have no soft limits, the wrong direction and a 1:1 sensor ratio.
 *
 * <p>Boot applies run on the main thread under {@link CanConfigBudget}. After boot, one
 * low-priority thread retries failed configs with backoff, applies runtime changes from {@link
 * #request}, and re-applies after a device reset, since signal rates are not kept in flash.
 *
 * <p>Thread safety: each entry's lock keeps two applies to one device apart, and after boot only
 * the retry thread applies. Callers pass a config snapshot. Alerts and logging stay on the main
 * thread in {@link #periodic()}.
 *
 * <p>Replay: the thread never starts. Logic sees config status only through {@code
 * MotorInputs.configApplied}.
 */
public final class CanConfigRetry {

    /** First retry delay after a failed apply. */
    public static final double BACKOFF_MIN_SECONDS = 0.5;

    /**
     * Longest delay between retries. Each failure prints a Phoenix error, so this keeps a dead bus
     * from flooding the console.
     */
    public static final double BACKOFF_MAX_SECONDS = 5.0;

    public static final double THREAD_PERIOD_SECONDS = 0.1;

    /** Label of a device's main configuration. */
    public static final String CONFIG = "Config";

    /** Label of a TalonFX's status-signal rates and bus optimisation. */
    public static final String SIGNALS = "Signals";

    public static final CanConfigRetry INSTANCE =
            new CanConfigRetry(
                    System::nanoTime,
                    new BootPolicy() {
                        // Not method references, so CanConfigBudget loads only on first use.
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

    /** How a boot apply is attempted, so tests can fake {@link CanConfigBudget}. */
    public interface BootPolicy {
        /** Re-read before every retry, so a budget spent part-way stops them. */
        int maxAttempts();

        StatusCode run(String name, DoubleFunction<StatusCode> call);
    }

    private final LongSupplier clock;
    private final BootPolicy bootPolicy;
    private final double retryTimeoutSeconds;
    private final List<Entry> entries = new CopyOnWriteArrayList<>();
    private final Semaphore wake = new Semaphore(0);
    private volatile Thread thread;

    /**
     * @param clock monotonic time in nanoseconds
     */
    CanConfigRetry(LongSupplier clock, BootPolicy bootPolicy, double retryTimeoutSeconds) {
        this.clock = clock;
        this.bootPolicy = bootPolicy;
        this.retryTimeoutSeconds = retryTimeoutSeconds;
    }

    // ── Registration and applying ──────────────────────────────────────────────

    /**
     * Applies a config during robot init while the boot budget allows, then leaves it to the retry
     * thread.
     *
     * @param device may be null
     * @param what {@link #CONFIG} or {@link #SIGNALS}
     * @param apply takes a timeout in seconds, and must use a config snapshot
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

    /** Same, with a lower cap on boot attempts. */
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

    /** Registers a config for the retry thread without trying it now. */
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
     * Hands a changed config to the retry thread. Never blocks.
     *
     * @param apply must use a config snapshot
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

    /** The entry for a device's config, or null. */
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
     * Call before the main thread goes real-time, since a thread inherits its creator's scheduling
     * policy. Not in replay.
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
                                } catch (Throwable e) {
                                    // Plain stdout: Telemetry is main-thread only.
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

    /** Package-private so tests can drive it without the thread. */
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

    /** Main thread only. */
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
                                            + "). Retrying.",
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

    static long backoffNanos(int failures) {
        double seconds = BACKOFF_MIN_SECONDS * Math.pow(2, Math.max(0, failures - 1));
        return (long) (Math.min(seconds, BACKOFF_MAX_SECONDS) * 1e9);
    }

    public static final class Entry {
        private final String name;
        private final int canId;
        private final String bus;
        private final String what;
        private final ParentDevice device;
        private final boolean raisesAlert;

        /** Retry thread only. */
        private final BooleanSupplier resetChecker;

        private boolean resetPrimed = false;

        private final ReentrantLock lock = new ReentrantLock();

        /** Bumped by every new config and every reset. */
        private final AtomicLong generation = new AtomicLong();

        /** Written under {@link #lock}. */
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
         * True once a config has applied and nothing has failed since. A queued change keeps it.
         */
        public boolean isApplied() {
            return everApplied && consecutiveFailures == 0;
        }

        /** A later failed change leaves the earlier config, soft limits included, on the device. */
        public boolean hasEverApplied() {
            return everApplied;
        }

        public boolean isUpToDate() {
            return appliedGeneration == generation.get();
        }

        public String getName() {
            return name;
        }

        public String getWhat() {
            return what;
        }

        public int getCanId() {
            return canId;
        }

        public String getBus() {
            return bus;
        }

        public int getAttempts() {
            return attempts;
        }

        public int getResets() {
            return resets;
        }

        public String getLastStatus() {
            return lastStatus;
        }

        boolean isDue(long nowNanos) {
            return !isUpToDate() && nowNanos >= nextAttemptNanos;
        }

        void setConfig(DoubleFunction<StatusCode> newApply, long nowNanos) {
            apply = newApply;
            generation.incrementAndGet();
            nextAttemptNanos = nowNanos;
        }

        /** Retry thread only. */
        void pollReset(long nowNanos) {
            if (resetChecker == null) {
                return;
            }
            boolean reset = resetChecker.getAsBoolean();
            if (!resetPrimed) {
                // The first report is the boot this program already configured.
                resetPrimed = true;
                return;
            }
            if (reset) {
                resets++;
                generation.incrementAndGet();
                nextAttemptNanos = nowNanos;
            }
        }

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
                    // A config handed in during this attempt stays due now.
                    if (generation.get() == gen) {
                        nextAttemptNanos = clock.getAsLong() + backoffNanos(consecutiveFailures);
                    }
                }
                return status;
            } finally {
                lock.unlock();
            }
        }
    }
}
