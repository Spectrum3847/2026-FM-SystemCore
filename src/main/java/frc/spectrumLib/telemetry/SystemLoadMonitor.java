package frc.spectrumLib.telemetry;

import java.io.BufferedReader;
import java.io.IOException;
import java.lang.management.GarbageCollectorMXBean;
import java.lang.management.ManagementFactory;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.List;
import org.wpilib.driverstation.Alert.Level;
import org.wpilib.driverstation.RobotState;
import org.wpilib.system.Timer;

/**
 * Watches the things that made the robot fall behind on 2026-09-05 and says so on the dashboard.
 *
 * <p>That day the roboRIO CPU sat at 92 to 95 percent and the loop ran at 30 ms, and nothing on the
 * Driver Station said so: the overrun watchdog is set to 200 ms and the CPU figure is buried in the
 * DS log viewer. This publishes the numbers once a second and raises an {@link Alert} when one of
 * them has been bad for long enough to matter:
 *
 * <ul>
 *   <li><b>CPU</b> from {@code /proc/stat}, alert after {@value #CPU_HOLD_SECONDS} s at or above
 *       {@value #CPU_ALERT_PERCENT} percent.
 *   <li><b>Loop overruns</b>: the share of loops longer than {@value #LOOP_OVERRUN_MS} ms, alert
 *       after {@value #LOOP_OVERRUN_HOLD_SECONDS} s above {@value #LOOP_OVERRUN_ALERT_PERCENT}
 *       percent.
 *   <li><b>Loop stall</b>: a single loop over {@value #LOOP_STALL_MS} ms while enabled, latched for
 *       {@value #LATCH_SECONDS} s so it is seen. Stalls while disabled are PathPlanner warmup and
 *       are ignored.
 *   <li><b>GC</b>: collector time from the JVM, alert when one second carried {@value
 *       #GC_ALERT_MS_PER_SECOND} ms or more of it while enabled.
 *   <li><b>Memory</b>: {@code MemAvailable} from {@code /proc/meminfo}, alert after {@value
 *       #MEMORY_HOLD_SECONDS} s under {@value #MEMORY_ALERT_MB} MB.
 * </ul>
 *
 * <p>Call {@link #periodic()} once per loop, first thing in {@code robotPeriodic()}. Per loop it
 * does a subtraction; once a second it reads two small files under {@code /proc}. Off the rio (no
 * {@code /proc}) the CPU and memory channels go quiet and the rest keeps working.
 */
public class SystemLoadMonitor {

    /** Seconds between samples and dashboard publishes. */
    public static final double SAMPLE_PERIOD_SECONDS = 1.0;

    /** A loop longer than this counts as an overrun. The budget is 20 ms. */
    public static final double LOOP_OVERRUN_FACTOR = 1.25;

    /** A loop longer than this counts as an overrun: 25 ms at 50 Hz, 12.5 ms at 100 Hz. */
    private static double loopOverrunMs() {
        return frc.spectrumLib.framework.RobotLoop.periodSeconds() * 1000.0 * LOOP_OVERRUN_FACTOR;
    }

    /** Share of overrunning loops that starts the loop alert clock. */
    public static final double LOOP_OVERRUN_ALERT_PERCENT = 50.0;

    /** Share of overrunning loops below which the loop alert clears. */
    public static final double LOOP_OVERRUN_CLEAR_PERCENT = 25.0;

    /** Seconds the overrun share must stay high before the loop alert shows. */
    public static final double LOOP_OVERRUN_HOLD_SECONDS = 5.0;

    /** A single enabled loop longer than this is a stall worth an error. */
    public static final double LOOP_STALL_MS = 200.0;

    /**
     * CPU percent that starts the CPU alert clock. 2026-09-05 ran at 92 to 95; a loop with a real
     * margin needs the two cores under about 80.
     */
    public static final double CPU_ALERT_PERCENT = 85.0;

    /** CPU percent below which the CPU alert clears. */
    public static final double CPU_CLEAR_PERCENT = 80.0;

    /** Seconds the CPU must stay high before the alert shows. */
    public static final double CPU_HOLD_SECONDS = 10.0;

    /** Collector milliseconds inside one second, while enabled, that raise the GC alert. */
    public static final double GC_ALERT_MS_PER_SECOND = 100.0;

    /** Available memory below which the memory alert clock starts. */
    public static final double MEMORY_ALERT_MB = 24.0;

    /** Seconds memory must stay low before the alert shows. */
    public static final double MEMORY_HOLD_SECONDS = 10.0;

    /** How long a one-shot alert (stall, GC) stays up so someone sees it. */
    public static final double LATCH_SECONDS = 10.0;

    /** Samples between refreshes of a sustained alert's text, to keep the Alerts log quiet. */
    private static final int TEXT_REFRESH_SAMPLES = 5;

    private static final Path PROC_STAT = Paths.get("/proc/stat");
    private static final Path PROC_MEMINFO = Paths.get("/proc/meminfo");

    private final Alert cpuAlert = new Alert("Controller CPU high", Level.MEDIUM);
    private final Alert loopAlert = new Alert("Robot loop overrunning", Level.MEDIUM);
    private final Alert stallAlert = new Alert("Robot loop stalled while enabled", Level.HIGH);
    private final Alert gcAlert = new Alert("GC pause while enabled", Level.MEDIUM);
    private final Alert memoryAlert = new Alert("Controller memory low", Level.MEDIUM);

    private final List<GarbageCollectorMXBean> gcBeans =
            ManagementFactory.getGarbageCollectorMXBeans();

    // ── Per-loop bookkeeping ───────────────────────────────────────────────────
    private double lastLoopSeconds = Double.NaN;
    private double bucketStartSeconds = Double.NaN;
    private int bucketLoops = 0;
    private int bucketOverruns = 0;
    private double bucketSumMs = 0;
    private double bucketMaxMs = 0;
    private long sampleCount = 0;

    // ── Alert state ───────────────────────────────────────────────────────────
    private double loopHighSinceSeconds = Double.NaN;
    private double cpuHighSinceSeconds = Double.NaN;
    private double memoryLowSinceSeconds = Double.NaN;
    private double stallLatchUntilSeconds = Double.NEGATIVE_INFINITY;
    private double gcLatchUntilSeconds = Double.NEGATIVE_INFINITY;

    // ── /proc and JVM counters ────────────────────────────────────────────────
    private boolean procAvailable = true;
    private long lastCpuTotalJiffies = -1;
    private long lastCpuIdleJiffies = -1;
    private long lastGcTimeMs = -1;
    private long lastGcCount = -1;

    /**
     * Records this loop and, once a second, samples the system and updates the alerts.
     *
     * <p>Call once per loop, before anything else in {@code robotPeriodic()}, so the loop period it
     * measures is the whole period.
     */
    public void periodic() {
        double now = Timer.getTimestamp();
        recordLoop(now);
        if (++censusLoops == 1000) {
            threadCensus("running"); // threads started lazily after robotInit
        }

        if (Double.isNaN(bucketStartSeconds)) {
            bucketStartSeconds = now;
        } else if (now - bucketStartSeconds >= SAMPLE_PERIOD_SECONDS) {
            sample(now);
            bucketStartSeconds = now;
        }
    }

    private void recordLoop(double now) {
        if (Double.isNaN(lastLoopSeconds)) {
            lastLoopSeconds = now;
            return;
        }
        double periodMs = (now - lastLoopSeconds) * 1000.0;
        lastLoopSeconds = now;

        bucketLoops++;
        bucketSumMs += periodMs;
        bucketMaxMs = Math.max(bucketMaxMs, periodMs);
        if (periodMs > loopOverrunMs()) {
            bucketOverruns++;
        }

        if (periodMs > LOOP_STALL_MS && RobotState.isEnabled()) {
            stallAlert.setText(String.format("Robot loop stalled %.0f ms while enabled", periodMs));
            stallAlert.set(true);
            stallLatchUntilSeconds = now + LATCH_SECONDS;
        }
    }

    private final java.lang.management.ThreadMXBean threadBean =
            ManagementFactory.getThreadMXBean();
    private final long mainThreadId = Thread.currentThread().threadId();
    private long lastProcessCpuNanos = -1;
    private long lastMainCpuNanos = -1;
    private long lastShareWallNanos = -1;

    /**
     * Logs how much CPU this program and its main (robot loop) thread used over the last sample.
     *
     * <p>Separates "the loop is slow because our code is slow" from "the loop is late because
     * something else on the controller has the CPU": on the SystemCore bench unit the loop ran 0.6
     * ms of work per 10 ms cycle yet overran, with the Limelight vision servers for two cameras
     * taking most of the CPU. {@code MainThreadPercent} is of one core; {@code ProcessPercent} is
     * of the whole machine.
     */
    private void logProcessShare(double now) {
        long wall = System.nanoTime();
        long processCpu =
                ProcessHandle.current()
                        .info()
                        .totalCpuDuration()
                        .map(java.time.Duration::toNanos)
                        .orElse(-1L);
        long mainCpu =
                threadBean.isThreadCpuTimeSupported()
                        ? threadBean.getThreadCpuTime(mainThreadId)
                        : -1;
        if (lastShareWallNanos > 0) {
            double wallNanos = wall - lastShareWallNanos;
            int cores = Runtime.getRuntime().availableProcessors();
            if (processCpu >= 0 && lastProcessCpuNanos >= 0) {
                Telemetry.logDash(
                        "System/ProcessPercent",
                        100.0 * (processCpu - lastProcessCpuNanos) / (wallNanos * cores),
                        "%");
            }
            if (mainCpu >= 0 && lastMainCpuNanos >= 0) {
                Telemetry.logDash(
                        "System/MainThreadPercent",
                        100.0 * (mainCpu - lastMainCpuNanos) / wallNanos,
                        "%");
            }
            Telemetry.logDash("System/ThreadCount", (long) threadBean.getThreadCount());
            logTopThreads(wallNanos);
        }
        lastShareWallNanos = wall;
        lastProcessCpuNanos = processCpu;
        lastMainCpuNanos = mainCpu;
    }

    private int censusLoops = 0;
    private static final java.util.Set<String> censusSeen = new java.util.HashSet<>();

    /**
     * Prints the thread ids that appeared since the last call, labelled, to the console (the
     * journal on the robot). Native threads have no useful name in /proc; this ties the ids that
     * {@code System/TopThreads} reports back to the library that started them. Boot-time only.
     */
    public static void threadCensus(String label) {
        java.io.File[] tasks = new java.io.File("/proc/self/task").listFiles();
        if (tasks == null) {
            return;
        }
        StringBuilder fresh = new StringBuilder();
        for (java.io.File task : tasks) {
            if (censusSeen.add(task.getName())) {
                String name = "?";
                try {
                    name = Files.readString(task.toPath().resolve("comm")).trim();
                } catch (IOException e) {
                    // exited
                }
                fresh.append(' ').append(task.getName()).append('(').append(name).append(')');
            }
        }
        StringBuilder busy = new StringBuilder();
        for (java.io.File task : tasks) {
            try {
                String stat = Files.readString(task.toPath().resolve("stat"));
                String[] f = stat.substring(stat.lastIndexOf(')') + 2).split(" ");
                long ticks = Long.parseLong(f[11]) + Long.parseLong(f[12]);
                if (ticks >= 20) {
                    busy.append(' ').append(task.getName()).append('=').append(ticks);
                }
            } catch (IOException | RuntimeException e) {
                // exited
            }
        }
        System.out.println("[Threads] " + label + ":" + fresh + " | cpu ticks:" + busy);
    }

    private final java.util.Map<String, Long> lastTaskTicks = new java.util.HashMap<>();

    /** Linux clock ticks per second for /proc times (USER_HZ; 100 on every ARM Linux here). */
    private static final double TICKS_PER_SECOND = 100.0;

    /**
     * Logs the busiest threads of this program over the last sample as {@code "name 12.3%"}
     * (percent of one core), from {@code /proc/self/task}, so native threads -- Phoenix's CAN and
     * odometry threads, NetworkTables, the HAL -- are counted as well as Java ones. On the
     * SystemCore bench unit the program used ~1.4 cores while its main loop used 0.17; this is how
     * to see where the rest goes.
     */
    private void logTopThreads(double wallNanos) {
        java.io.File[] tasks = new java.io.File("/proc/self/task").listFiles();
        if (tasks == null) {
            return; // not Linux (the desktop sim)
        }
        java.util.Map<String, Long> now = new java.util.HashMap<>();
        java.util.List<String> names = new java.util.ArrayList<>();
        java.util.List<Double> pcts = new java.util.ArrayList<>();
        for (java.io.File task : tasks) {
            try {
                String stat = Files.readString(task.toPath().resolve("stat"));
                int open = stat.indexOf('(');
                int close = stat.lastIndexOf(')');
                String name = stat.substring(open + 1, close);
                String[] f = stat.substring(close + 2).split(" ");
                // After "(comm) ": state is f[0], utime f[11], stime f[12].
                long ticks = Long.parseLong(f[11]) + Long.parseLong(f[12]);
                // rt_priority and policy: fields 40 and 41 of stat, 37 and 38 after "(comm) ".
                int rtPriority = Integer.parseInt(f[37]);
                String rt = rtPriority > 0 ? " [rt " + rtPriority + "]" : "";
                String key = task.getName() + ":" + name;
                now.put(key, ticks);
                Long before = lastTaskTicks.get(key);
                if (before != null) {
                    // Native threads that never set a name inherit "java"; keep them apart.
                    names.add((name.equals("java") ? "java/" + task.getName() : name) + rt);
                    pcts.add(100.0 * (ticks - before) / TICKS_PER_SECOND / (wallNanos / 1e9));
                }
            } catch (IOException | RuntimeException e) {
                // thread exited between listing and reading
            }
        }
        lastTaskTicks.clear();
        lastTaskTicks.putAll(now);
        // Sum threads that share a name (thread pools), then rank.
        java.util.Map<String, Double> byName = new java.util.HashMap<>();
        for (int i = 0; i < names.size(); i++) {
            byName.merge(names.get(i), pcts.get(i), Double::sum);
        }
        String[] top =
                byName.entrySet().stream()
                        .sorted((a, b) -> Double.compare(b.getValue(), a.getValue()))
                        .limit(10)
                        .map(e -> String.format("%s %.1f%%", e.getKey(), e.getValue()))
                        .toArray(String[]::new);
        Telemetry.addDashboardKey("System/TopThreads");
        Telemetry.log("System/TopThreads", top);
    }

    private void sample(double now) {
        sampleCount++;
        boolean refreshText = sampleCount % TEXT_REFRESH_SAMPLES == 0;
        boolean enabled = RobotState.isEnabled();

        // ── Loop ──────────────────────────────────────────────────────────────
        double overrunPercent = bucketLoops > 0 ? 100.0 * bucketOverruns / bucketLoops : 0;
        double meanMs = bucketLoops > 0 ? bucketSumMs / bucketLoops : 0;
        Telemetry.logDashAlways("System/Loop/MeanPeriodMs", meanMs, "ms");
        Telemetry.logDashAlways("System/Loop/MaxPeriodMs", bucketMaxMs, "ms");
        Telemetry.logDashAlways("System/Loop/OverrunPercent", overrunPercent, "%");

        if (overrunPercent >= LOOP_OVERRUN_ALERT_PERCENT) {
            if (Double.isNaN(loopHighSinceSeconds)) {
                loopHighSinceSeconds = now;
            }
            double held = now - loopHighSinceSeconds;
            if (held >= LOOP_OVERRUN_HOLD_SECONDS && (!loopAlert.get() || refreshText)) {
                loopAlert.setText(
                        String.format(
                                "Robot loop overrunning: %.0f%% of loops over %.0f ms for %.0f s"
                                        + " (mean %.1f ms)",
                                overrunPercent, loopOverrunMs(), held, meanMs));
                loopAlert.set(true);
            }
        } else if (overrunPercent < LOOP_OVERRUN_CLEAR_PERCENT) {
            loopHighSinceSeconds = Double.NaN;
            loopAlert.set(false);
        }

        bucketLoops = 0;
        bucketOverruns = 0;
        bucketSumMs = 0;
        bucketMaxMs = 0;

        // ── CPU ───────────────────────────────────────────────────────────────
        double cpuPercent = readCpuPercent();
        if (!Double.isNaN(cpuPercent)) {
            Telemetry.logDashAlways("System/CpuPercent", cpuPercent, "%");
            if (cpuPercent >= CPU_ALERT_PERCENT) {
                if (Double.isNaN(cpuHighSinceSeconds)) {
                    cpuHighSinceSeconds = now;
                }
                double held = now - cpuHighSinceSeconds;
                if (held >= CPU_HOLD_SECONDS && (!cpuAlert.get() || refreshText)) {
                    cpuAlert.setText(
                            String.format(
                                    "Controller CPU at %.0f%% for %.0f s - loop budget at risk",
                                    cpuPercent, held));
                    cpuAlert.set(true);
                }
            } else if (cpuPercent < CPU_CLEAR_PERCENT) {
                cpuHighSinceSeconds = Double.NaN;
                cpuAlert.set(false);
            }
        }

        // ── This program's share ─────────────────────────────────────────────
        logProcessShare(now);

        // ── Memory ────────────────────────────────────────────────────────────
        double availableMb = readMemAvailableMb();
        if (!Double.isNaN(availableMb)) {
            Telemetry.logDashAlways("System/MemAvailableMB", availableMb, "MB");
            if (availableMb < MEMORY_ALERT_MB) {
                if (Double.isNaN(memoryLowSinceSeconds)) {
                    memoryLowSinceSeconds = now;
                }
                double held = now - memoryLowSinceSeconds;
                if (held >= MEMORY_HOLD_SECONDS && (!memoryAlert.get() || refreshText)) {
                    memoryAlert.setText(
                            String.format(
                                    "roboRIO memory low: %.0f MB available for %.0f s",
                                    availableMb, held));
                    memoryAlert.set(true);
                }
            } else {
                memoryLowSinceSeconds = Double.NaN;
                memoryAlert.set(false);
            }
        }

        // ── GC and heap ───────────────────────────────────────────────────────
        long gcTimeMs = 0;
        long gcCount = 0;
        for (GarbageCollectorMXBean bean : gcBeans) {
            long t = bean.getCollectionTime();
            long c = bean.getCollectionCount();
            if (t > 0) {
                gcTimeMs += t;
            }
            if (c > 0) {
                gcCount += c;
            }
        }
        if (lastGcTimeMs >= 0) {
            long gcMsThisSecond = gcTimeMs - lastGcTimeMs;
            long collectionsThisSecond = gcCount - lastGcCount;
            Telemetry.logDashAlways("System/Gc/MsPerSecond", (double) gcMsThisSecond, "ms");
            Telemetry.logDashAlways("System/Gc/CollectionsPerSecond", collectionsThisSecond);
            if (gcMsThisSecond >= GC_ALERT_MS_PER_SECOND && enabled) {
                gcAlert.setText(
                        String.format(
                                "GC took %d ms in one second while enabled (%d collections)",
                                gcMsThisSecond, collectionsThisSecond));
                gcAlert.set(true);
                gcLatchUntilSeconds = now + LATCH_SECONDS;
            }
        }
        lastGcTimeMs = gcTimeMs;
        lastGcCount = gcCount;

        Runtime runtime = Runtime.getRuntime();
        Telemetry.logDashAlways(
                "System/HeapUsedMB", (runtime.totalMemory() - runtime.freeMemory()) / 1e6, "MB");

        // ── Latched one-shots ─────────────────────────────────────────────────
        if (now > stallLatchUntilSeconds) {
            stallAlert.set(false);
        }
        if (now > gcLatchUntilSeconds) {
            gcAlert.set(false);
        }
    }

    /**
     * Whole-machine CPU busy percent since the previous call, from the first line of {@code
     * /proc/stat}, or NaN on the first call or where {@code /proc} does not exist.
     */
    private double readCpuPercent() {
        if (!procAvailable) {
            return Double.NaN;
        }
        try (BufferedReader reader =
                Files.newBufferedReader(PROC_STAT, StandardCharsets.US_ASCII)) {
            String line = reader.readLine();
            if (line == null || !line.startsWith("cpu ")) {
                return Double.NaN;
            }
            // cpu user nice system idle iowait irq softirq steal ...
            String[] fields = line.trim().split("\\s+");
            long total = 0;
            long idle = 0;
            for (int i = 1; i < fields.length && i <= 8; i++) {
                long value = Long.parseLong(fields[i]);
                total += value;
                if (i == 4 || i == 5) {
                    idle += value;
                }
            }
            if (lastCpuTotalJiffies < 0) {
                lastCpuTotalJiffies = total;
                lastCpuIdleJiffies = idle;
                return Double.NaN;
            }
            long deltaTotal = total - lastCpuTotalJiffies;
            long deltaIdle = idle - lastCpuIdleJiffies;
            lastCpuTotalJiffies = total;
            lastCpuIdleJiffies = idle;
            return deltaTotal > 0 ? 100.0 * (deltaTotal - deltaIdle) / deltaTotal : Double.NaN;
        } catch (IOException | RuntimeException e) {
            // Not Linux, or an unexpected format: stop trying rather than fail every second.
            procAvailable = false;
            return Double.NaN;
        }
    }

    /**
     * {@code MemAvailable} from {@code /proc/meminfo} in megabytes, or NaN where it does not exist.
     * MemAvailable counts reclaimable page cache, unlike the MemFree the Driver Station shows,
     * which is why the DS read 4 to 5 MB free all day on 2026-09-05 with nothing actually wrong.
     */
    private double readMemAvailableMb() {
        if (!procAvailable) {
            return Double.NaN;
        }
        try (BufferedReader reader =
                Files.newBufferedReader(PROC_MEMINFO, StandardCharsets.US_ASCII)) {
            String line;
            int linesRead = 0;
            while ((line = reader.readLine()) != null && linesRead++ < 10) {
                if (line.startsWith("MemAvailable:")) {
                    String[] fields = line.trim().split("\\s+");
                    return Long.parseLong(fields[1]) / 1024.0;
                }
            }
            return Double.NaN;
        } catch (IOException | RuntimeException e) {
            procAvailable = false;
            return Double.NaN;
        }
    }
}
