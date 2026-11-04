package frc.spectrumLib.telemetry;

import java.io.IOException;
import java.lang.management.GarbageCollectorMXBean;
import java.lang.management.ManagementFactory;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.wpilib.util.AlertDataJNI;

/**
 * CPU and allocation cost of the robot loop, for the replay bench ({@code tools/pi_bench.sh}).
 *
 * <p>{@link #start()} at the end of robot init takes a snapshot; {@link #cycle()} counts loops; at
 * exit (the replay log ends and AdvantageKit exits) it prints, per thread, the CPU used since init
 * and per loop, plus allocation and GC. Native threads (ntcore, HAL, Phoenix) are included because
 * the per-thread numbers come from {@code /proc/self/task}, not the JVM. Off unless {@code
 * -Dfm.bench=true}; it never logs, so replay output is unchanged.
 */
public final class BenchReport {
    private BenchReport() {}

    public static final boolean ENABLED = Boolean.getBoolean("fm.bench");

    private static long cycles;
    private static long startNanos;
    private static long startAllocBytes;
    private static long startGcCount;
    private static long startGcMs;
    private static Map<String, Long> startThreadNs = Map.of();
    private static boolean started;

    /** Snapshot after init. Later loops are measured against it. */
    public static void start() {
        if (!ENABLED || started) {
            return;
        }
        started = true;
        printInitCost();
        startThreadNs = threadCpuNs();
        startAllocBytes = allocatedBytes();
        startGcCount = gcCount();
        startGcMs = gcMs();
        startNanos = System.nanoTime();
        Runtime.getRuntime().addShutdownHook(new Thread(BenchReport::print, "BenchReport"));
    }

    public static void cycle() {
        if (started && ++cycles % 500 == 0) {
            // AdvantageKit stops its log receiver thread before the exit hook runs, so threads
            // are read during the run too and the last reading of an exited thread is kept.
            lastSeenThreadNs.putAll(threadCpuNs());
            // AdvantageKit copies every registered alert out of native code each loop.
            alertsRegistered = Math.max(alertsRegistered, AlertDataJNI.getNumAlerts());
        }
    }

    private static int alertsRegistered;

    private static final Map<String, Long> lastSeenThreadNs = new HashMap<>();

    private static void printInitCost() {
        long total = 0;
        for (long ns : threadCpuNs().values()) {
            total += ns;
        }
        System.out.printf(
                "[Bench] init: %.1f s CPU across threads, %.1f s since JVM start%n",
                total / 1e9, ManagementFactory.getRuntimeMXBean().getUptime() / 1e3);
    }

    private static void print() {
        double wallS = (System.nanoTime() - startNanos) / 1e9;
        long n = Math.max(1, cycles);
        Map<String, Long> end = new HashMap<>(lastSeenThreadNs);
        end.putAll(threadCpuNs());
        List<Map.Entry<String, Long>> rows = new ArrayList<>();
        long total = 0;
        // Group by thread name: GC and compiler pools have many threads of one name.
        Map<String, Long> byName = new HashMap<>();
        for (var e : end.entrySet()) {
            long d = e.getValue() - startThreadNs.getOrDefault(e.getKey(), 0L);
            if (d <= 0) {
                continue;
            }
            total += d;
            String name = e.getKey().substring(e.getKey().indexOf(' ') + 1);
            name = name.replaceAll("#\\d+$", "#").replaceAll("\\d+$", "");
            byName.merge(name, d, Long::sum);
        }
        rows.addAll(byName.entrySet());
        rows.sort((a, b) -> Long.compare(b.getValue(), a.getValue()));
        long alloc = allocatedBytes() - startAllocBytes;
        StringBuilder sb = new StringBuilder();
        sb.append(
                String.format(
                        "[Bench] loops %d, wall %.2f s (%.3f ms/loop)%n",
                        cycles, wallS, wallS * 1e3 / n));
        sb.append(
                String.format(
                        "[Bench] process CPU %.3f ms/loop (%.1f s)%n",
                        total / 1e6 / n, total / 1e9));
        sb.append(
                String.format(
                        "[Bench] allocated %.1f KB/loop (%.0f MB), GC %d collections, %d ms%n",
                        alloc / 1024.0 / n,
                        alloc / 1e6,
                        gcCount() - startGcCount,
                        gcMs() - startGcMs));
        sb.append(
                String.format(
                        "[Bench] peak RSS %s, heap used %.0f MB%n",
                        status("VmHWM"),
                        (Runtime.getRuntime().totalMemory() - Runtime.getRuntime().freeMemory())
                                / 1e6));
        sb.append(String.format("[Bench] alerts registered: %d%n", alertsRegistered));
        for (var e : rows) {
            if (e.getValue() < total / 500) {
                break;
            }
            sb.append(
                    String.format(
                            "[Bench]   %8.3f ms/loop %5.1f%%  %s%n",
                            e.getValue() / 1e6 / n, 100.0 * e.getValue() / total, e.getKey()));
        }
        System.out.print(sb);
        System.out.flush();
    }

    /** Run time of every thread of this process, by "tid comm", in nanoseconds. */
    private static Map<String, Long> threadCpuNs() {
        Map<String, Long> out = new HashMap<>();
        try (var tasks = Files.list(Path.of("/proc/self/task"))) {
            for (Path t : (Iterable<Path>) tasks::iterator) {
                try {
                    String comm = Files.readString(t.resolve("comm")).trim();
                    String sched = Files.readString(t.resolve("schedstat")).trim();
                    long ns = Long.parseLong(sched.substring(0, sched.indexOf(' ')));
                    out.put(t.getFileName() + " " + comm, ns);
                } catch (IOException | RuntimeException e) {
                    // The thread exited between the listing and the read.
                }
            }
        } catch (IOException e) {
            // Not Linux.
        }
        return out;
    }

    private static long allocatedBytes() {
        var bean = ManagementFactory.getThreadMXBean();
        if (bean instanceof com.sun.management.ThreadMXBean sun) {
            return sun.getTotalThreadAllocatedBytes();
        }
        return 0;
    }

    private static long gcCount() {
        long c = 0;
        for (GarbageCollectorMXBean b : ManagementFactory.getGarbageCollectorMXBeans()) {
            c += Math.max(0, b.getCollectionCount());
        }
        return c;
    }

    private static long gcMs() {
        long c = 0;
        for (GarbageCollectorMXBean b : ManagementFactory.getGarbageCollectorMXBeans()) {
            c += Math.max(0, b.getCollectionTime());
        }
        return c;
    }

    private static String status(String key) {
        try {
            for (String line : Files.readAllLines(Path.of("/proc/self/status"))) {
                if (line.startsWith(key + ":")) {
                    return line.substring(key.length() + 1).trim();
                }
            }
        } catch (IOException e) {
            // Not Linux.
        }
        return "?";
    }
}
