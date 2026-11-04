package frc.spectrumLib.telemetry;

import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.LogDataReceiver;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.LogTable.LogValue;

/**
 * Feeds AdvantageKit's NetworkTables publisher only what a dashboard needs, at a dashboard's rate.
 *
 * <p>The 2026 offseason code stopped mirroring the whole log to NetworkTables because on the
 * roboRIO that took one of the two cores full time. On SystemCore the publisher runs on
 * AdvantageKit's receiver thread, not the robot loop. The controller still shares its CPU with the
 * Limelight vision servers, so the same rule applies:
 *
 * <ul>
 *   <li>Only dashboard keys ({@link Telemetry#isDashboardKey}: every {@code logDash} key and every
 *       key the Elastic layout reads), plus metadata. Inputs never go.
 *   <li>Everything, while the {@value Telemetry#NT_MIRROR_SWITCH_KEY} switch is on. This is for a
 *       live AdvantageScope session on the bench, and the switch is ignored with the FMS attached.
 *   <li>One table every {@code n} cycles. Skipped cycles are merged, not dropped: a change is only
 *       in the changes of the cycle that made it, so a value that changes once a second would
 *       otherwise never reach the dashboard whenever its cycle was a skipped one.
 * </ul>
 *
 * <p>The WPILOG file gets every key, every cycle, regardless.
 */
public class DashboardReceiver implements LogDataReceiver {
    private static final String METADATA = "/RealMetadata/";

    private final LogDataReceiver inner;
    private final int everyN;
    private final Map<String, LogValue> pending = new HashMap<>();
    private long cycle = 0;
    private boolean wasMirroringAll = false;

    /**
     * @param inner the NetworkTables publisher
     * @param everyN hand on one merged table per this many cycles
     */
    public DashboardReceiver(LogDataReceiver inner, int everyN) {
        this.inner = inner;
        this.everyN = Math.max(1, everyN);
    }

    @Override
    public void start() {
        inner.start();
    }

    @Override
    public void end() {
        inner.end();
    }

    @Override
    public void putChanges(LogTable changes, LogTable current) throws InterruptedException {
        boolean all = Telemetry.isMirroringAll();
        // Turning the mirror on sends everything once; after that, only what changes.
        collect(all && !wasMirroringAll ? current : changes, all);
        wasMirroringAll = all;
        if (++cycle % everyN != 0) {
            return;
        }
        LogTable merged = new LogTable(changes.getTimestamp());
        for (Map.Entry<String, LogValue> e : pending.entrySet()) {
            merged.put(e.getKey(), e.getValue());
        }
        pending.clear();
        inner.putChanges(merged, merged);
    }

    @Override
    public void putTable(LogTable table) throws InterruptedException {
        collect(table, Telemetry.isMirroringAll());
        if (++cycle % everyN != 0) {
            return;
        }
        LogTable merged = new LogTable(table.getTimestamp());
        for (Map.Entry<String, LogValue> e : pending.entrySet()) {
            merged.put(e.getKey(), e.getValue());
        }
        pending.clear();
        inner.putTable(merged);
    }

    /** Adds the table's dashboard entries to the ones waiting for the next publish. */
    private void collect(LogTable table, boolean all) {
        // getAll(false) is the table's own map (absolute keys, "/RealOutputs/..."), not a copy.
        for (Map.Entry<String, LogValue> e : table.getAll(false).entrySet()) {
            String key = e.getKey();
            // The Orin's raw results are most of the log's bytes; never mirror them to NT.
            boolean mirror = all && !key.endsWith("/Results") && !key.contains("/Results/");
            if (mirror || key.startsWith(METADATA) || Telemetry.isDashboardTableKey(key)) {
                pending.put(key.substring(1), e.getValue()); // put() takes keys relative to "/"
            }
        }
    }
}
