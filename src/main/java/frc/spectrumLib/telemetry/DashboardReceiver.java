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
 * roboRIO it was a full-time job for one of the two cores. On SystemCore the publisher runs on
 * AdvantageKit's receiver thread, not the robot loop, but the controller shares its CPU with the
 * Limelight vision servers, so the same rule applies:
 *
 * <ul>
 *   <li>Only dashboard keys ({@link Telemetry#isDashboardKey}: every {@code logDash} key and every
 *       key the Elastic layout reads), plus metadata. Inputs never go.
 *   <li>Everything, while the {@value Telemetry#NT_MIRROR_SWITCH_KEY} switch is on (never with the
 *       FMS attached) -- for a live AdvantageScope session on the bench.
 *   <li>One table every {@code n} cycles. Skipped cycles are merged, not dropped: an output is only
 *       in the table of the cycle that recorded it, so a value logged once a second would otherwise
 *       never reach the dashboard whenever its cycle was a skipped one.
 * </ul>
 *
 * <p>The WPILOG file gets every key, every cycle, regardless.
 */
public class DashboardReceiver implements LogDataReceiver {
    private static final String OUTPUTS = "RealOutputs/";
    private static final String METADATA = "RealMetadata/";

    private final LogDataReceiver inner;
    private final int everyN;
    private final Map<String, LogValue> pending = new HashMap<>();
    private long cycle = 0;

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
    public void putTable(LogTable table) throws InterruptedException {
        boolean all = Telemetry.isMirroringAll();
        for (Map.Entry<String, LogValue> e : table.getAll(true).entrySet()) {
            String key = e.getKey();
            if (all
                    || key.startsWith(METADATA)
                    || (key.startsWith(OUTPUTS)
                            && Telemetry.isDashboardKey(key.substring(OUTPUTS.length())))) {
                pending.put(key, e.getValue());
            }
        }
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
}
