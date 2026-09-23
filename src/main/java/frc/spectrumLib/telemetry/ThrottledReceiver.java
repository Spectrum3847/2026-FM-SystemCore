package frc.spectrumLib.telemetry;

import org.littletonrobotics.junction.LogDataReceiver;
import org.littletonrobotics.junction.LogTable;

/**
 * Passes every {@code n}th AdvantageKit cycle on to another receiver.
 *
 * <p>For the NetworkTables publisher: it pushes every changed key to every client each cycle, and
 * at 100 Hz with the whole localization testbed logged that is CPU and network the dashboards
 * cannot use (AdvantageScope's live view and Elastic draw at screen rate). The WPILOG file still
 * gets every cycle; only the live NT view is thinned. The table handed on is the latest cycle's, so
 * nothing is stale, only skipped.
 *
 * <p>Receivers run on AdvantageKit's receiver thread, not the robot loop.
 */
public class ThrottledReceiver implements LogDataReceiver {
    private final LogDataReceiver inner;
    private final int everyN;
    private long cycle = 0;

    /**
     * @param inner the receiver to throttle
     * @param everyN pass one cycle in this many
     */
    public ThrottledReceiver(LogDataReceiver inner, int everyN) {
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
        if (cycle++ % everyN == 0) {
            inner.putTable(table);
        }
    }
}
