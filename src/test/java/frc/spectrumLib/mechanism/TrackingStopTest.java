package frc.spectrumLib.mechanism;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class TrackingStopTest {
    private static final double LOOP = 0.01;

    private final TrackingStop stop = new TrackingStop(2.0, 0.5);
    private double now = 0;

    /** Runs {@code seconds} of loops; returns whether any of them latched. */
    private boolean run(
            double seconds, boolean enabled, boolean connected, boolean closedLoop, double error) {
        boolean latched = false;
        for (double end = now + seconds; now < end - 1e-9; now += LOOP) {
            latched |= stop.update(now, enabled, connected, closedLoop, error, true);
        }
        return latched;
    }

    @Test
    void insideToleranceNeverStops() {
        assertFalse(run(10, true, true, true, 1.9));
        assertFalse(stop.isStopped());
    }

    @Test
    void stopsOnceOutForTheDebounce() {
        assertFalse(run(0.49, true, true, true, -3));
        assertFalse(stop.isStopped());
        assertTrue(run(0.03, true, true, true, -3));
        assertTrue(stop.isStopped());
        assertFalse(run(1, true, true, true, -3), "reports the latch once");
        assertTrue(stop.isStopped());
    }

    @Test
    void aShortExcursionRestartsTheDebounce() {
        for (int i = 0; i < 20; i++) {
            run(0.4, true, true, true, 5);
            run(0.02, true, true, true, 0);
        }
        assertFalse(stop.isStopped());
        assertFalse(stop.isFault());
    }

    @Test
    void disconnectedReadingsNeverCount() {
        assertFalse(run(5, true, false, true, 100), "a dropped bus reads zero, not a jam");
        run(0.4, true, true, true, 100);
        run(0.02, true, false, true, 100);
        assertFalse(run(0.4, true, true, true, 100), "a disconnect restarts the debounce");
        assertFalse(stop.isStopped());
    }

    @Test
    void openLoopNeverCounts() {
        assertFalse(run(5, true, true, false, 100));
        assertFalse(stop.isFault());
    }

    @Test
    void disablingClearsIt() {
        run(1, true, true, true, 5);
        assertTrue(stop.isStopped());
        run(LOOP, false, true, true, 5);
        assertFalse(stop.isStopped());
        assertFalse(stop.isFault());
        assertFalse(run(0.4, true, true, true, 5), "the debounce starts over after enabling");
    }

    @Test
    void clearingReleasesItAndStillStuckStopsAgain() {
        run(1, true, true, true, 5);
        stop.clear();
        assertFalse(stop.isStopped());
        // Held in neutral, the motor sends no closed-loop setpoint until it is released.
        assertFalse(run(0.4, true, true, true, 5));
        assertTrue(run(0.2, true, true, true, 5));
    }

    @Test
    void reportOnlyLogsTheFaultWithoutStopping() {
        for (int i = 0; i < 100; i++) {
            assertFalse(stop.update(now, true, true, true, 5, false));
            now += LOOP;
        }
        assertTrue(stop.isFault());
        assertFalse(stop.isStopped());
    }
}
