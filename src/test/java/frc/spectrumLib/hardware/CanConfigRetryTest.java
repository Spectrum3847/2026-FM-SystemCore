package frc.spectrumLib.hardware;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.StatusCode;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.DoubleFunction;
import org.junit.jupiter.api.Test;

/** The retry bookkeeping of {@link CanConfigRetry}, with a fake clock and no hardware. */
class CanConfigRetryTest {
    private static final StatusCode FAIL = StatusCode.ConfigFailed;
    private static final long SECOND = 1_000_000_000L;

    /** A settable clock. */
    private long now = 0;

    /** A boot budget allowing {@code attempts} per config and charging nothing. */
    private static CanConfigRetry.BootPolicy budget(AtomicInteger attempts) {
        return new CanConfigRetry.BootPolicy() {
            @Override
            public int maxAttempts() {
                return attempts.get();
            }

            @Override
            public StatusCode run(String name, DoubleFunction<StatusCode> call) {
                return call.apply(0.1);
            }
        };
    }

    private CanConfigRetry tracker(int attempts) {
        return new CanConfigRetry(() -> now, budget(new AtomicInteger(attempts)), 0.1);
    }

    /** An apply that fails {@code failures} times, then succeeds; counts its calls. */
    private static DoubleFunction<StatusCode> failing(int failures, AtomicInteger calls) {
        return timeout -> calls.incrementAndGet() > failures ? StatusCode.OK : FAIL;
    }

    private static CanConfigRetry.Entry boot(CanConfigRetry t, DoubleFunction<StatusCode> apply) {
        return t.applyAtBoot("Hood", null, 15, "systemcore:2", CanConfigRetry.CONFIG, true, apply);
    }

    @Test
    void bootRetriesUntilTheConfigApplies() {
        AtomicInteger calls = new AtomicInteger();
        CanConfigRetry.Entry e = boot(tracker(10), failing(2, calls));
        assertEquals(3, calls.get());
        assertTrue(e.isApplied());
        assertTrue(e.isUpToDate());
    }

    @Test
    void bootStopsAtTheAttemptLimitAndLeavesItPending() {
        AtomicInteger calls = new AtomicInteger();
        CanConfigRetry.Entry e = boot(tracker(4), failing(100, calls));
        assertEquals(4, calls.get());
        assertFalse(e.isApplied());
        assertFalse(e.isUpToDate());
        assertEquals(FAIL.getName(), e.getLastStatus());
    }

    @Test
    void bootStopsRetryingWhenTheBudgetRunsOutPartWay() {
        AtomicInteger allowed = new AtomicInteger(10);
        CanConfigRetry t = new CanConfigRetry(() -> now, budget(allowed), 0.1);
        AtomicInteger calls = new AtomicInteger();
        boot(
                t,
                timeout -> {
                    if (calls.incrementAndGet() == 2) {
                        allowed.set(1); // the budget is spent during the second attempt
                    }
                    return FAIL;
                });
        assertEquals(2, calls.get());
    }

    @Test
    void failedConfigIsRetriedInTheBackgroundWithBackoffUntilItApplies() {
        CanConfigRetry t = tracker(1);
        AtomicInteger calls = new AtomicInteger();
        CanConfigRetry.Entry e = boot(t, failing(2, calls)); // boot: fail 1
        assertEquals(1, calls.get());

        t.retryPass(); // too soon
        assertEquals(1, calls.get());

        now += CanConfigRetry.backoffNanos(1);
        t.retryPass(); // fail 2
        assertEquals(2, calls.get());
        assertFalse(e.isApplied());

        now += CanConfigRetry.backoffNanos(1); // not yet: the second failure doubled the wait
        t.retryPass();
        assertEquals(2, calls.get());

        now += CanConfigRetry.backoffNanos(2);
        t.retryPass(); // succeeds
        assertEquals(3, calls.get());
        assertTrue(e.isApplied());
        assertTrue(e.isUpToDate());

        now += 100 * SECOND;
        t.retryPass(); // nothing left to do
        assertEquals(3, calls.get());
    }

    @Test
    void backoffDoublesFromHalfASecondAndCapsAtFive() {
        assertEquals(SECOND / 2, CanConfigRetry.backoffNanos(1));
        assertEquals(SECOND, CanConfigRetry.backoffNanos(2));
        assertEquals(4 * SECOND, CanConfigRetry.backoffNanos(4));
        assertEquals(5 * SECOND, CanConfigRetry.backoffNanos(5));
        assertEquals(5 * SECOND, CanConfigRetry.backoffNanos(50));
    }

    @Test
    void runtimeRequestIsAppliedByTheRetryThreadNotTheCaller() {
        CanConfigRetry t = tracker(10);
        CanConfigRetry.Entry e = boot(t, failing(0, new AtomicInteger()));
        AtomicInteger newCalls = new AtomicInteger();
        DoubleFunction<StatusCode> newConfig = failing(0, newCalls);

        e.setConfig(newConfig, now);
        assertEquals(0, newCalls.get(), "request must not apply on the caller's thread");
        assertTrue(e.isApplied(), "a queued change is not a missing config");
        assertFalse(e.isUpToDate());

        t.retryPass();
        assertEquals(1, newCalls.get());
        assertTrue(e.isUpToDate());
    }

    @Test
    void failedRuntimeChangeCountsAsNotApplied() {
        CanConfigRetry t = tracker(10);
        CanConfigRetry.Entry e = boot(t, failing(0, new AtomicInteger()));
        e.setConfig(timeout -> FAIL, now);
        t.retryPass();
        assertFalse(e.isApplied());
    }

    @Test
    void changeHandedInDuringAnAttemptIsAppliedAfterIt() {
        CanConfigRetry t = tracker(10);
        AtomicInteger newer = new AtomicInteger();
        CanConfigRetry.Entry[] entry = new CanConfigRetry.Entry[1];
        AtomicBoolean first = new AtomicBoolean(true);
        entry[0] =
                t.defer(
                        "Hood",
                        null,
                        15,
                        "systemcore:2",
                        CanConfigRetry.CONFIG,
                        true,
                        timeout -> {
                            if (first.getAndSet(false)) {
                                // The main thread edits the config while this one is applying.
                                entry[0].setConfig(failing(0, newer), now);
                            }
                            return StatusCode.OK;
                        });
        t.retryPass();
        assertTrue(entry[0].isApplied());
        assertFalse(entry[0].isUpToDate(), "the older config's success must not count");
        t.retryPass();
        assertEquals(1, newer.get());
        assertTrue(entry[0].isUpToDate());
    }

    @Test
    void deferredConfigIsNotAppliedUntilTheRetryThreadRuns() {
        CanConfigRetry t = tracker(10);
        AtomicInteger calls = new AtomicInteger();
        CanConfigRetry.Entry e =
                t.defer(
                        "Launcher",
                        null,
                        46,
                        "systemcore:2",
                        CanConfigRetry.SIGNALS,
                        false,
                        failing(0, calls));
        assertEquals(0, calls.get());
        assertFalse(e.isApplied());
        t.retryPass();
        assertEquals(1, calls.get());
        assertTrue(e.isApplied());
    }

    @Test
    void deviceResetReappliesItsConfig() {
        AtomicBoolean reset = new AtomicBoolean(true); // the boot the program already configured
        CanConfigRetry.Entry e =
                new CanConfigRetry.Entry(
                        "Hood", 15, "systemcore:2", CanConfigRetry.CONFIG, null, reset::get, true);
        AtomicInteger calls = new AtomicInteger();
        e.setConfig(failing(0, calls), now);
        e.attempt(fn -> fn.apply(0.1), () -> now);
        assertEquals(1, calls.get());

        e.pollReset(now); // first look: swallowed
        assertFalse(e.isDue(now));

        reset.set(false);
        e.pollReset(now);
        assertFalse(e.isDue(now));

        reset.set(true); // a brownout
        e.pollReset(now);
        assertEquals(1, e.getResets());
        assertTrue(e.isApplied(), "flash still holds the config until an attempt says otherwise");
        assertTrue(e.isDue(now));
        e.attempt(fn -> fn.apply(0.1), () -> now);
        assertEquals(2, calls.get());
        assertTrue(e.isUpToDate());
    }
}
