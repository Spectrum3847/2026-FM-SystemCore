package frc.spectrumLib.checks;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.spectrumLib.checks.CheckResult.Status;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

class PreMatchChecksTest {

    /** Records what the registry publishes. */
    private static final class Recorder implements PreMatchChecks.Sink {
        final List<String> judged = new ArrayList<>();
        final List<String> cleared = new ArrayList<>();
        boolean go;
        String reason;
        boolean active;

        @Override
        public void result(String name, CheckResult result) {
            judged.add(name);
        }

        @Override
        public void overall(boolean go, String reason, String[] summary, boolean active) {
            this.go = go;
            this.reason = reason;
            this.active = active;
        }

        @Override
        public void clear(String name) {
            cleared.add(name);
        }
    }

    private static final class Fixed implements Check {
        CheckResult result;
        int evaluations = 0;
        int observations = 0;

        Fixed(CheckResult result) {
            this.result = result;
        }

        @Override
        public void observe(double now) {
            observations++;
        }

        @Override
        public CheckResult evaluate(double now) {
            evaluations++;
            return result;
        }
    }

    private final Recorder sink = new Recorder();
    private final PreMatchChecks checks = new PreMatchChecks(sink);

    private void loops(int n, boolean disabled) {
        for (int i = 0; i < n; i++) {
            checks.periodic(disabled, i * 0.02);
        }
    }

    @Test
    void judgesOneCheckPerLoopInTurn() {
        Fixed a = new Fixed(CheckResult.pass("a"));
        Fixed b = new Fixed(CheckResult.pass("b"));
        Fixed c = new Fixed(CheckResult.pass("c"));
        checks.register("A", a).register("B", b).register("C", c);
        loops(7, true);
        assertEquals(List.of("A", "B", "C", "A", "B", "C", "A"), sink.judged);
        assertEquals(7, a.observations, "every check observes every loop");
        assertEquals(3, a.evaluations);
    }

    @Test
    void notGoUntilEveryCheckHasBeenJudged() {
        checks.register("A", new Fixed(CheckResult.pass("a")))
                .register("B", new Fixed(CheckResult.pass("b")));
        loops(1, true);
        assertFalse(checks.isGo());
        assertEquals("Checking (1 of 2)", checks.getReason());
        loops(1, true);
        assertTrue(checks.isGo());
        assertEquals("GO: all 2 checks pass", checks.getReason());
    }

    @Test
    void aFailureIsNoGoAndNamesTheCheck() {
        checks.register("CAN", new Fixed(CheckResult.fail("CANivore: Hood (15) not answering")))
                .register("Battery", new Fixed(CheckResult.warn("12.2 V at rest")))
                .register("Auto", new Fixed(CheckResult.fail("no paths")));
        loops(3, true);
        assertFalse(checks.isGo());
        assertEquals("NO-GO: CAN: CANivore: Hood (15) not answering (+1 more)", checks.getReason());
        assertTrue(sink.go == checks.isGo() && sink.active);
    }

    @Test
    void warningsKeepGo() {
        checks.register("Battery", new Fixed(CheckResult.warn("12.2 V at rest")))
                .register("Settings", new Fixed(CheckResult.pass("defaults")))
                .register("Coprocessors", new Fixed(CheckResult.skip("none")));
        loops(3, true);
        assertTrue(checks.isGo());
        assertEquals("GO, 1 warning: Battery: 12.2 V at rest", checks.getReason());
    }

    @Test
    void enablingStopsJudgingAndClearsAlertsAndDisablingStartsFresh() {
        Fixed a = new Fixed(CheckResult.fail("bad"));
        checks.register("A", a);
        loops(3, true);
        assertEquals(3, a.evaluations);
        loops(5, false);
        assertEquals(3, a.evaluations, "never judged while enabled");
        assertEquals(List.of("A"), sink.cleared, "alerts cleared once, on enable");
        assertFalse(sink.active);
        assertEquals(8, a.observations, "observe still runs while enabled");

        a.result = CheckResult.pass("fixed");
        checks.periodic(true, 1.0);
        assertEquals(Status.PASS, checks.resultOf("A").status());
        assertTrue(checks.isGo());
    }

    @Test
    void resultsFromBeforeTheEnableDoNotCountAfterIt() {
        checks.register("A", new Fixed(CheckResult.pass("a")))
                .register("B", new Fixed(CheckResult.pass("b")));
        loops(2, true);
        loops(1, false);
        checks.periodic(true, 1.0);
        assertNull(checks.resultOf("B"), "B not judged again yet");
        assertFalse(checks.isGo());
    }

    @Test
    void aCheckThatThrowsWarnsInsteadOfStoppingTheLoop() {
        checks.register(
                        "Broken",
                        now -> {
                            throw new IllegalStateException("boom");
                        })
                .register("Fine", new Fixed(CheckResult.pass("ok")));
        loops(2, true);
        assertEquals(Status.WARN, checks.resultOf("Broken").status());
        assertTrue(checks.isGo(), "a broken check is not a robot fault");
    }

    @Test
    void severityOrder() {
        assertEquals(Status.FAIL, Status.WARN.worst(Status.FAIL));
        assertEquals(Status.WARN, Status.WARN.worst(Status.PASS));
        assertEquals(Status.PASS, Status.SKIP.worst(Status.PASS));
    }
}
