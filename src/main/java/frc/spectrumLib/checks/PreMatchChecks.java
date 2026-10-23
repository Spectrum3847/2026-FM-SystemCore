package frc.spectrumLib.checks;

import frc.spectrumLib.checks.CheckResult.Status;
import frc.spectrumLib.telemetry.Alert;
import frc.spectrumLib.telemetry.Telemetry;
import java.util.ArrayList;
import java.util.List;
import org.wpilib.driverstation.Alert.Level;

/**
 * Pre-match self-checks: a registry of {@link Check}s that run while the robot is disabled and add
 * up to one GO / NO-GO.
 *
 * <p>Each check gives PASS, WARN, FAIL or SKIP with a one-line reason. The robot is NO-GO while any
 * check fails; warnings are shown but do not change GO. Results are outputs, never inputs to
 * anything: the checks are advisory and never stop the robot from enabling.
 *
 * <p>Outputs, under the prefix given (e.g. {@code PreMatch}):
 *
 * <ul>
 *   <li>{@code Go} and {@code Reason}: the verdict and one line saying why, on NetworkTables for
 *       Elastic and the pit app.
 *   <li>{@code Summary}: one line per check, {@code "FAIL CAN: ..."}, on NetworkTables too.
 *   <li>{@code Checks/<name>/Status} and {@code Checks/<name>/Reason}.
 *   <li>{@code Active}: whether the checks are running (disabled). While enabled the last results
 *       stay published but nothing is re-judged.
 * </ul>
 *
 * <p>A failing check raises a HIGH alert and a warning a MEDIUM one, both cleared on enable: in a
 * match the subsystems' own alerts speak for themselves.
 *
 * <p>One check is evaluated per loop, in turn, so the cost is one check's work whatever the number
 * registered. Every check's {@link Check#observe} still runs every loop.
 */
public final class PreMatchChecks {

    /** Where results go. The robot logs and raises alerts; tests collect them. */
    interface Sink {
        void result(String name, CheckResult result);

        void overall(boolean go, String reason, String[] summary, boolean active);

        void clear(String name);
    }

    private final List<Entry> entries = new ArrayList<>();
    private final Sink sink;
    private int next = 0;
    private boolean wasActive = false;
    private boolean go = false;
    private String reason = "Not checked yet";

    private static final class Entry {
        final String name;
        final Check check;
        CheckResult result;

        Entry(String name, Check check) {
            this.name = name;
            this.check = check;
        }
    }

    /**
     * @param prefix log key prefix, e.g. {@code "PreMatch"}
     */
    public PreMatchChecks(String prefix) {
        this.sink = new TelemetrySink(prefix);
    }

    PreMatchChecks(Sink sink) {
        this.sink = sink;
    }

    /**
     * Adds a check. Call at startup, before the first {@link #periodic}.
     *
     * @param name short display name, also its log path ({@code Checks/<name>/...})
     * @param check the check
     * @return this, for chaining
     */
    public PreMatchChecks register(String name, Check check) {
        entries.add(new Entry(name, check));
        if (sink instanceof TelemetrySink t) {
            t.addAlerts(name);
        }
        return this;
    }

    /**
     * Runs the checks. Call once per loop on the main thread, after this loop's inputs are read.
     *
     * @param disabled whether the robot is disabled; the checks judge only then
     * @param nowSeconds this loop's robot time
     */
    public void periodic(boolean disabled, double nowSeconds) {
        for (Entry e : entries) {
            try {
                e.check.observe(nowSeconds);
            } catch (RuntimeException ex) {
                // A broken check must never take the robot loop down with it.
                e.result = CheckResult.warn("check error: " + ex);
            }
        }
        if (!disabled) {
            if (wasActive) {
                wasActive = false;
                for (Entry e : entries) {
                    sink.clear(e.name);
                }
                sink.overall(go, reason, summary(), false);
            }
            return;
        }
        if (!wasActive) {
            // Fresh results only: nothing judged before the last enable counts now.
            wasActive = true;
            next = 0;
            for (Entry e : entries) {
                e.result = null;
            }
        }
        if (entries.isEmpty()) {
            go = true;
            reason = "GO: no checks registered";
            sink.overall(go, reason, new String[0], true);
            return;
        }
        Entry e = entries.get(next);
        next = (next + 1) % entries.size();
        CheckResult r;
        try {
            r = e.check.evaluate(nowSeconds);
        } catch (RuntimeException ex) {
            r = CheckResult.warn("check error: " + ex);
        }
        e.result = r == null ? CheckResult.warn("check returned nothing") : r;
        sink.result(e.name, e.result);
        summarize();
        sink.overall(go, reason, summary(), true);
    }

    /** Whether no check fails (and every check has been judged since the robot disabled). */
    public boolean isGo() {
        return go;
    }

    /** One line: GO or NO-GO, and the first failing (else warning) check's reason. */
    public String getReason() {
        return reason;
    }

    /** The latest result of a check, or null before it has been judged since the last disable. */
    public CheckResult resultOf(String name) {
        for (Entry e : entries) {
            if (e.name.equals(name)) {
                return e.result;
            }
        }
        return null;
    }

    private void summarize() {
        int pending = 0;
        List<Entry> fails = new ArrayList<>();
        List<Entry> warns = new ArrayList<>();
        for (Entry e : entries) {
            if (e.result == null) {
                pending++;
            } else if (e.result.isFail()) {
                fails.add(e);
            } else if (e.result.isWarn()) {
                warns.add(e);
            }
        }
        go = fails.isEmpty() && pending == 0;
        if (!fails.isEmpty()) {
            reason = "NO-GO: " + first(fails);
        } else if (pending > 0) {
            reason = "Checking (" + (entries.size() - pending) + " of " + entries.size() + ")";
        } else if (!warns.isEmpty()) {
            reason =
                    "GO, "
                            + warns.size()
                            + (warns.size() == 1 ? " warning: " : " warnings: ")
                            + first(warns);
        } else {
            reason = "GO: all " + entries.size() + " checks pass";
        }
    }

    private static String first(List<Entry> list) {
        Entry e = list.get(0);
        return e.name
                + ": "
                + e.result.reason()
                + (list.size() > 1 ? " (+" + (list.size() - 1) + " more)" : "");
    }

    private String[] summary() {
        String[] out = new String[entries.size()];
        for (int i = 0; i < out.length; i++) {
            Entry e = entries.get(i);
            out[i] =
                    e.result == null
                            ? "...  " + e.name
                            : e.result.status() + " " + e.name + ": " + e.result.reason();
        }
        return out;
    }

    /** Logs every result and raises an alert per failing or warning check. */
    private static final class TelemetrySink implements Sink {
        private final String prefix;
        private final java.util.Map<String, Alert[]> alerts = new java.util.HashMap<>();
        private final java.util.Map<String, String[]> keys = new java.util.HashMap<>();

        TelemetrySink(String prefix) {
            this.prefix = prefix;
            Telemetry.addDashboardKey(prefix + "/Summary");
        }

        void addAlerts(String name) {
            alerts.put(name, new Alert[] {new Alert("", Level.HIGH), new Alert("", Level.MEDIUM)});
            keys.put(
                    name,
                    new String[] {
                        prefix + "/Checks/" + name + "/Status",
                        prefix + "/Checks/" + name + "/Reason"
                    });
        }

        @Override
        public void result(String name, CheckResult r) {
            String[] k = keys.get(name);
            Telemetry.logDash(k[0], r.status().name());
            Telemetry.logDash(k[1], r.reason());
            Alert[] a = alerts.get(name);
            if (r.status() == Status.FAIL) {
                a[0].setText("Pre-match NO-GO, " + name + ": " + r.reason());
            } else if (r.status() == Status.WARN) {
                a[1].setText("Pre-match, " + name + ": " + r.reason());
            }
            a[0].set(r.status() == Status.FAIL);
            a[1].set(r.status() == Status.WARN);
        }

        @Override
        public void overall(boolean go, String reason, String[] summary, boolean active) {
            Telemetry.logDash(prefix + "/Go", go);
            Telemetry.logDash(prefix + "/Reason", reason);
            Telemetry.log(prefix + "/Active", active);
            Telemetry.log(prefix + "/Summary", summary);
        }

        @Override
        public void clear(String name) {
            for (Alert a : alerts.get(name)) {
                a.set(false);
            }
        }
    }
}
