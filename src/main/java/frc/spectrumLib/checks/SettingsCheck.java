package frc.spectrumLib.checks;

import frc.spectrumLib.checks.CheckResult.Status;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

/**
 * Flags settings that are not at their defaults: a field layout other than the season's, a practice
 * profile left on, a dashboard switch flipped in the pits. Each may be on purpose; the check makes
 * sure someone meant it.
 */
public final class SettingsCheck implements Check {

    private record Entry(Supplier<String> changed, Status status) {}

    private final List<Entry> entries = new ArrayList<>();
    private Supplier<String> info = () -> "";

    /**
     * Adds a setting.
     *
     * @param changed what is not at its default, as a short phrase (e.g. {@code "Orin practice
     *     profile ON"}), or null when it is at its default
     * @param status how bad a change is
     */
    public SettingsCheck add(Supplier<String> changed, Status status) {
        entries.add(new Entry(changed, status));
        return this;
    }

    /**
     * Sets a line shown when everything is at its default, e.g. which field layout is loaded.
     *
     * @param info the line
     */
    public SettingsCheck info(Supplier<String> info) {
        this.info = info;
        return this;
    }

    @Override
    public CheckResult evaluate(double now) {
        Status worst = Status.PASS;
        List<String> changed = new ArrayList<>();
        for (Entry e : entries) {
            String c = e.changed.get();
            if (c != null) {
                changed.add(c);
                worst = worst.worst(e.status);
            }
        }
        if (changed.isEmpty()) {
            String i = info.get();
            return CheckResult.pass(i.isEmpty() ? "all at defaults" : i + "; rest at defaults");
        }
        return new CheckResult(worst, "not default: " + String.join("; ", changed));
    }
}
