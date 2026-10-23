package frc.spectrumLib.checks;

import frc.spectrumLib.mechanism.Mechanism;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

/**
 * Every mechanism that relies on its position knows where it is.
 *
 * <p>A mechanism reports a {@link State}. {@link #addMechanism} covers any soft-limited {@link
 * Mechanism} from what it already tracks ({@link Mechanism#isPositionLost()}); a mechanism with its
 * own homing routine adds a supplier that maps that routine's state.
 */
public final class HomingCheck implements Check {

    public enum State {
        /** Its position is known. */
        HOMED,
        /** Not homed yet, and it homes by itself once enabled (e.g. at auto start). */
        HOMES_WHEN_ENABLED,
        /** Its motor rebooted since it last homed: it holds until it re-homes. */
        LOST,
        /** Its last homing attempt failed. */
        FAILED,
        /** Its motor is not answering, so nothing it reports means anything. */
        DISCONNECTED
    }

    private record Entry(String name, Supplier<State> state) {}

    private final List<Entry> entries = new ArrayList<>();

    /**
     * Adds a mechanism with a state of its own.
     *
     * @param name its name in messages
     * @param state its homing state, from logged inputs
     */
    public void add(String name, Supplier<State> state) {
        entries.add(new Entry(name, state));
    }

    /**
     * Adds a mechanism by its reboot tracking. Skipped unless it is attached and soft-limited (only
     * then does its position matter).
     *
     * @param m the mechanism
     */
    public void addMechanism(Mechanism m) {
        if (!m.isAttached() || !m.hasSoftLimits()) {
            return;
        }
        add(m.getName(), () -> stateOf(m));
    }

    /** A mechanism's state from its reboot tracking alone. */
    public static State stateOf(Mechanism m) {
        if (!m.getInputs().connected) {
            return State.DISCONNECTED;
        }
        return m.isPositionLost() ? State.LOST : State.HOMED;
    }

    @Override
    public CheckResult evaluate(double now) {
        if (entries.isEmpty()) {
            return CheckResult.skip("no mechanism tracks its position");
        }
        List<String> failed = new ArrayList<>();
        List<String> lost = new ArrayList<>();
        List<String> disconnected = new ArrayList<>();
        List<String> later = new ArrayList<>();
        for (Entry e : entries) {
            switch (e.state.get()) {
                case FAILED -> failed.add(e.name);
                case LOST -> lost.add(e.name);
                case DISCONNECTED -> disconnected.add(e.name);
                case HOMES_WHEN_ENABLED -> later.add(e.name);
                case HOMED -> {}
            }
        }
        if (!failed.isEmpty()) {
            return CheckResult.fail(String.join(", ", failed) + " failed to home - check it");
        }
        if (!lost.isEmpty()) {
            return CheckResult.warn(
                    String.join(", ", lost)
                            + " lost position (motor rebooted); holds until it re-homes");
        }
        if (!disconnected.isEmpty()) {
            return CheckResult.warn(
                    String.join(", ", disconnected) + " not answering, homing unknown");
        }
        int homed = entries.size() - later.size();
        return CheckResult.pass(
                homed
                        + " homed"
                        + (later.isEmpty()
                                ? ""
                                : ", " + String.join(", ", later) + " homes when enabled"));
    }
}
