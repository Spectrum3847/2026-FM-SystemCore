package frc.spectrumLib.checks;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * The battery at rest is charged, and is not the one used in the last match.
 *
 * <p>Voltage: disabled, the robot draws only its controllers and coprocessors, so the reading is
 * close to the pack's resting voltage. It must stay under a threshold for {@link #HOLD_SECONDS}
 * before it counts, so a mechanism moved by hand or a camera booting does not trip it.
 *
 * <p>Same battery (6328 2024): when a match starts, the battery's ID is saved; at the next boot,
 * the same ID means nobody swapped it. Needs a battery ID (an NFC tag reader); without one this
 * part is skipped.
 */
public final class BatteryCheck implements Check {

    /**
     * A full pack reads 12.6-12.9 V disabled on FM's SystemCore (TXMAN1 pre-match, nine logs:
     * lowest 12.57 V). Below this, it has been used.
     */
    public static final double WARN_VOLTS = 12.3;

    /** Below this at rest, a match's load will brown it out. */
    public static final double FAIL_VOLTS = 11.8;

    public static final double HOLD_SECONDS = 3.0;

    /** Readings at or below this mean no reading, not an empty battery. */
    private static final double NO_READING_VOLTS = 1.0;

    /** The battery IDs this check compares, from logged inputs. */
    public record Identity(String current, String lastMatch) {
        public static final Identity NONE = new Identity("", "");
    }

    private final DoubleSupplier volts;
    private final BooleanSupplier atRest;
    private final Supplier<Identity> identity;
    private double underWarnSince = Double.NaN;
    private double underFailSince = Double.NaN;
    private double lastVolts = Double.NaN;

    /**
     * @param volts battery voltage, a logged input
     * @param atRest whether the robot is disabled; sag while enabled is load, not charge
     * @param identity the battery in the robot and the one in the last match; {@link Identity#NONE}
     *     without a reader
     */
    public BatteryCheck(DoubleSupplier volts, BooleanSupplier atRest, Supplier<Identity> identity) {
        this.volts = volts;
        this.atRest = atRest;
        this.identity = identity;
    }

    @Override
    public void observe(double now) {
        double v = volts.getAsDouble();
        lastVolts = v;
        boolean reading = v > NO_READING_VOLTS && atRest.getAsBoolean();
        underWarnSince = track(reading && v < WARN_VOLTS, underWarnSince, now);
        underFailSince = track(reading && v < FAIL_VOLTS, underFailSince, now);
    }

    private static double track(boolean under, double since, double now) {
        if (!under) {
            return Double.NaN;
        }
        return Double.isNaN(since) ? now : since;
    }

    private static boolean held(double since, double now) {
        return !Double.isNaN(since) && now - since >= HOLD_SECONDS;
    }

    @Override
    public CheckResult evaluate(double now) {
        Identity id = identity.get();
        boolean known = id != null && !id.current().isEmpty();
        boolean same = known && id.current().equals(id.lastMatch());
        String volt = String.format("%.2f V at rest", lastVolts);
        String idText = known ? ", battery " + id.current() : "";
        if (!(lastVolts > NO_READING_VOLTS)) {
            return CheckResult.warn("no battery voltage reading");
        }
        if (held(underFailSince, now)) {
            return CheckResult.fail(volt + idText + ": swap the battery");
        }
        if (same) {
            return CheckResult.fail(
                    "battery "
                            + id.current()
                            + " was in the robot last match: swap it ("
                            + volt
                            + ")");
        }
        if (held(underWarnSince, now)) {
            return CheckResult.warn(volt + idText + ": not fully charged");
        }
        return CheckResult.pass(volt + (known ? idText : " (no battery ID reader)"));
    }
}
