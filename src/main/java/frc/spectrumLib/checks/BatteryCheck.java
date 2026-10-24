package frc.spectrumLib.checks;

import java.util.ArrayList;
import java.util.List;
import java.util.OptionalDouble;
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
 * <p>The thresholds are for the voltage at the battery. Each controller reads it a little off, so
 * the reading is corrected by the robot's calibration offset. A robot whose offset has not been
 * measured warns at most on voltage: a FAIL there could be the controller, not the battery.
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
    private final OptionalDouble offsetVolts;
    private final BooleanSupplier atRest;
    private final Supplier<Identity> identity;
    private double underWarnSince = Double.NaN;
    private double underFailSince = Double.NaN;
    private double lastVolts = Double.NaN;

    /**
     * A check on a reading that is already calibrated (offset 0).
     *
     * @param volts battery voltage, a logged input
     * @param atRest whether the robot is disabled; sag while enabled is load, not charge
     * @param identity the battery in the robot and the one in the last match; {@link Identity#NONE}
     *     without a reader
     */
    public BatteryCheck(DoubleSupplier volts, BooleanSupplier atRest, Supplier<Identity> identity) {
        this(volts, OptionalDouble.of(0), atRest, identity);
    }

    /**
     * @param volts battery voltage as the controller reads it, a logged input
     * @param offsetVolts added to the reading to give the voltage at the battery; empty when it has
     *     not been measured on this robot
     * @param atRest whether the robot is disabled; sag while enabled is load, not charge
     * @param identity the battery in the robot and the one in the last match; {@link Identity#NONE}
     *     without a reader
     */
    public BatteryCheck(
            DoubleSupplier volts,
            OptionalDouble offsetVolts,
            BooleanSupplier atRest,
            Supplier<Identity> identity) {
        this.volts = volts;
        this.offsetVolts = offsetVolts;
        this.atRest = atRest;
        this.identity = identity;
    }

    @Override
    public void observe(double now) {
        double raw = volts.getAsDouble();
        boolean reading = raw > NO_READING_VOLTS;
        double v = reading ? raw + offsetVolts.orElse(0) : raw;
        lastVolts = v;
        reading &= atRest.getAsBoolean();
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
        boolean calibrated = offsetVolts.isPresent();
        if (held(underFailSince, now)) {
            return calibrated
                    ? CheckResult.fail(volt + idText + ": swap the battery")
                    : CheckResult.warn(
                            volt + idText + ": swap the battery? (voltage reading not calibrated)");
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
            return CheckResult.warn(
                    volt
                            + idText
                            + ": not fully charged"
                            + (calibrated ? "" : " (voltage reading not calibrated)"));
        }
        List<String> notes = new ArrayList<>();
        if (!known) {
            notes.add("no battery ID reader");
        }
        if (!calibrated) {
            notes.add("voltage reading not calibrated");
        }
        return CheckResult.pass(
                volt + idText + (notes.isEmpty() ? "" : " (" + String.join(", ", notes) + ")"));
    }
}
