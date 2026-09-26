package frc.spectrumLib.telemetry;

import java.util.function.DoubleSupplier;
import lombok.Getter;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * A number that can be changed from a dashboard at runtime without redeploying, for tuning gains,
 * speeds and other constants.
 *
 * <p>Backed by AdvantageKit's {@link LoggedNetworkNumber} (in 2026 it read SmartDashboard
 * directly), so the value is a logged input: a replayed match sees exactly the values the robot was
 * tuned to at every moment. Published under {@code /Tuning/<name>}.
 */
public class TuneValue {
    private final LoggedNetworkNumber number;

    /** Current value, refreshed on each call to {@link #update()}. */
    @Getter private double value;

    /** Dashboard key under which this value is published and read. */
    @Getter private final String name;

    /**
     * Creates a TuneValue, publishing {@code defaultValue}.
     *
     * @param name dashboard key
     * @param defaultValue initial value
     */
    public TuneValue(String name, double defaultValue) {
        this.name = name;
        this.value = defaultValue;
        this.number = new LoggedNetworkNumber("/Tuning/" + name, defaultValue);
    }

    /**
     * Reads the current value and caches it locally.
     *
     * @return the latest value
     */
    public Double update() {
        value = number.get();
        return value;
    }

    /**
     * Returns a {@link DoubleSupplier} that calls {@link #update()} each time it is queried.
     *
     * @return a supplier backed by this TuneValue
     */
    public DoubleSupplier getSupplier() {
        return this::update;
    }
}
