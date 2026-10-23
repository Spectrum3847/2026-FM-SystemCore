package frc.spectrumLib.params;

/**
 * Whether dashboards may change tunable values. Off, every tunable ({@link Param}, {@link
 * frc.spectrumLib.telemetry.TuneValue}) holds its default or params value and nothing is published
 * for tuning. The robot sets it from its own constant before creating anything tunable; until then
 * it is off, so a tunable created too early fails safe.
 */
public final class TuningMode {
    private TuningMode() {}

    private static boolean enabled = false;

    public static void set(boolean on) {
        enabled = on;
    }

    public static boolean enabled() {
        return enabled;
    }
}
