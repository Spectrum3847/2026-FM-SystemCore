package frc.spectrumLib.params;

import java.util.function.Consumer;

/**
 * Declares the params of one group. The group name is the file name: group {@code cameras} reads
 * {@code params/cameras.json}. Get one from {@link Params#group(String)}.
 *
 * <pre>{@code
 * private static final ParamGroup CAMERAS = Params.group("cameras");
 * public static final DoubleParam RESEND_SECONDS =
 *         CAMERAS.number("settingsResendPeriodSeconds", 2.0, "s", 0.5, 30);
 * }</pre>
 */
public final class ParamGroup {
    private final String name;
    private final Consumer<Param<?>> registrar;

    ParamGroup(String name, Consumer<Param<?>> registrar) {
        this.name = name;
        this.registrar = registrar;
    }

    public String name() {
        return name;
    }

    /** A number with no bounds beyond being finite. */
    public DoubleParam number(String key, double defaultValue, String units) {
        return number(key, defaultValue, units, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
    }

    /** A number within {@code [min, max]}; a file or dashboard value outside it is refused. */
    public DoubleParam number(
            String key, double defaultValue, String units, double min, double max) {
        return add(new DoubleParam(name, key, defaultValue, units, min, max));
    }

    public IntParam integer(String key, int defaultValue, String units) {
        return integer(key, defaultValue, units, Integer.MIN_VALUE, Integer.MAX_VALUE);
    }

    /** A whole number within {@code [min, max]}. */
    public IntParam integer(String key, int defaultValue, String units, int min, int max) {
        return add(new IntParam(name, key, defaultValue, units, min, max));
    }

    public BooleanParam bool(String key, boolean defaultValue) {
        return add(new BooleanParam(name, key, defaultValue));
    }

    public StringParam string(String key, String defaultValue) {
        return add(new StringParam(name, key, defaultValue));
    }

    private <P extends Param<?>> P add(P param) {
        registrar.accept(param);
        return param;
    }
}
