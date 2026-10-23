package frc.spectrumLib.params;

import com.fasterxml.jackson.databind.JsonNode;
import java.util.Objects;
import java.util.regex.Pattern;

/**
 * One tunable value, declared in code with a name, type, default, units and optional bounds, and
 * resolved from the layers described in {@link Params}. Declare params through a {@link
 * ParamGroup}. Read and change them on the main thread only.
 *
 * @param <T> the value type
 */
public abstract sealed class Param<T> permits DoubleParam, IntParam, BooleanParam, StringParam {
    /** The value types a params file and the transport can carry. */
    public enum Type {
        NUMBER,
        INTEGER,
        BOOLEAN,
        STRING
    }

    /** Group and key names: they become file names, JSON keys, log keys and NT topics. */
    private static final Pattern NAME = Pattern.compile("[A-Za-z0-9_.-]+");

    private final String group;
    private final String key;
    private final T defaultValue;
    private final String units;

    private T value;
    private T baseValue;
    private ParamSource source = ParamSource.DEFAULT;
    private int version;

    Param(String group, String key, T defaultValue, String units) {
        requireName(group, "group");
        requireName(key, "key");
        this.group = group;
        this.key = key;
        this.defaultValue = Objects.requireNonNull(defaultValue, "default");
        this.units = units == null ? "" : units;
        this.value = defaultValue;
        this.baseValue = defaultValue;
    }

    private static void requireName(String name, String what) {
        if (name == null || !NAME.matcher(name).matches()) {
            throw new IllegalArgumentException(
                    "Param " + what + " '" + name + "' must match " + NAME.pattern());
        }
    }

    /** Throws if the default breaks the bounds. Subclasses call it once their bounds are set. */
    final void checkDefault() {
        String why = check(defaultValue);
        if (why != null) {
            throw new IllegalArgumentException(
                    "Param " + path() + " default " + defaultValue + " " + why);
        }
    }

    /** The current value. */
    public T get() {
        return value;
    }

    /** The default in the declaration. */
    public T getDefault() {
        return defaultValue;
    }

    public String group() {
        return group;
    }

    public String key() {
        return key;
    }

    /** {@code group/key}, the name used in logs, NT and messages. */
    public String path() {
        return group + "/" + key;
    }

    public String units() {
        return units;
    }

    /** Which layer the current value came from. */
    public ParamSource source() {
        return source;
    }

    /**
     * Goes up by one each time the value changes, for code that must act on a change (re-sending a
     * setting to a device, say): keep the last version seen and compare.
     */
    public int version() {
        return version;
    }

    public abstract Type type();

    /** Lower bound, or NaN when the type has none. */
    public double min() {
        return Double.NaN;
    }

    /** Upper bound, or NaN when the type has none. */
    public double max() {
        return Double.NaN;
    }

    /** Null if the value is acceptable, otherwise why not, phrased to follow the value. */
    abstract String check(T v);

    /** Converts a JSON value, or throws {@link IllegalArgumentException} saying why it cannot. */
    abstract T fromJson(JsonNode node);

    abstract JsonNode toJson(T v);

    /** Converts a value from the transport (a Double, Boolean or String), or throws. */
    abstract T fromLive(Object o);

    final T validated(T v) {
        String why = check(v);
        if (why != null) {
            throw new IllegalArgumentException(v + " " + why);
        }
        return v;
    }

    /** The value from the layers below the robot-local file: what a save compares against. */
    final T base() {
        return baseValue;
    }

    final JsonNode valueJson() {
        return toJson(value);
    }

    final JsonNode baseJson() {
        return toJson(baseValue);
    }

    final void setBase(T v) {
        baseValue = v;
    }

    final void set(T v, ParamSource from) {
        if (!Objects.equals(v, value)) {
            version++;
        }
        value = v;
        source = from;
    }

    /** For values whose type was checked by {@link #type()}: the log and the transport. */
    @SuppressWarnings("unchecked")
    final void setUnchecked(Object v, ParamSource from) {
        set((T) v, from);
    }
}
