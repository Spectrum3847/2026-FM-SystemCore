package frc.spectrumLib.params;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.DoubleNode;
import java.util.function.DoubleSupplier;

/** A finite number, with inclusive bounds. */
public final class DoubleParam extends Param<Double> implements DoubleSupplier {
    private final double min;
    private final double max;

    DoubleParam(
            String group, String key, double defaultValue, String units, double min, double max) {
        super(group, key, defaultValue, units);
        if (!(min <= max)) {
            throw new IllegalArgumentException("Param " + path() + " min > max");
        }
        this.min = min;
        this.max = max;
        checkDefault();
    }

    @Override
    public double getAsDouble() {
        return get();
    }

    @Override
    public Type type() {
        return Type.NUMBER;
    }

    @Override
    public double min() {
        return min;
    }

    @Override
    public double max() {
        return max;
    }

    @Override
    String check(Double v) {
        if (!Double.isFinite(v)) {
            return "is not a finite number";
        }
        if (v < min || v > max) {
            return "is outside [" + min + ", " + max + "]";
        }
        return null;
    }

    @Override
    Double fromJson(JsonNode node) {
        if (!node.isNumber()) {
            throw new IllegalArgumentException("expected a number, got " + node);
        }
        return validated(node.doubleValue());
    }

    @Override
    JsonNode toJson(Double v) {
        return DoubleNode.valueOf(v);
    }

    @Override
    Double fromLive(Object o) {
        if (!(o instanceof Number n)) {
            throw new IllegalArgumentException("expected a number, got " + o);
        }
        return validated(n.doubleValue());
    }
}
