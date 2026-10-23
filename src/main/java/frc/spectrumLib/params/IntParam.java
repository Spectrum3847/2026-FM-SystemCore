package frc.spectrumLib.params;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.IntNode;
import java.util.function.IntSupplier;

/** A whole number, with inclusive bounds. NT carries it as a double, so 2.0 reads as 2. */
public final class IntParam extends Param<Integer> implements IntSupplier {
    private final int min;
    private final int max;

    IntParam(String group, String key, int defaultValue, String units, int min, int max) {
        super(group, key, defaultValue, units);
        if (min > max) {
            throw new IllegalArgumentException("Param " + path() + " min > max");
        }
        this.min = min;
        this.max = max;
        checkDefault();
    }

    @Override
    public int getAsInt() {
        return get();
    }

    @Override
    public Type type() {
        return Type.INTEGER;
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
    String check(Integer v) {
        if (v < min || v > max) {
            return "is outside [" + min + ", " + max + "]";
        }
        return null;
    }

    @Override
    Integer fromJson(JsonNode node) {
        if (!node.isNumber()) {
            throw new IllegalArgumentException("expected a whole number, got " + node);
        }
        return validated(toInt(node.doubleValue(), node.toString()));
    }

    @Override
    JsonNode toJson(Integer v) {
        return IntNode.valueOf(v);
    }

    @Override
    Integer fromLive(Object o) {
        if (!(o instanceof Number n)) {
            throw new IllegalArgumentException("expected a whole number, got " + o);
        }
        return validated(toInt(n.doubleValue(), o.toString()));
    }

    private static int toInt(double d, String text) {
        if (d != Math.rint(d) || d < Integer.MIN_VALUE || d > Integer.MAX_VALUE) {
            throw new IllegalArgumentException("expected a whole number, got " + text);
        }
        return (int) d;
    }
}
