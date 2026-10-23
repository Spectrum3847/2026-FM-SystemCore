package frc.spectrumLib.params;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.BooleanNode;
import java.util.function.BooleanSupplier;

/** A switch. */
public final class BooleanParam extends Param<Boolean> implements BooleanSupplier {
    BooleanParam(String group, String key, boolean defaultValue) {
        super(group, key, defaultValue, "");
    }

    @Override
    public boolean getAsBoolean() {
        return get();
    }

    @Override
    public Type type() {
        return Type.BOOLEAN;
    }

    @Override
    String check(Boolean v) {
        return null;
    }

    @Override
    Boolean fromJson(JsonNode node) {
        if (!node.isBoolean()) {
            throw new IllegalArgumentException("expected true or false, got " + node);
        }
        return node.booleanValue();
    }

    @Override
    JsonNode toJson(Boolean v) {
        return BooleanNode.valueOf(v);
    }

    @Override
    Boolean fromLive(Object o) {
        if (!(o instanceof Boolean b)) {
            throw new IllegalArgumentException("expected true or false, got " + o);
        }
        return b;
    }
}
