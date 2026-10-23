package frc.spectrumLib.params;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.TextNode;
import java.util.function.Supplier;

/** Text, such as a camera name or a mode. */
public final class StringParam extends Param<String> implements Supplier<String> {
    StringParam(String group, String key, String defaultValue) {
        super(group, key, defaultValue, "");
    }

    @Override
    public Type type() {
        return Type.STRING;
    }

    @Override
    String check(String v) {
        return null;
    }

    @Override
    String fromJson(JsonNode node) {
        if (!node.isTextual()) {
            throw new IllegalArgumentException("expected a string, got " + node);
        }
        return node.textValue();
    }

    @Override
    JsonNode toJson(String v) {
        return TextNode.valueOf(v);
    }

    @Override
    String fromLive(Object o) {
        if (!(o instanceof String s)) {
            throw new IllegalArgumentException("expected a string, got " + o);
        }
        return s;
    }
}
