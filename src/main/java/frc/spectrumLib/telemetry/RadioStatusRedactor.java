package frc.spectrumLib.telemetry;

import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.JsonNodeFactory;
import com.fasterxml.jackson.databind.node.ObjectNode;
import java.util.Map;
import java.util.Set;
import org.littletonrobotics.junction.LogDataReceiver;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.LogTable.LogValue;
import org.littletonrobotics.junction.LogTable.LoggableType;
import org.littletonrobotics.junction.Logger;

/**
 * Rewrites the radio's status JSON in {@value #KEY} before any log file or dashboard gets it,
 * keeping only fields known to be safe and blanking the rest.
 *
 * <p>AdvantageKit logs the radio's {@code /status} JSON every cycle and has no switch to turn that
 * off. Logs go to the public log site, and the status holds the Wi-Fi key hashes, their salts and
 * MAC addresses: a salted hash is enough to try the Wi-Fi key offline. A list of fields to blank
 * misses whatever the next radio firmware adds, so this keeps a list of fields to log instead
 * ({@link #ALLOWED}), in the status and in its {@code networkStatus24} / {@code networkStatus6}
 * objects. Every other value keeps its type but not its content: strings become {@code ""}, numbers
 * {@code 0}, booleans {@code false}, arrays and objects empty. A status that does not parse becomes
 * {@code {}}.
 *
 * <p>Every data receiver is handed the same per-cycle copy of the log table, in the order they were
 * added, so this one must be added before all the others ({@link #addFirst}). It runs on
 * AdvantageKit's receiver thread and only rewrites that copy.
 */
public final class RadioStatusRedactor implements LogDataReceiver {
    static final String KEY = "RadioStatus/Status";

    /** What a status that cannot be parsed is replaced with. */
    static final String UNPARSEABLE = "{}";

    /** Fields logged as they are, at the top level and inside the network objects. */
    static final Set<String> ALLOWED =
            Set.of(
                    "mode",
                    "channel",
                    "teamNumber",
                    "ssidSuffix",
                    "ssid",
                    "isEnabled",
                    "isLinked",
                    "dataAgeMs",
                    "signalDbm",
                    "noiseDbm",
                    "signalNoiseRatio",
                    "rxRateMbps",
                    "rxPackets",
                    "rxBytes",
                    "txRateMbps",
                    "txPackets",
                    "txBytes",
                    "bandwidthUsedMbps",
                    "connectionQuality",
                    "status",
                    "version",
                    "sixGhzBypassed",
                    "qosEnabled",
                    "systemcoreEnabled");

    /** Objects whose fields go through the same allowlist. */
    static final Set<String> NETWORKS = Set.of("networkStatus24", "networkStatus6");

    private static final ObjectMapper MAPPER =
            new ObjectMapper().enable(DeserializationFeature.FAIL_ON_TRAILING_TOKENS);

    // The radio is polled every 5 s, so most cycles repeat the last status.
    private String lastIn;
    private String lastOut;

    /**
     * Adds a redactor as AdvantageKit's first data receiver, then the others in order. The way to
     * add receivers, so none can be added ahead of the redactor.
     *
     * @param others the log writer, the dashboard publisher, ...
     */
    public static void addFirst(LogDataReceiver... others) {
        Logger.addDataReceiver(new RadioStatusRedactor());
        for (LogDataReceiver r : others) {
            Logger.addDataReceiver(r);
        }
    }

    @Override
    public void putTable(LogTable table) {
        LogValue v = table.get(KEY);
        if (v == null || v.type != LoggableType.String) {
            return;
        }
        String in = v.getString();
        if (!in.equals(lastIn)) {
            lastIn = in;
            lastOut = redact(in);
        }
        if (!lastOut.equals(in)) {
            table.put(KEY, new LogValue(lastOut, v.customTypeStr));
        }
    }

    /**
     * Returns the status JSON with only the allowed fields' values kept.
     *
     * @param json the radio's status JSON
     * @return it, compact, with every other value blanked; {@value #UNPARSEABLE} if it does not
     *     parse as a JSON object
     */
    static String redact(String json) {
        try {
            JsonNode root = MAPPER.readTree(json);
            if (root == null || !root.isObject()) {
                return UNPARSEABLE;
            }
            return MAPPER.writeValueAsString(filter((ObjectNode) root));
        } catch (Exception e) {
            return UNPARSEABLE;
        }
    }

    private static ObjectNode filter(ObjectNode in) {
        ObjectNode out = JsonNodeFactory.instance.objectNode();
        for (Map.Entry<String, JsonNode> field : in.properties()) {
            String name = field.getKey();
            JsonNode value = field.getValue();
            if (ALLOWED.contains(name) && value.isValueNode()) {
                out.set(name, value);
            } else if (NETWORKS.contains(name) && value.isObject()) {
                out.set(name, filter((ObjectNode) value));
            } else {
                out.set(name, blank(value));
            }
        }
        return out;
    }

    /** The same type with nothing in it. */
    private static JsonNode blank(JsonNode value) {
        JsonNodeFactory f = JsonNodeFactory.instance;
        if (value.isTextual()) {
            return f.textNode("");
        }
        if (value.isNumber()) {
            return f.numberNode(0);
        }
        if (value.isBoolean()) {
            return f.booleanNode(false);
        }
        if (value.isArray()) {
            return f.arrayNode();
        }
        if (value.isObject()) {
            return f.objectNode();
        }
        return f.nullNode();
    }
}
