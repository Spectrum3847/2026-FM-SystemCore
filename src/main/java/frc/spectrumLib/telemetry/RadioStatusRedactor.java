package frc.spectrumLib.telemetry;

import java.util.regex.Pattern;
import org.littletonrobotics.junction.LogDataReceiver;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.LogTable.LogValue;
import org.littletonrobotics.junction.LogTable.LoggableType;

/**
 * Blanks the radio's Wi-Fi key hashes, their salts and its MAC address in {@value #KEY} before any
 * log file or dashboard gets it.
 *
 * <p>AdvantageKit logs the radio's {@code /status} JSON every cycle and has no switch to turn that
 * off. Logs go to the public log site, and a salted hash is enough to try the Wi-Fi key offline.
 * Every data receiver is handed the same per-cycle copy of the log table, in the order they were
 * added, so this one must be added before all the others. It runs on AdvantageKit's receiver thread
 * and only rewrites that copy.
 */
public final class RadioStatusRedactor implements LogDataReceiver {
    static final String KEY = "RadioStatus/Status";

    /** A JSON string field whose name says key, salt, password or MAC, e.g. hashedWpaKey. */
    private static final Pattern SECRET =
            Pattern.compile(
                    "(\"[A-Za-z0-9_]*(?i:key|salt|pass|mac)[A-Za-z0-9_]*\"\\s*:\\s*)\"[^\"]*\"");

    // The radio is polled every 5 s, so most cycles repeat the last status.
    private String lastIn;
    private String lastOut;

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
     * Returns the status JSON with every key, salt, password and MAC value emptied.
     *
     * @param json the radio's status JSON
     * @return the same JSON with those values set to {@code ""}
     */
    static String redact(String json) {
        return SECRET.matcher(json).replaceAll("$1\"\"");
    }
}
