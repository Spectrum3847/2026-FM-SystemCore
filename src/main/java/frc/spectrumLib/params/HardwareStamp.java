package frc.spectrumLib.params;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;
import java.util.function.Supplier;

/**
 * Ties the pit-saved file of a hard-stop group (swerve offsets, soft limits) to the robot it was
 * saved on, so a controller moved to another robot does not bring that robot's values with it.
 *
 * <p>The identity is a set of numbers the robot's own devices hold, read at boot before anything
 * writes them. A saved file records the readings that this robot can show at later boots (the
 * {@code afterSave} list). At boot, the file is used only if the readings match one of them.
 * Otherwise, or if the devices could not be read, the file is refused, an alert is raised and the
 * values come from git.
 *
 * <p>The readings are a logged input ({@code Params/Hardware/<name>}). No WPILib here, so the rules
 * can be unit tested.
 */
public final class HardwareStamp {
    /** The key a stamped file keeps its stamp under. Not a param. */
    static final String KEY = "$hardware";

    private final String name;
    private final String what;
    private final double tolerance;
    private final Supplier<List<double[]>> afterSave;
    private double[] atBoot;

    /**
     * @param name the log key, e.g. {@code SwerveMagnetOffsets}
     * @param what what the readings are, for messages, e.g. {@code "swerve CANcoder magnet
     *     offsets"}
     * @param atBoot the readings, or null if a device did not answer
     * @param tolerance how far a reading may be from a stamped one and still match
     * @param afterSave the readings this robot may show at a later boot, given what is about to be
     *     saved; called on the main thread when a save is prepared
     */
    public HardwareStamp(
            String name,
            String what,
            double[] atBoot,
            double tolerance,
            Supplier<List<double[]>> afterSave) {
        this.name = name;
        this.what = what;
        this.atBoot = atBoot == null ? null : atBoot.clone();
        this.tolerance = tolerance;
        this.afterSave = afterSave;
    }

    public String name() {
        return name;
    }

    /** The readings at boot, or null if a device did not answer. */
    public double[] atBoot() {
        return atBoot == null ? null : atBoot.clone();
    }

    /** Sets the readings from the log (replay). */
    void restore(double[] readings) {
        atBoot = readings == null ? null : readings.clone();
    }

    /** What a file saved now records. */
    ObjectNode stamp() {
        ObjectNode out = ParamStore.JSON.createObjectNode();
        out.put("what", what);
        ArrayNode accept = out.putArray("accept");
        for (double[] readings : afterSave.get()) {
            ArrayNode a = accept.addArray();
            for (double v : readings) {
                a.add(v);
            }
        }
        return out;
    }

    /**
     * Why a file with this stamp may not be used on this robot, or null if it may.
     *
     * @param stamp the file's {@link #KEY} entry, or null if it has none
     */
    String mismatch(JsonNode stamp) {
        if (atBoot == null) {
            return "cannot be checked against this robot (the "
                    + what
                    + " could not be read at boot)";
        }
        JsonNode accept = stamp == null ? null : stamp.get("accept");
        if (accept == null || !accept.isArray()) {
            return "has no hardware stamp, so it may be another robot's";
        }
        double[] first = null;
        for (JsonNode readings : accept) {
            double[] stamped = numbers(readings);
            if (stamped == null) {
                continue;
            }
            if (first == null) {
                first = stamped;
            }
            if (matches(stamped)) {
                return null;
            }
        }
        return "was saved on another robot (the "
                + what
                + " are "
                + format(atBoot)
                + ", saved with "
                + (first == null ? "none" : format(first))
                + ")";
    }

    private boolean matches(double[] stamped) {
        if (stamped.length != atBoot.length) {
            return false;
        }
        for (int i = 0; i < stamped.length; i++) {
            if (!(Math.abs(stamped[i] - atBoot[i]) <= tolerance)) {
                return false;
            }
        }
        return true;
    }

    private static double[] numbers(JsonNode node) {
        if (!node.isArray()) {
            return null;
        }
        double[] out = new double[node.size()];
        for (int i = 0; i < out.length; i++) {
            if (!node.get(i).isNumber()) {
                return null;
            }
            out[i] = node.get(i).doubleValue();
        }
        return out;
    }

    private static String format(double[] v) {
        return Arrays.stream(v)
                .mapToObj(x -> String.format(Locale.ROOT, "%.4f", x))
                .toList()
                .toString();
    }
}
