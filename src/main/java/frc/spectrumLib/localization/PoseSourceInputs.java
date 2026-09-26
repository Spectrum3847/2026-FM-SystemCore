package frc.spectrumLib.localization;

import frc.spectrumLib.localization.PoseObservation.Kind;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.wpilib.math.geometry.Pose3d;

/**
 * Everything a pose source reported since the last loop, as an AdvantageKit input.
 *
 * <p>This is logged <em>before</em> any gate runs. It is the raw record that answers "would this
 * source have been right?", and in replay it is what the gates and the fusion re-run against. The
 * observations are stored as parallel arrays because AdvantageKit logs arrays of primitives and
 * structs natively.
 *
 * <p>Size matters here: these inputs were a third of the log. AdvantageKit writes a value only when
 * it changes, and the observation arrays change every frame (a frame is reported once, then the
 * arrays go back to empty). So values that are the same for every observation of a source are not
 * arrays: {@code Kind} is one number per source, written once, and {@code Tracking} is written only
 * for a loop in which some observation was not tracking (AprilTag solves always are; the Quest
 * sometimes is not). Logs written before this change still replay: their per-observation {@code
 * Kinds} and full {@code Tracking} arrays are read when present.
 *
 * <p>Logged under {@code /Localization/Sources/<name>/...}.
 */
public class PoseSourceInputs implements LoggableInputs {
    /** Whether the device is talking at all (NT heartbeat, frames arriving). */
    public boolean connected = false;

    private static final double[] EMPTY_DOUBLE = new double[0];
    private static final int[] EMPTY_INT = new int[0];
    private static final boolean[] EMPTY_BOOLEAN = new boolean[0];
    private static final Pose3d[] EMPTY_POSE = new Pose3d[0];

    public double[] timestamps = EMPTY_DOUBLE;
    public Pose3d[] poses = EMPTY_POSE;

    /** {@link Kind} ordinal of this source's observations; -1 before the first. */
    public int kind = -1;

    /** Per-observation kinds, only from logs written before {@link #kind}; else empty. */
    private int[] legacyKinds = EMPTY_INT;

    public int[] tagCounts = EMPTY_INT;
    public double[] avgTagDistances = EMPTY_DOUBLE;
    public double[] targetSizes = EMPTY_DOUBLE;
    public double[] maxAmbiguities = EMPTY_DOUBLE;
    public boolean[] tracking = EMPTY_BOOLEAN;

    /** Per-observation {@link PoseObservation#stdDevScale}; logged only when some entry isn't 1. */
    public double[] stdDevScales = EMPTY_DOUBLE;

    /** Every tag id seen this loop, for display. */
    public int[] tagIds = new int[0];

    /** Source-specific health numbers, e.g. QuestNav battery, Limelight fps. Keys fixed per IO. */
    public double[] health = new double[0];

    /** Sets the observation arrays to hold {@code n} entries. */
    public void resize(int n) {
        legacyKinds = EMPTY_INT;
        if (n == 0) {
            // Most loops: no new frame. Shared empty arrays, nothing allocated (they cannot be
            // written into, so sharing them keeps the never-mutate-an-input rule).
            timestamps = EMPTY_DOUBLE;
            poses = EMPTY_POSE;
            tagCounts = EMPTY_INT;
            avgTagDistances = EMPTY_DOUBLE;
            targetSizes = EMPTY_DOUBLE;
            maxAmbiguities = EMPTY_DOUBLE;
            tracking = EMPTY_BOOLEAN;
            stdDevScales = EMPTY_DOUBLE;
            return;
        }
        timestamps = new double[n];
        poses = new Pose3d[n];
        tagCounts = new int[n];
        avgTagDistances = new double[n];
        targetSizes = new double[n];
        maxAmbiguities = new double[n];
        tracking = new boolean[n];
        stdDevScales = new double[n];
    }

    /** Writes observation {@code i}. */
    public void set(int i, PoseObservation o) {
        timestamps[i] = o.timestampSeconds();
        poses[i] = o.pose();
        kind = o.kind().ordinal();
        tagCounts[i] = o.tagCount();
        avgTagDistances[i] = o.avgTagDistanceMeters();
        targetSizes[i] = o.targetSizePercent();
        maxAmbiguities[i] = o.maxAmbiguity();
        tracking[i] = o.tracking();
        stdDevScales[i] = o.stdDevScale();
    }

    /** Number of observations. */
    public int count() {
        return timestamps.length;
    }

    /** Rebuilds observation {@code i}, e.g. from replayed inputs. */
    public PoseObservation get(String source, int i) {
        return new PoseObservation(
                source,
                Kind.values()[legacyKinds.length == count() ? legacyKinds[i] : kind],
                timestamps[i],
                poses[i],
                tagCounts[i],
                avgTagDistances[i],
                targetSizes[i],
                maxAmbiguities[i],
                tracking[i],
                stdDevScales[i]);
    }

    @Override
    public void toLog(LogTable table) {
        table.put("Connected", connected);
        table.put("Timestamps", timestamps);
        table.put("Poses", poses);
        table.put("Kind", kind);
        table.put("TagCounts", tagCounts);
        table.put("AvgTagDistances", avgTagDistances);
        table.put("TargetSizes", targetSizes);
        table.put("MaxAmbiguities", maxAmbiguities);
        boolean allTracking = true;
        for (boolean t : tracking) {
            allTracking &= t;
        }
        table.put("Tracking", allTracking ? EMPTY_BOOLEAN : tracking);
        table.put("StdDevScales", allOnes(stdDevScales) ? EMPTY_DOUBLE : stdDevScales);
        table.put("TagIds", tagIds);
        table.put("Health", health);
    }

    @Override
    public void fromLog(LogTable table) {
        connected = table.get("Connected", connected);
        timestamps = table.get("Timestamps", timestamps);
        poses = table.get("Poses", poses);
        kind = table.get("Kind", kind);
        legacyKinds = table.get("Kinds", EMPTY_INT);
        tagCounts = table.get("TagCounts", tagCounts);
        avgTagDistances = table.get("AvgTagDistances", avgTagDistances);
        targetSizes = table.get("TargetSizes", targetSizes);
        maxAmbiguities = table.get("MaxAmbiguities", maxAmbiguities);
        tracking = table.get("Tracking", EMPTY_BOOLEAN);
        if (tracking.length != timestamps.length) {
            // Written empty when every observation was tracking.
            tracking = new boolean[timestamps.length];
            java.util.Arrays.fill(tracking, true);
        }
        stdDevScales = table.get("StdDevScales", EMPTY_DOUBLE);
        if (stdDevScales.length != timestamps.length) {
            // Written empty when every scale was 1, and absent from older logs.
            stdDevScales = new double[timestamps.length];
            java.util.Arrays.fill(stdDevScales, 1.0);
        }
        tagIds = table.get("TagIds", tagIds);
        health = table.get("Health", health);
    }

    private static boolean allOnes(double[] values) {
        for (double v : values) {
            if (v != 1.0) {
                return false;
            }
        }
        return true;
    }

    /**
     * Records these observations as AdvantageKit <em>outputs</em> under {@code prefix}, for a
     * source whose observations are computed from other logged inputs (see {@link
     * PoseSourceIO#derived()}). In replay they are recomputed, not read back.
     */
    public void recordAsOutputs(String prefix) {
        Logger.recordOutput(prefix + "/Connected", connected);
        Logger.recordOutput(prefix + "/Timestamps", timestamps);
        Logger.recordOutput(prefix + "/Poses", poses);
        Logger.recordOutput(prefix + "/TagCounts", tagCounts);
        Logger.recordOutput(prefix + "/AvgTagDistances", avgTagDistances);
        Logger.recordOutput(prefix + "/MaxAmbiguities", maxAmbiguities);
        Logger.recordOutput(prefix + "/StdDevScales", stdDevScales);
        Logger.recordOutput(prefix + "/TagIds", tagIds);
    }
}
