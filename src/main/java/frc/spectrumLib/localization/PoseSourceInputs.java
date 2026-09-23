package frc.spectrumLib.localization;

import frc.spectrumLib.localization.PoseObservation.Kind;
import org.littletonrobotics.junction.LogTable;
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
 * <p>Logged under {@code /Localization/Sources/<name>/...}.
 */
public class PoseSourceInputs implements LoggableInputs {
    /** Whether the device is talking at all (NT heartbeat, frames arriving). */
    public boolean connected = false;

    public double[] timestamps = new double[0];
    public Pose3d[] poses = new Pose3d[0];

    /** {@link Kind} ordinal per observation. */
    public int[] kinds = new int[0];

    public int[] tagCounts = new int[0];
    public double[] avgTagDistances = new double[0];
    public double[] targetSizes = new double[0];
    public double[] maxAmbiguities = new double[0];
    public boolean[] tracking = new boolean[0];

    /** Every tag id seen this loop, for display. */
    public int[] tagIds = new int[0];

    /** Source-specific health numbers, e.g. QuestNav battery, Limelight fps. Keys fixed per IO. */
    public double[] health = new double[0];

    /** Sets the observation arrays to hold {@code n} entries. */
    public void resize(int n) {
        timestamps = new double[n];
        poses = new Pose3d[n];
        kinds = new int[n];
        tagCounts = new int[n];
        avgTagDistances = new double[n];
        targetSizes = new double[n];
        maxAmbiguities = new double[n];
        tracking = new boolean[n];
    }

    /** Writes observation {@code i}. */
    public void set(int i, PoseObservation o) {
        timestamps[i] = o.timestampSeconds();
        poses[i] = o.pose();
        kinds[i] = o.kind().ordinal();
        tagCounts[i] = o.tagCount();
        avgTagDistances[i] = o.avgTagDistanceMeters();
        targetSizes[i] = o.targetSizePercent();
        maxAmbiguities[i] = o.maxAmbiguity();
        tracking[i] = o.tracking();
    }

    /** Number of observations. */
    public int count() {
        return timestamps.length;
    }

    /** Rebuilds observation {@code i}, e.g. from replayed inputs. */
    public PoseObservation get(String source, int i) {
        return new PoseObservation(
                source,
                Kind.values()[kinds[i]],
                timestamps[i],
                poses[i],
                tagCounts[i],
                avgTagDistances[i],
                targetSizes[i],
                maxAmbiguities[i],
                tracking[i]);
    }

    @Override
    public void toLog(LogTable table) {
        table.put("Connected", connected);
        table.put("Timestamps", timestamps);
        table.put("Poses", poses);
        table.put("Kinds", kinds);
        table.put("TagCounts", tagCounts);
        table.put("AvgTagDistances", avgTagDistances);
        table.put("TargetSizes", targetSizes);
        table.put("MaxAmbiguities", maxAmbiguities);
        table.put("Tracking", tracking);
        table.put("TagIds", tagIds);
        table.put("Health", health);
    }

    @Override
    public void fromLog(LogTable table) {
        connected = table.get("Connected", connected);
        timestamps = table.get("Timestamps", timestamps);
        poses = table.get("Poses", poses);
        kinds = table.get("Kinds", kinds);
        tagCounts = table.get("TagCounts", tagCounts);
        avgTagDistances = table.get("AvgTagDistances", avgTagDistances);
        targetSizes = table.get("TargetSizes", targetSizes);
        maxAmbiguities = table.get("MaxAmbiguities", maxAmbiguities);
        tracking = table.get("Tracking", tracking);
        tagIds = table.get("TagIds", tagIds);
        health = table.get("Health", health);
    }
}
