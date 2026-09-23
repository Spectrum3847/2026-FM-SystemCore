package frc.spectrumLib.localization;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;
import org.wpilib.math.estimator.PoseEstimator;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;

/**
 * Feeds vision measurements to a WPILib pose estimator so that the result does not depend on the
 * order they arrive in.
 *
 * <p>{@link PoseEstimator#addVisionMeasurement} discards every stored measurement newer than the
 * one being added (its step 7, "matches previous behavior"). FM fuses several sources with
 * different latencies -- three Limelights, the Orin cameras, the Quest -- one source after another,
 * so whenever a slower camera's frame was added after a faster source's newer one, the newer
 * correction was thrown away. A Limelight frame from last loop landing after this loop's Quest
 * update did the same across loops.
 *
 * <p>This keeps the measurements it has applied for as long as the estimator can use them ({@link
 * #HISTORY_SECONDS}, WPILib's pose buffer) and, when one arrives older than the newest, re-applies
 * every newer one after it, oldest first. The estimator ends up exactly where it would have been
 * had they arrived in time order. It costs one extra estimator update per newer measurement, a
 * handful per loop at most.
 */
final class OrderedVisionUpdates {
    /** WPILib's pose estimator buffer (PoseEstimator.kBufferDuration). */
    static final double HISTORY_SECONDS = 1.5;

    private record Measurement(Pose2d pose, Matrix<N3, N1> stdDevs) {}

    private final PoseEstimator<?> estimator;
    private final TreeMap<Double, Measurement> applied = new TreeMap<>();

    /** Measurements re-applied because an older one arrived after them, since construction. */
    private long reapplied = 0;

    OrderedVisionUpdates(PoseEstimator<?> estimator) {
        this.estimator = estimator;
    }

    /**
     * Adds a measurement as if it had arrived in time order.
     *
     * @param pose measured robot pose
     * @param timestampSeconds capture time, the estimator's time base
     * @param stdDevs x, y, heading std-devs
     * @param newestOdometrySeconds the newest odometry sample time, for pruning
     */
    void add(
            Pose2d pose,
            double timestampSeconds,
            Matrix<N3, N1> stdDevs,
            double newestOdometrySeconds) {
        applied.headMap(newestOdometrySeconds - HISTORY_SECONDS, false).clear();
        estimator.addVisionMeasurement(pose, timestampSeconds, stdDevs);
        applied.put(timestampSeconds, new Measurement(pose, stdDevs));
        Map<Double, Measurement> newer = applied.tailMap(timestampSeconds, false);
        if (newer.isEmpty()) {
            return;
        }
        // Copy first: each add below clears the estimator's newer entries, not this map's.
        List<Map.Entry<Double, Measurement>> redo = new ArrayList<>(newer.entrySet());
        for (Map.Entry<Double, Measurement> e : redo) {
            estimator.addVisionMeasurement(e.getValue().pose(), e.getKey(), e.getValue().stdDevs());
            reapplied++;
        }
    }

    /** Forgets every applied measurement; call when the estimator is reset. */
    void clear() {
        applied.clear();
    }

    long reapplied() {
        return reapplied;
    }
}
