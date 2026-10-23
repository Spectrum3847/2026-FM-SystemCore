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
 * different latencies (three Limelights, the Orin cameras, the Quest), one source after another. So
 * whenever a slower camera's frame was added after a faster source's newer one, the newer
 * correction was thrown away. A Limelight frame from last loop landing after this loop's Quest
 * update did the same across loops.
 *
 * <p>This keeps the measurements it has applied for as long as the estimator can use them ({@link
 * #HISTORY_SECONDS}, WPILib's pose buffer) and, when one arrives older than the newest, re-applies
 * every newer one after it, oldest first. The estimator ends up exactly where it would have been
 * had they arrived in time order. It costs one extra estimator update per newer measurement, a
 * handful per loop at most.
 *
 * <p>Measurements that share a timestamp are all kept, in the order added, because the estimator
 * composes them. A seed and the same frame fused as a camera measurement share one capture time,
 * and keeping only the second would undo the seeded heading on the next redo.
 */
final class OrderedVisionUpdates {
    /** WPILib's pose estimator buffer (PoseEstimator.kBufferDuration). */
    static final double HISTORY_SECONDS = 1.5;

    /**
     * Most timestamps kept, since nothing ages out if odometry stalls. Pruning by the newest
     * measurement instead would let one bad future timestamp keep the map empty.
     */
    static final int MAX_TIMESTAMPS = 2048;

    private record Measurement(Pose2d pose, Matrix<N3, N1> stdDevs) {}

    private final PoseEstimator<?> estimator;

    /** Applied measurements by capture time, each list in the order added. */
    private final TreeMap<Double, List<Measurement>> applied = new TreeMap<>();

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
        prune(newestOdometrySeconds);
        estimator.addVisionMeasurement(pose, timestampSeconds, stdDevs);
        applied.computeIfAbsent(timestampSeconds, t -> new ArrayList<>(2))
                .add(new Measurement(pose, stdDevs));
        Map<Double, List<Measurement>> newer = applied.tailMap(timestampSeconds, false);
        if (newer.isEmpty()) {
            return;
        }
        // Copy first: each add below clears the estimator's newer entries, not this map's.
        List<Map.Entry<Double, List<Measurement>>> redo = new ArrayList<>(newer.entrySet());
        for (Map.Entry<Double, List<Measurement>> e : redo) {
            for (Measurement m : e.getValue()) {
                estimator.addVisionMeasurement(m.pose(), e.getKey(), m.stdDevs());
                reapplied++;
            }
        }
    }

    private void prune(double newestOdometrySeconds) {
        applied.headMap(newestOdometrySeconds - HISTORY_SECONDS, false).clear();
        while (applied.size() >= MAX_TIMESTAMPS) {
            applied.pollFirstEntry();
        }
    }

    /** Forgets every applied measurement; call when the estimator is reset. */
    void clear() {
        applied.clear();
    }

    /** For tests. */
    int size() {
        return applied.size();
    }

    long reapplied() {
        return reapplied;
    }
}
