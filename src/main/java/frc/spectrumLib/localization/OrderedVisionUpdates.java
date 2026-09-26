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
 *
 * <p>Several measurements can share a timestamp, and they are all kept, in the order they were
 * added. The case that matters is seeding: {@code Vision} seeds with a MegaTag1 frame (tiny heading
 * std-dev) and the same frame, at the same capture time, can also arrive as an ordinary camera
 * measurement (heading std-dev {@code LARGE_VARIANCE}). The estimator composes the two -- its
 * {@code sampleAt} floor is inclusive, so the second builds on the first's correction and replaces
 * its entry. When this map held one measurement per timestamp, the second overwrote the seed here
 * too, and the next out-of-order redo re-applied only the camera frame: the seeded heading was
 * silently undone. Re-applying each timestamp's measurements in their original order reproduces the
 * estimator's composition exactly.
 */
final class OrderedVisionUpdates {
    /** WPILib's pose estimator buffer (PoseEstimator.kBufferDuration). */
    static final double HISTORY_SECONDS = 1.5;

    /**
     * Most timestamps kept. Pruning by odometry time keeps the map to {@link #HISTORY_SECONDS} of
     * measurements -- a few hundred timestamps with every source running -- but if odometry stops
     * (a stalled drivetrain thread) the cameras keep reporting and nothing ages out, so this bounds
     * it. Pruning by the newest measurement time instead would let one bad future timestamp empty
     * the map and keep it empty.
     */
    static final int MAX_TIMESTAMPS = 2048;

    private record Measurement(Pose2d pose, Matrix<N3, N1> stdDevs) {}

    private final PoseEstimator<?> estimator;

    /** Applied measurements by capture time; each list is in the order they were added. */
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

    /** Drops measurements the estimator can no longer use, and caps the map's size. */
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

    /** Timestamps currently kept, for tests. */
    int size() {
        return applied.size();
    }

    long reapplied() {
        return reapplied;
    }
}
