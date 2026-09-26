package frc.spectrumLib.localization;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;

/**
 * One robot-pose measurement from one source, before any gating.
 *
 * <p>This is the 2026 {@code Vision.VisionFieldPoseEstimate} (pose + timestamp + covariance)
 * promoted out of {@code Vision}, with the fields every gate in the 2026 code actually read made
 * explicit, a {@link #source} so the fused log can say where each measurement came from, and {@link
 * #tagCount} optional so a VIO source like QuestNav fits the same shape.
 *
 * <p>Every field here is derived from a logged AdvantageKit input, so a replayed log rebuilds the
 * exact same observations and a gate threshold can be changed and re-run against a real match.
 *
 * @param source source name, e.g. {@code "LL-Back/MT1"}, {@code "Orin-Left"}, {@code "Quest"}
 * @param kind what produced it, which decides which gates and std-dev model apply
 * @param timestampSeconds capture time, in the same time base as {@code Timer.getTimestamp()}
 * @param pose robot pose on the field (3-D, so tilt and height gates can see a bad solve)
 * @param tagCount AprilTags in the solve, or {@code -1} for a source with no tags
 * @param avgTagDistanceMeters mean camera-to-tag distance, or {@code NaN}
 * @param targetSizePercent Limelight target area as a percentage of the image, or {@code NaN}
 * @param maxAmbiguity largest single-tag ambiguity in the solve (0..1), or {@code NaN}
 * @param tracking VIO tracking flag; {@code true} for sources that have no such notion
 */
public record PoseObservation(
        String source,
        Kind kind,
        double timestampSeconds,
        Pose3d pose,
        int tagCount,
        double avgTagDistanceMeters,
        double targetSizePercent,
        double maxAmbiguity,
        boolean tracking) {

    /** What produced an observation. */
    public enum Kind {
        /** Limelight MegaTag1: heading and translation solved from tag geometry. */
        LIMELIGHT_MT1,
        /** Limelight MegaTag2: translation solved with the heading the robot pushed in. */
        LIMELIGHT_MT2,
        /** PhotonVision multi-tag (or lowest-ambiguity single-tag) solve. */
        PHOTON,
        /** Meta Quest inside-out tracking via QuestNav. */
        QUESTNAV
    }

    /** The pose flattened to the field plane. */
    public Pose2d pose2d() {
        return pose.toPose2d();
    }

    /** Whether the solve used more than one tag. */
    public boolean multiTag() {
        return tagCount > 1;
    }
}
