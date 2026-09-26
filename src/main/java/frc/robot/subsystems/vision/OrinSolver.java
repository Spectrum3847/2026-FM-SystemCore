package frc.robot.subsystems.vision;

import frc.spectrumLib.localization.PoseObservation;
import frc.spectrumLib.localization.PoseObservation.Kind;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.DoubleFunction;
import org.photonvision.common.dataflow.structures.Packet;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.vision.apriltag.AprilTagFieldLayout;

/**
 * Turns one Orin camera's logged PhotonLib results into pose observations. Reads only {@link
 * OrinCameraInputs} and robot state that is itself replayed, so it runs identically live and in
 * replay (issue #10, section 2).
 *
 * <ul>
 *   <li><b>Multi-tag:</b> the Jetson's multi-tag solve ({@code estimatedPose.best}). Tag count and
 *       distance come from {@code fiducialIDsUsed}, not every visible target (section 10a).
 *   <li><b>Single tag:</b> the lowest-ambiguity usable tag. Of its two PnP candidates, the one
 *       whose heading is closest to the gyro's at the frame's timestamp (6328, section 8b); the
 *       ambiguity gate in {@link VisionGates#orinGates} then drops frames where the two are too
 *       alike.
 *   <li>Targets with {@code fiducialId < 0} (game pieces), tags missing from the layout and tags on
 *       the excluded list (section 12) are never used. The Jetson already leaves excluded tags out
 *       of multi-tag; this keeps them out of the single-tag path too.
 *   <li>Each observation's std-dev scale is the camera's factor times {@link #edgeScale}: tags near
 *       the image edge are trusted less, not dropped (section 8b).
 * </ul>
 */
public final class OrinSolver {

    /** Std-dev falloff starts this far (px) from the image edge. */
    public static final double EDGE_BAND_PX = 100.0;

    /** Std-dev multiplier at the very edge; 1.0 turns the edge scaling off. */
    public static final double EDGE_MAX_SCALE = 4.0;

    private final Transform3d cameraToRobot;
    private final AprilTagFieldLayout layout;
    private final double cameraFactor;
    private final int imageWidth;
    private final int imageHeight;

    /** Stats from the most recent {@link #solve} call, for logging. */
    public int decodeFailures;

    public int excludedTargets;

    /** Every fiducial id seen in the last {@link #solve} call's results. */
    public final java.util.BitSet seenTags = new java.util.BitSet();

    public double lastLatencyMs = Double.NaN;
    public double lastTimestampSeconds = Double.NaN;

    /**
     * @param robotToCamera camera mount, robot frame (x forward, y left, z up)
     * @param layout the field's AprilTags; must match the Jetson's
     * @param cameraFactor per-camera std-dev multiplier (6328's camera factor)
     * @param imageWidth image width in px (the Jetson's images are 1280x800)
     * @param imageHeight image height in px
     */
    public OrinSolver(
            Transform3d robotToCamera,
            AprilTagFieldLayout layout,
            double cameraFactor,
            int imageWidth,
            int imageHeight) {
        this.cameraToRobot = robotToCamera.inverse();
        this.layout = layout;
        this.cameraFactor = cameraFactor;
        this.imageWidth = imageWidth;
        this.imageHeight = imageHeight;
    }

    /** Decodes one logged result, or empty if the bytes are bad (PhotonVision #2528). */
    public Optional<PhotonPipelineResult> decode(byte[] bytes) {
        try {
            // A new Packet each time: setData() does not reset the read position.
            return Optional.of(PhotonPipelineResult.photonStruct.unpack(new Packet(bytes)));
        } catch (RuntimeException e) {
            decodeFailures++;
            return Optional.empty();
        }
    }

    /**
     * Solves every logged result.
     *
     * @param inputs this loop's camera inputs
     * @param excludedTags tag ids to leave out (section 12)
     * @param headingAt the robot's heading at a capture time, or {@code null} if unknown (before
     *     the pose is seeded); picks between single-tag candidates
     * @return one observation per result that had a usable tag
     */
    public List<PoseObservation> solve(
            OrinCameraInputs inputs,
            Set<Integer> excludedTags,
            DoubleFunction<Rotation2d> headingAt) {
        decodeFailures = 0;
        excludedTargets = 0;
        seenTags.clear();
        List<PoseObservation> out = new ArrayList<>();
        for (byte[] bytes : inputs.results) {
            Optional<PhotonPipelineResult> decoded = decode(bytes);
            if (decoded.isEmpty()) {
                continue;
            }
            PhotonPipelineResult result = decoded.get();
            for (PhotonTrackedTarget t : result.getTargets()) {
                if (t.fiducialId >= 0) {
                    seenTags.set(t.fiducialId);
                }
            }
            lastLatencyMs = result.metadata.getLatencyMillis();
            lastTimestampSeconds = result.getTimestampSeconds();
            PoseObservation o = solveOne(result, excludedTags, headingAt);
            if (o != null) {
                out.add(o);
            }
        }
        return out;
    }

    private PoseObservation solveOne(
            PhotonPipelineResult result,
            Set<Integer> excludedTags,
            DoubleFunction<Rotation2d> headingAt) {
        double timestamp = result.getTimestampSeconds();
        Optional<MultiTargetPNPResult> multitag = result.getMultiTagResult();
        if (multitag.isPresent() && multitag.get().fiducialIDsUsed.size() > 1) {
            List<Short> used = multitag.get().fiducialIDsUsed;
            double distanceSum = 0;
            double edgeSum = 0;
            int counted = 0;
            for (PhotonTrackedTarget t : result.getTargets()) {
                if (t.fiducialId >= 0 && used.contains((short) t.fiducialId)) {
                    distanceSum += t.bestCameraToTarget.getTranslation().getNorm();
                    edgeSum += edgeScale(t);
                    counted++;
                }
            }
            Transform3d fieldToCamera = multitag.get().estimatedPose.best;
            Pose3d robot =
                    new Pose3d(fieldToCamera.getTranslation(), fieldToCamera.getRotation())
                            .transformBy(cameraToRobot);
            return new PoseObservation(
                    "",
                    Kind.PHOTON,
                    timestamp,
                    robot,
                    used.size(),
                    counted > 0 ? distanceSum / counted : Double.NaN,
                    Double.NaN,
                    Double.NaN,
                    true,
                    cameraFactor * (counted > 0 ? edgeSum / counted : 1.0));
        }

        // Single tag: the least ambiguous usable one.
        PhotonTrackedTarget best = null;
        Pose3d bestTagPose = null;
        for (PhotonTrackedTarget t : result.getTargets()) {
            if (t.fiducialId < 0) {
                continue;
            }
            if (excludedTags.contains(t.fiducialId)) {
                excludedTargets++;
                continue;
            }
            Optional<Pose3d> tagPose = layout.getTagPose(t.fiducialId);
            if (tagPose.isEmpty()) {
                continue;
            }
            if (best == null || ambiguity(t) < ambiguity(best)) {
                best = t;
                bestTagPose = tagPose.get();
            }
        }
        if (best == null) {
            return null;
        }
        Pose3d fromBest = robotFromTag(bestTagPose, best.bestCameraToTarget);
        Pose3d chosen = fromBest;
        Transform3d chosenCameraToTag = best.bestCameraToTarget;
        Rotation2d heading = headingAt == null ? null : headingAt.apply(timestamp);
        if (heading != null && best.altCameraToTarget != null) {
            Pose3d fromAlt = robotFromTag(bestTagPose, best.altCameraToTarget);
            double bestErr =
                    Math.abs(fromBest.toPose2d().getRotation().minus(heading).getRadians());
            double altErr = Math.abs(fromAlt.toPose2d().getRotation().minus(heading).getRadians());
            if (altErr < bestErr) {
                chosen = fromAlt;
                chosenCameraToTag = best.altCameraToTarget;
            }
        }
        return new PoseObservation(
                "",
                Kind.PHOTON,
                timestamp,
                chosen,
                1,
                chosenCameraToTag.getTranslation().getNorm(),
                Double.NaN,
                best.poseAmbiguity >= 0 ? best.poseAmbiguity : Double.NaN,
                true,
                cameraFactor * edgeScale(best));
    }

    private Pose3d robotFromTag(Pose3d tagPose, Transform3d cameraToTag) {
        return tagPose.transformBy(cameraToTag.inverse()).transformBy(cameraToRobot);
    }

    /** Ambiguity for ranking; unknown (negative) sorts last. */
    private static double ambiguity(PhotonTrackedTarget t) {
        return t.poseAmbiguity >= 0 ? t.poseAmbiguity : Double.MAX_VALUE;
    }

    /**
     * 1.0 away from the image edges, rising smoothly to {@link #EDGE_MAX_SCALE} as the tag's
     * closest corner reaches an edge. Near the edge lens distortion is strongest, the calibration
     * has the least data, and tags are often cut off.
     */
    double edgeScale(PhotonTrackedTarget t) {
        List<TargetCorner> corners = t.getDetectedCorners();
        if (corners == null || corners.isEmpty()) {
            return 1.0;
        }
        double e = Double.MAX_VALUE; // closest corner's distance to any image edge, px
        for (TargetCorner c : corners) {
            e =
                    Math.min(
                            e,
                            Math.min(
                                    Math.min(c.x, imageWidth - c.x),
                                    Math.min(c.y, imageHeight - c.y)));
        }
        double u = Math.max(0.0, 1.0 - e / EDGE_BAND_PX); // 0 inside the band, 1 at the edge
        u = Math.min(u, 1.0);
        return 1.0 + (EDGE_MAX_SCALE - 1.0) * u * u;
    }
}
