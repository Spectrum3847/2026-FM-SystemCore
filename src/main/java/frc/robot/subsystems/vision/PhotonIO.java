package frc.robot.subsystems.vision;

import frc.spectrumLib.localization.PoseObservation;
import frc.spectrumLib.localization.PoseObservation.Kind;
import frc.spectrumLib.localization.PoseSourceIO;
import frc.spectrumLib.localization.PoseSourceInputs;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.vision.apriltag.AprilTagFieldLayout;

/**
 * One PhotonVision camera on the Jetson Orin (the 971 CUDA AprilTag detector, via PhotonVision's
 * {@code jetson-orin} fork), read with PhotonLib.
 *
 * <p>Every unread pipeline result becomes an observation: the multi-tag solve when the coprocessor
 * produced one, otherwise the single best target solved against the field layout. Robot pose is the
 * field-to-camera pose composed with the inverse of {@link #robotToCamera}.
 *
 * <p>PhotonVision's own multi-tag solve must be enabled in the camera's pipeline settings, with the
 * same field layout as {@link Vision#getTagLayout()}.
 */
public class PhotonIO implements PoseSourceIO {
    private final PhotonCamera camera;
    private final Transform3d robotToCamera;
    private final AprilTagFieldLayout layout;

    /**
     * @param cameraName the camera's name in the PhotonVision UI
     * @param robotToCamera camera mount, robot frame (x forward, y left, z up)
     * @param layout the field's AprilTags
     */
    public PhotonIO(String cameraName, Transform3d robotToCamera, AprilTagFieldLayout layout) {
        this.camera = new PhotonCamera(cameraName);
        this.robotToCamera = robotToCamera;
        this.layout = layout;
    }

    /** Tag ids seen this update; reused so a loop with no targets allocates nothing. */
    private final java.util.BitSet ids = new java.util.BitSet();

    private static final int[] NO_IDS = new int[0];
    private static final double[] NO_HEALTH = new double[0];

    @Override
    public void updateInputs(PoseSourceInputs inputs) {
        inputs.connected = camera.isConnected();
        List<PoseObservation> observations = new ArrayList<>();
        ids.clear();

        for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
            if (!result.hasTargets()) {
                continue;
            }
            double distanceSum = 0;
            for (PhotonTrackedTarget t : result.getTargets()) {
                distanceSum += t.bestCameraToTarget.getTranslation().getNorm();
                if (t.fiducialId >= 0) {
                    ids.set(t.fiducialId);
                }
            }
            double avgDistance = distanceSum / result.getTargets().size();

            var multitag = result.getMultiTagResult();
            if (multitag.isPresent()) {
                Transform3d fieldToCamera = multitag.get().estimatedPose.best;
                Pose3d robot =
                        new Pose3d(fieldToCamera.getTranslation(), fieldToCamera.getRotation())
                                .transformBy(robotToCamera.inverse());
                observations.add(
                        new PoseObservation(
                                "",
                                Kind.PHOTON,
                                result.getTimestampSeconds(),
                                robot,
                                multitag.get().fiducialIDsUsed.size(),
                                avgDistance,
                                Double.NaN,
                                multitag.get().estimatedPose.ambiguity,
                                true));
            } else {
                PhotonTrackedTarget target = result.getBestTarget();
                Optional<Pose3d> tagPose = layout.getTagPose(target.fiducialId);
                if (tagPose.isEmpty()) {
                    continue;
                }
                Pose3d robot =
                        tagPose.get()
                                .transformBy(target.bestCameraToTarget.inverse())
                                .transformBy(robotToCamera.inverse());
                observations.add(
                        new PoseObservation(
                                "",
                                Kind.PHOTON,
                                result.getTimestampSeconds(),
                                robot,
                                1,
                                target.bestCameraToTarget.getTranslation().getNorm(),
                                Double.NaN,
                                target.poseAmbiguity,
                                true));
            }
        }

        inputs.resize(observations.size());
        for (int i = 0; i < observations.size(); i++) {
            inputs.set(i, observations.get(i));
        }
        inputs.tagIds = ids.isEmpty() ? NO_IDS : ids.stream().toArray();
        inputs.health = NO_HEALTH;
    }
}
