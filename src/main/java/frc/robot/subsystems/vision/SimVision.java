package frc.robot.subsystems.vision;

import frc.spectrumLib.localization.PoseObservation;
import frc.spectrumLib.localization.PoseObservation.Kind;
import frc.spectrumLib.localization.PoseSourceIO;
import frc.spectrumLib.localization.PoseSourceInputs;
import frc.spectrumLib.vision.Limelight.LimelightConfig;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleFunction;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.util.Units;
import org.wpilib.vision.apriltag.AprilTagFieldLayout;

/**
 * Simulated cameras for every pose source, on PhotonLib's {@link VisionSystemSim}.
 *
 * <p>One PhotonLib vision system holds every AprilTag camera on the robot, the Limelights as well
 * as the Orin's cameras, each with its own mount and camera properties. All of them look at the
 * 2026 field from the simulated <em>truth</em> pose (see {@code Swerve.SimErrors}). The Orin
 * cameras are read exactly as on the robot, by {@link PhotonOrinIO} over NetworkTables. The
 * Limelights are read by {@link LimelightSimIO}, which turns the same PhotonLib results into what a
 * Limelight reports:
 *
 * <ul>
 *   <li><b>MegaTag1</b> uses the multi-tag (or single-tag) solve, heading included.
 *   <li><b>MegaTag2</b> solves translation from each tag with the heading the robot pushed, which
 *       is how MegaTag2 works and why it inherits gyro drift.
 *   <li><b>Target size</b> reports the largest target's area, as {@code ta}.
 * </ul>
 */
public class SimVision {
    private final VisionSystemSim system = new VisionSystemSim("fm");
    private final Supplier<Optional<Pose2d>> truth;

    /**
     * @param layout field tags
     * @param truth simulated true robot pose
     */
    public SimVision(AprilTagFieldLayout layout, Supplier<Optional<Pose2d>> truth) {
        this.truth = truth;
        system.addAprilTags(layout);
    }

    /** Moves every simulated camera to the current truth pose. Call once per loop, first. */
    public void update() {
        truth.get().ifPresent(system::update);
    }

    /** Field2d showing what PhotonLib's sim sees (tags, cameras, robot). */
    public org.wpilib.smartdashboard.Field2d getDebugField() {
        return system.getDebugField();
    }

    /** Properties of a Limelight 4 (1280x800, ~82 deg diagonal), per Limelight's spec sheet. */
    public static SimCameraProperties limelight4() {
        SimCameraProperties p = new SimCameraProperties();
        p.setCalibration(1280, 800, Rotation2d.fromDegrees(82));
        p.setCalibError(0.15, 0.05);
        p.setFPS(40);
        p.setAvgLatencyMs(25);
        p.setLatencyStdDevMs(5);
        return p;
    }

    /** Properties of a Thrifty Bot Thriftiest Cam (OV9281, 1280x800) as the Orin runs it. */
    public static SimCameraProperties thriftiestCam() {
        SimCameraProperties p = new SimCameraProperties();
        p.setCalibration(1280, 800, Rotation2d.fromDegrees(90));
        p.setCalibError(0.15, 0.05);
        p.setFPS(50);
        p.setAvgLatencyMs(20);
        p.setLatencyStdDevMs(4);
        return p;
    }

    /**
     * Adds a simulated camera that publishes as a PhotonVision camera named {@code name}.
     *
     * @param name camera name on NetworkTables
     * @param properties sensor model
     * @param robotToCamera mount
     * @return the PhotonLib camera handle, to read results from
     */
    public PhotonCamera addCamera(
            String name, SimCameraProperties properties, Transform3d robotToCamera) {
        PhotonCamera camera = new PhotonCamera(name);
        PhotonCameraSim sim = new PhotonCameraSim(camera, properties);
        sim.enableRawStream(false);
        sim.enableProcessedStream(false);
        sim.enableDrawWireframe(false);
        sim.setMaxSightRange(8.0);
        system.addCamera(sim, robotToCamera);
        return camera;
    }

    /**
     * A Limelight's mount in WPILib's robot frame (x forward, y left, z up). Limelight's own
     * convention is forward, right, up and pitch positive up; WPILib's pitch is positive down.
     */
    public static Transform3d robotToCamera(LimelightConfig c) {
        return new Transform3d(
                new Translation3d(c.getForward(), -c.getRight(), c.getUp()),
                new Rotation3d(
                        Units.degreesToRadians(c.getRoll()),
                        Units.degreesToRadians(-c.getPitch()),
                        Units.degreesToRadians(c.getYaw())));
    }

    // ── Limelight emulation ────────────────────────────────────────────────

    /** One simulated Limelight: PhotonLib results read once per loop, shared by MT1 and MT2. */
    public static final class LimelightSimCamera {
        private final PhotonCamera camera;
        private final Transform3d robotToCamera;
        private final AprilTagFieldLayout layout;
        private List<PhotonPipelineResult> results = List.of();

        public LimelightSimCamera(
                PhotonCamera camera, Transform3d robotToCamera, AprilTagFieldLayout layout) {
            this.camera = camera;
            this.robotToCamera = robotToCamera;
            this.layout = layout;
        }

        /** Reads this loop's frames. Call once per loop before either IO. */
        public void refresh() {
            results = camera.getAllUnreadResults();
        }
    }

    /**
     * A Limelight source backed by a PhotonLib sim camera.
     *
     * <p>{@code pushedHeading} is the heading the robot would have pushed to the camera at a given
     * capture time (the fused heading), used for MegaTag2.
     */
    public static final class LimelightSimIO implements PoseSourceIO {
        private final LimelightSimCamera cam;
        private final Kind kind;
        private final DoubleFunction<Rotation2d> pushedHeading;

        public LimelightSimIO(
                LimelightSimCamera cam, Kind kind, DoubleFunction<Rotation2d> pushedHeading) {
            this.cam = cam;
            this.kind = kind;
            this.pushedHeading = pushedHeading;
        }

        @Override
        public void updateInputs(PoseSourceInputs inputs) {
            inputs.connected = true;
            List<PoseObservation> out = new ArrayList<>();
            List<Integer> ids = new ArrayList<>();
            double[] fiducials = new double[0];
            for (PhotonPipelineResult r : cam.results) {
                if (!r.hasTargets()) {
                    continue;
                }
                double t = r.getTimestampSeconds();
                double maxArea = 0;
                double maxAmbiguity = 0;
                double distSum = 0;
                for (PhotonTrackedTarget target : r.getTargets()) {
                    maxArea = Math.max(maxArea, target.area);
                    maxAmbiguity = Math.max(maxAmbiguity, target.poseAmbiguity);
                    distSum += target.bestCameraToTarget.getTranslation().getNorm();
                    ids.add(target.fiducialId);
                }
                int tagCount = r.getTargets().size();
                Pose3d mt1 = megaTag1(r);
                if (mt1 == null) {
                    continue;
                }
                Pose3d pose = kind == Kind.LIMELIGHT_MT1 ? mt1 : megaTag2(r, t, mt1);
                if (pose == null) {
                    continue;
                }
                if (kind == Kind.LIMELIGHT_MT1) {
                    fiducials = fiducials(r);
                }
                out.add(
                        new PoseObservation(
                                "",
                                kind,
                                t,
                                pose,
                                tagCount,
                                distSum / tagCount,
                                maxArea,
                                maxAmbiguity,
                                true));
            }
            inputs.resize(out.size());
            for (int i = 0; i < out.size(); i++) {
                inputs.set(i, out.get(i));
            }
            inputs.tagIds = ids.stream().mapToInt(Integer::intValue).distinct().toArray();
            inputs.fiducials = fiducials;
            inputs.health = new double[0];
        }

        /** The frame's tags as a Limelight's raw fiducials (the newest frame's, as on a robot). */
        private static double[] fiducials(PhotonPipelineResult r) {
            int stride = PoseSourceInputs.FIDUCIAL_STRIDE;
            List<PhotonTrackedTarget> targets = r.getTargets();
            double[] out = new double[targets.size() * stride];
            for (int i = 0; i < targets.size(); i++) {
                PhotonTrackedTarget target = targets.get(i);
                Translation3d v = target.bestCameraToTarget.getTranslation();
                out[i * stride] = target.fiducialId;
                out[i * stride + 1] = Math.toDegrees(Math.atan2(-v.getY(), v.getX()));
                out[i * stride + 2] = Math.toDegrees(Math.atan2(v.getZ(), v.getX()));
                out[i * stride + 3] = v.getNorm();
                out[i * stride + 4] = target.poseAmbiguity;
            }
            return out;
        }

        private Pose3d megaTag1(PhotonPipelineResult r) {
            var multi = r.getMultiTagResult();
            if (multi.isPresent()) {
                Transform3d fieldToCamera = multi.get().estimatedPose.best;
                return new Pose3d(fieldToCamera.getTranslation(), fieldToCamera.getRotation())
                        .transformBy(cam.robotToCamera.inverse());
            }
            PhotonTrackedTarget target = r.getBestTarget();
            return cam.layout
                    .getTagPose(target.fiducialId)
                    .map(
                            tag ->
                                    tag.transformBy(target.bestCameraToTarget.inverse())
                                            .transformBy(cam.robotToCamera.inverse()))
                    .orElse(null);
        }

        /**
         * MegaTag2: with the robot's yaw given, each tag's camera-frame translation fixes the robot
         * translation on its own; average over tags. Roll, pitch and height come from MT1, as the
         * real IO does.
         */
        private Pose3d megaTag2(PhotonPipelineResult r, double t, Pose3d mt1) {
            Rotation2d yaw = pushedHeading.apply(t);
            Rotation3d robotRotation = new Rotation3d(0, 0, yaw.getRadians());
            Rotation3d cameraRotation = cam.robotToCamera.getRotation().rotateBy(robotRotation);
            double sx = 0;
            double sy = 0;
            int n = 0;
            for (PhotonTrackedTarget target : r.getTargets()) {
                var tag = cam.layout.getTagPose(target.fiducialId);
                if (tag.isEmpty()) {
                    continue;
                }
                Translation3d camToTagField =
                        target.bestCameraToTarget.getTranslation().rotateBy(cameraRotation);
                Translation3d cameraField = tag.get().getTranslation().minus(camToTagField);
                Translation3d robotField =
                        cameraField.minus(
                                cam.robotToCamera.getTranslation().rotateBy(robotRotation));
                sx += robotField.getX();
                sy += robotField.getY();
                n++;
            }
            if (n == 0) {
                return null;
            }
            return new Pose3d(
                    sx / n,
                    sy / n,
                    mt1.getZ(),
                    new Rotation3d(
                            mt1.getRotation().getX(), mt1.getRotation().getY(), yaw.getRadians()));
        }
    }
}
