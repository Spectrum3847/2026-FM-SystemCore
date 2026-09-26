package frc.robot.subsystems.vision;

import frc.spectrumLib.localization.PoseObservation;
import frc.spectrumLib.localization.PoseObservation.Kind;
import frc.spectrumLib.localization.PoseSourceIO;
import frc.spectrumLib.localization.PoseSourceInputs;
import frc.spectrumLib.vision.Limelight.LimelightConfig;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.Random;
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
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.util.Units;
import org.wpilib.system.Timer;
import org.wpilib.vision.apriltag.AprilTagFieldLayout;

/**
 * Simulated cameras for every pose source, on PhotonLib's {@link VisionSystemSim}.
 *
 * <p>One PhotonLib vision system holds every AprilTag camera on the robot -- the Limelights as well
 * as the Orin's cameras -- each with its own mount and camera properties, all looking at the 2026
 * field from the simulated <em>truth</em> pose (see {@code Swerve.SimErrors}). The Orin cameras are
 * read exactly as on the robot, by {@link PhotonIO} over NetworkTables. The Limelights are read by
 * {@link LimelightSimIO}, which turns the same PhotonLib results into what a Limelight reports:
 *
 * <ul>
 *   <li><b>MegaTag1</b> -- the multi-tag (or single-tag) solve, heading included.
 *   <li><b>MegaTag2</b> -- translation solved from each tag with the heading the robot pushed,
 *       which is how MegaTag2 works and why it inherits gyro drift.
 *   <li><b>Target size</b> -- the largest target's area, as {@code ta}.
 * </ul>
 *
 * <p>The QuestNav is {@link QuestSimIO}: truth plus a slow random-walk drift, in a field frame that
 * only lines up once the robot has sent a pose reset -- the two things a VIO source gets wrong.
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
     * convention is forward, right, up, and pitch positive up; WPILib's pitch is positive down.
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
            inputs.health = new double[0];
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

    // ── QuestNav emulation ──────────────────────────────────────────────────

    /**
     * Simulated QuestNav: reports truth in its own frame, which lines up with the field only after
     * {@link #resetPose}, plus a random-walk drift. Seeded, so a sim run is repeatable.
     */
    public static final class QuestSimIO implements PoseSourceIO, QuestNavControl {
        /** Drift random walk, metres per sqrt(second). */
        private static final double DRIFT_M_PER_SQRT_S = 0.01;

        private final Supplier<Optional<Pose2d>> truth;
        private final Random random = new Random(3847);

        /** Maps truth to what the Quest reports. Starts as an arbitrary boot origin. */
        private Transform2d questFrame = new Transform2d(1.3, -0.7, Rotation2d.fromDegrees(37));

        private Translation2d drift = Translation2d.kZero;
        private double lastTime = Double.NaN;
        private int resetCount = 0;
        private double lastResetTime = Double.NaN;
        private int frameCount = 0;

        public QuestSimIO(Supplier<Optional<Pose2d>> truth) {
            this.truth = truth;
        }

        @Override
        public void resetPose(Pose2d robotPose) {
            // Choose the frame so the current truth reads as the pose the robot believes.
            truth.get()
                    .ifPresent(
                            t -> {
                                Pose2d raw = t.plus(new Transform2d(drift, Rotation2d.kZero));
                                // Frame (R, T) such that R * raw + T == robotPose.
                                Rotation2d r = robotPose.getRotation().minus(raw.getRotation());
                                Translation2d offset =
                                        robotPose
                                                .getTranslation()
                                                .minus(raw.getTranslation().rotateBy(r));
                                questFrame = new Transform2d(offset, r);
                            });
            resetCount++;
            lastResetTime = Timer.getTimestamp();
        }

        @Override
        public void updateInputs(PoseSourceInputs inputs) {
            inputs.connected = true;
            Optional<Pose2d> t = truth.get();
            double now = Timer.getTimestamp();
            if (t.isEmpty()) {
                inputs.resize(0);
                return;
            }
            if (!Double.isNaN(lastTime)) {
                double sigma = DRIFT_M_PER_SQRT_S * Math.sqrt(Math.max(now - lastTime, 0));
                drift =
                        drift.plus(
                                new Translation2d(
                                        random.nextGaussian() * sigma,
                                        random.nextGaussian() * sigma));
            }
            lastTime = now;
            Pose2d withDrift = t.get().plus(new Transform2d(drift, Rotation2d.kZero));
            // Apply the Quest frame: field pose of the reported frame's origin composed with truth.
            Pose2d reported =
                    new Pose2d(
                            withDrift
                                    .getTranslation()
                                    .rotateBy(questFrame.getRotation())
                                    .plus(questFrame.getTranslation()),
                            withDrift.getRotation().plus(questFrame.getRotation()));
            inputs.resize(1);
            inputs.set(
                    0,
                    new PoseObservation(
                            "",
                            Kind.QUESTNAV,
                            now - 0.004,
                            new Pose3d(reported),
                            -1,
                            Double.NaN,
                            Double.NaN,
                            Double.NaN,
                            true));
            inputs.tagIds = new int[0];
            inputs.health = new double[] {now, ++frameCount, 100, 0, resetCount, lastResetTime};
        }
    }
}
