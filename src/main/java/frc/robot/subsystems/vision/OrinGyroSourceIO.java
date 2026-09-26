package frc.robot.subsystems.vision;

import frc.spectrumLib.localization.PoseObservation;
import frc.spectrumLib.localization.PoseObservation.Kind;
import frc.spectrumLib.localization.PoseSourceIO;
import frc.spectrumLib.localization.PoseSourceInputs;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.DoubleFunction;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;
import org.wpilib.math.numbers.N8;
import org.wpilib.vision.apriltag.AprilTagFieldLayout;

/**
 * An Orin camera pose source that uses the robot's heading instead of solving for it. Built on the
 * camera's {@link OrinPoseSourceIO}: it reads the results that source already logged and decoded,
 * so it logs nothing new and runs in replay too. Logged and shadowed, not fused, until replay says
 * it can be trusted. Only the newest usable result per loop is solved (CPU).
 *
 * <ul>
 *   <li>{@link Mode#GYRO_PNP} (issue #10, section 3): PhotonLib's constrained SolvePnP over every
 *       usable tag with the heading held to the robot's at the frame's timestamp, seeded from the
 *       fused pose; one tag falls back to PhotonLib's distance-trig solve. The robot-side answer to
 *       the Limelight's MegaTag2, without its "last pushed heading" latency.
 *   <li>{@link Mode#TXTY} (section 8c, 6328's {@code addTxTyObservation}): the closest tag's angles
 *       and solved distance, turned into a robot position with the robot's heading. The distance is
 *       reliable even when a single-tag pose is ambiguous; 6328 uses this for final alignment.
 * </ul>
 *
 * <p>Both need the heading, so they report nothing until the pose is seeded.
 */
public class OrinGyroSourceIO implements PoseSourceIO {

    /** Which estimate this source makes. */
    public enum Mode {
        GYRO_PNP,
        TXTY
    }

    private final Mode mode;
    private final OrinPoseSourceIO camera;
    private final String logPrefix;
    private final Transform3d robotToCamera;
    private final AprilTagFieldLayout layout;
    private final Supplier<Set<Integer>> excludedTags;
    private final DoubleFunction<Pose2d> fusedPoseAt;
    private final PhotonPoseEstimator estimator;
    private double lastSolvedTimestamp = Double.NaN;

    /**
     * Whether PhotonLib's native solver loaded. It is otherwise loaded only as a side effect of
     * constructing a {@code PhotonCamera}, which replay never does. Without it the constrained
     * solve is skipped and every frame uses the distance-trig solve.
     */
    private static final boolean NATIVE_SOLVER = loadNativeSolver();

    private static boolean loadNativeSolver() {
        try {
            return org.photonvision.jni.LibraryLoader.loadTargeting();
        } catch (Throwable t) {
            return false;
        }
    }

    private boolean nativeSolverFailed = !NATIVE_SOLVER;

    /** A constrained solve further than this from its seed has diverged. */
    private static final double MAX_CONSTRAINED_JUMP_METERS = 1.0;

    private long divergedSolves = 0;

    /**
     * @param mode which estimate
     * @param cameraName PhotonVision camera name, for the log path
     * @param camera the camera's main source, which must update first each loop
     * @param robotToCamera camera mount
     * @param layout the field's tags
     * @param excludedTags tags to leave out
     * @param fusedPoseAt the fused pose at a capture time, or null before the pose is seeded
     */
    public OrinGyroSourceIO(
            Mode mode,
            String cameraName,
            OrinPoseSourceIO camera,
            Transform3d robotToCamera,
            AprilTagFieldLayout layout,
            Supplier<Set<Integer>> excludedTags,
            DoubleFunction<Pose2d> fusedPoseAt) {
        this.mode = mode;
        this.camera = camera;
        this.logPrefix = "Vision/Orin/" + cameraName + "/" + mode;
        this.robotToCamera = robotToCamera;
        this.layout = layout;
        this.excludedTags = excludedTags;
        this.fusedPoseAt = fusedPoseAt;
        this.estimator = new PhotonPoseEstimator(layout, robotToCamera);
    }

    @Override
    public boolean derived() {
        return true;
    }

    @Override
    public void updateInputs(PoseSourceInputs inputs) {
        inputs.connected = camera.getCameraInputs().connected;
        inputs.kind = (mode == Mode.GYRO_PNP ? Kind.PHOTON_GYRO : Kind.PHOTON_TXTY).ordinal();
        PoseObservation o = null;
        long start = System.nanoTime();
        try {
            o = solveNewest();
        } catch (RuntimeException e) {
            // The constrained solve is native code; a failure drops this frame, not the loop.
            Logger.recordOutput(logPrefix + "/LastError", e.toString());
        } catch (LinkageError e) {
            // Native library missing: fall back to the distance-trig solve from now on.
            nativeSolverFailed = true;
            Logger.recordOutput(logPrefix + "/LastError", e.toString());
        }
        Logger.recordOutput(logPrefix + "/NativeSolver", !nativeSolverFailed);
        if (o != null) {
            Logger.recordOutput(logPrefix + "/SolveMicros", (System.nanoTime() - start) / 1000);
            inputs.resize(1);
            inputs.set(0, o);
        } else {
            inputs.resize(0);
        }
    }

    /** The newest result with a usable tag, solved; null if none or no heading yet. */
    private PoseObservation solveNewest() {
        List<PhotonPipelineResult> results = camera.getSolver().decoded;
        Set<Integer> excluded = excludedTags.get();
        for (int i = results.size() - 1; i >= 0; i--) {
            PhotonPipelineResult r = results.get(i);
            List<PhotonTrackedTarget> usable = new ArrayList<>();
            for (PhotonTrackedTarget t : r.getTargets()) {
                if (t.fiducialId >= 0
                        && !excluded.contains(t.fiducialId)
                        && layout.getTagPose(t.fiducialId).isPresent()) {
                    usable.add(t);
                }
            }
            if (usable.isEmpty()) {
                continue;
            }
            double ts = r.getTimestampSeconds();
            if (ts == lastSolvedTimestamp) {
                return null;
            }
            Pose2d fused = fusedPoseAt.apply(ts);
            if (fused == null) {
                return null;
            }
            lastSolvedTimestamp = ts;
            return mode == Mode.GYRO_PNP
                    ? solveGyroPnp(r, usable, fused)
                    : solveTxTy(ts, usable, fused.getRotation());
        }
        return null;
    }

    private PoseObservation solveGyroPnp(
            PhotonPipelineResult r, List<PhotonTrackedTarget> usable, Pose2d fused) {
        double ts = r.getTimestampSeconds();
        // One heading sample at the frame's own timestamp: the solve looks it up by timestamp.
        estimator.resetHeadingData(ts, fused.getRotation());
        PhotonPipelineResult filtered =
                new PhotonPipelineResult(r.metadata, usable, Optional.empty());
        Optional<EstimatedRobotPose> est;
        Matrix<N3, N3> k = cameraMatrix();
        Matrix<N8, N1> d = distCoeffs();
        if (usable.size() >= 2 && k != null && d != null && !nativeSolverFailed) {
            est =
                    estimator.estimateConstrainedSolvepnpPose(
                            filtered, k, d, new Pose3d(fused), false, 1.0);
            // It is a local optimizer: from a poor seed (just after the pose is seeded, say) it can
            // land anywhere, once 8 km away in sim. Past a metre from the seed, use the trig solve.
            if (est.isPresent()
                    && est.get()
                                    .estimatedPose
                                    .toPose2d()
                                    .getTranslation()
                                    .getDistance(fused.getTranslation())
                            > MAX_CONSTRAINED_JUMP_METERS) {
                divergedSolves++;
                Logger.recordOutput(logPrefix + "/DivergedSolves", divergedSolves);
                est = estimator.estimatePnpDistanceTrigSolvePose(filtered);
            }
        } else {
            est = estimator.estimatePnpDistanceTrigSolvePose(filtered);
        }
        if (est.isEmpty()) {
            return null;
        }
        return observation(ts, est.get().estimatedPose, usable);
    }

    private PoseObservation solveTxTy(
            double ts, List<PhotonTrackedTarget> usable, Rotation2d heading) {
        PhotonTrackedTarget closest = null;
        for (PhotonTrackedTarget t : usable) {
            if (closest == null
                    || t.bestCameraToTarget.getTranslation().getNorm()
                            < closest.bestCameraToTarget.getTranslation().getNorm()) {
                closest = t;
            }
        }
        double distance = closest.bestCameraToTarget.getTranslation().getNorm();
        // PhotonVision's yaw is positive to the right; the camera frame's y is positive left.
        double yaw = -Math.toRadians(closest.yaw);
        double pitch = Math.toRadians(closest.pitch);
        Translation3d cameraToTag =
                new Translation3d(
                        distance * Math.cos(pitch) * Math.cos(yaw),
                        distance * Math.cos(pitch) * Math.sin(yaw),
                        distance * Math.sin(pitch));
        Translation3d robotToTag =
                robotToCamera
                        .getTranslation()
                        .plus(cameraToTag.rotateBy(robotToCamera.getRotation()));
        Translation2d tagOnField =
                layout.getTagPose(closest.fiducialId).get().toPose2d().getTranslation();
        Translation2d robot = tagOnField.minus(robotToTag.toTranslation2d().rotateBy(heading));
        Pose3d pose =
                new Pose3d(
                        new Translation3d(robot.getX(), robot.getY(), 0),
                        new Rotation3d(0, 0, heading.getRadians()));
        return observation(ts, pose, List.of(closest));
    }

    private PoseObservation observation(double ts, Pose3d pose, List<PhotonTrackedTarget> used) {
        double distanceSum = 0;
        double edgeSum = 0;
        for (PhotonTrackedTarget t : used) {
            distanceSum += t.bestCameraToTarget.getTranslation().getNorm();
            edgeSum += camera.getSolver().edgeScale(t);
        }
        return new PoseObservation(
                "",
                mode == Mode.GYRO_PNP ? Kind.PHOTON_GYRO : Kind.PHOTON_TXTY,
                ts,
                pose,
                used.size(),
                distanceSum / used.size(),
                Double.NaN,
                Double.NaN,
                true,
                camera.getSolver().cameraFactor() * edgeSum / used.size());
    }

    private Matrix<N3, N3> cameraMatrix() {
        double[] m = camera.getCameraInputs().cameraMatrix;
        return m.length == 9
                ? new Matrix<>(org.wpilib.math.util.Nat.N3(), org.wpilib.math.util.Nat.N3(), m)
                : null;
    }

    private Matrix<N8, N1> distCoeffs() {
        double[] m = camera.getCameraInputs().distCoeffs;
        if (m.length == 0 || m.length > 8) {
            return null;
        }
        return new Matrix<>(
                org.wpilib.math.util.Nat.N8(),
                org.wpilib.math.util.Nat.N1(),
                java.util.Arrays.copyOf(m, 8));
    }
}
