package frc.spectrumLib.localization;

import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;
import org.wpilib.math.estimator.SwerveDrivePoseEstimator;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveDriveOdometry;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.linalg.VecBuilder;

/**
 * The robot's pose estimate, and the machinery to compare every source against it.
 *
 * <p>Three kinds of estimator run side by side, all fed the same odometry samples:
 *
 * <ul>
 *   <li><b>Fused</b> -- the robot pose. WPILib {@link SwerveDrivePoseEstimator}, corrected by the
 *       accepted measurements of every <em>enabled</em> source. This is what the robot drives on.
 *   <li><b>Odometry</b> -- wheels and gyro only, never corrected. The baseline every source is
 *       trying to beat, and the thing that shows how far the robot drifts without help.
 *   <li><b>Shadow, one per source</b> -- odometry plus <em>only</em> that source's accepted
 *       measurements, whether or not the source is enabled. This is the comparison: after a match
 *       each shadow track is "what the robot would have believed with only this source", and they
 *       can be overlaid in AdvantageScope. Seeds and explicit resets are applied to every shadow,
 *       so they all start from the same place.
 * </ul>
 *
 * <p>Everything here runs on the main loop from logged inputs, so the whole comparison is
 * regenerated in AdvantageKit replay, and a gate or std-dev change can be tested against a real
 * match without the robot.
 *
 * <p>Decisions carried over from the 2026 handoff: fusion stays WPILib's estimator (971's EKF was
 * evaluated and rejected), and it runs where the drivetrain loop runs -- on SystemCore.
 */
public class PoseFusion {

    private static final String PREFIX = "Localization/";

    private final SwerveDriveKinematics kinematics;
    private final SwerveDrivePoseEstimator fused;
    private final SwerveDriveOdometry odometry;
    private final Map<String, SwerveDrivePoseEstimator> shadows = new LinkedHashMap<>();

    private Rotation2d lastGyro = Rotation2d.kZero;
    private SwerveModulePosition[] lastPositions;

    /** Ground truth pose in simulation; empty on a robot and in replay without it logged. */
    @Getter private Optional<Pose2d> truth = Optional.empty();

    /**
     * Creates the estimators.
     *
     * @param kinematics drivetrain kinematics
     * @param gyro initial gyro angle
     * @param positions initial module positions
     * @param odometryStdDevs state std-devs [x, y, theta] (m, m, rad) for the wheel model
     */
    public PoseFusion(
            SwerveDriveKinematics kinematics,
            Rotation2d gyro,
            SwerveModulePosition[] positions,
            double[] odometryStdDevs) {
        this.kinematics = kinematics;
        this.lastGyro = gyro;
        this.lastPositions = copy(positions);
        this.fused = newEstimator(odometryStdDevs, Pose2d.kZero);
        this.odometry = new SwerveDriveOdometry(kinematics, gyro, positions, Pose2d.kZero);
        this.odometryStdDevs = odometryStdDevs;
    }

    private final double[] odometryStdDevs;

    /** Whether a real odometry sample has arrived yet. */
    private boolean haveOdometry = false;

    private SwerveDrivePoseEstimator newEstimator(double[] stateStdDevs, Pose2d start) {
        return new SwerveDrivePoseEstimator(
                kinematics,
                lastGyro,
                lastPositions,
                start,
                VecBuilder.fill(stateStdDevs[0], stateStdDevs[1], stateStdDevs[2]),
                VecBuilder.fill(0.9, 0.9, 0.9));
    }

    /**
     * Registers a source so it gets a shadow estimator. Call once per source at startup.
     *
     * @param source the source
     */
    public void register(PoseSource source) {
        shadows.computeIfAbsent(source.getName(), n -> newEstimator(odometryStdDevs, getPose()));
    }

    /**
     * Feeds one odometry sample to every estimator.
     *
     * @param timestampSeconds sample time, {@code Timer.getTimestamp()} base
     * @param gyro gyro yaw at the sample
     * @param positions module positions at the sample
     */
    public void addOdometry(
            double timestampSeconds, Rotation2d gyro, SwerveModulePosition[] positions) {
        lastGyro = gyro;
        lastPositions = copy(positions);
        if (!haveOdometry) {
            // The estimators were built before any sample existed, with a zero gyro and zero
            // wheel distances. Re-baseline them on the first real sample without moving the pose,
            // or the first update would read the gyro's boot angle and the motors' boot positions
            // as motion.
            haveOdometry = true;
            fused.resetPosition(gyro, positions, fused.getEstimatedPosition());
            odometry.resetPosition(gyro, positions, odometry.getPose());
            for (SwerveDrivePoseEstimator shadow : shadows.values()) {
                shadow.resetPosition(gyro, positions, shadow.getEstimatedPosition());
            }
            return;
        }
        fused.updateWithTime(timestampSeconds, gyro, positions);
        odometry.update(gyro, positions);
        for (SwerveDrivePoseEstimator shadow : shadows.values()) {
            shadow.updateWithTime(timestampSeconds, gyro, positions);
        }
    }

    /**
     * Applies one source's results: every accepted measurement goes to its shadow, and to the fused
     * pose if the source is enabled. Also logs each measurement's disagreement with the fused pose
     * at its own capture time, which is the per-frame comparison metric.
     *
     * @param source the source the results came from
     * @param results this loop's results
     * @param fuse whether accepted measurements also move the fused pose
     */
    public void apply(PoseSource source, List<PoseSource.Result> results, boolean fuse) {
        SwerveDrivePoseEstimator shadow = shadows.get(source.getName());
        int n = results.size();
        double[] errorMeters = new double[n];
        double[] headingErrorDeg = new double[n];
        double[] truthErrorMeters = new double[n];
        for (int i = 0; i < n; i++) {
            PoseSource.Result r = results.get(i);
            Pose2d measured = r.observation().pose2d();
            double t = r.observation().timestampSeconds();
            Pose2d reference = fused.sampleAt(t).orElse(getPose());
            errorMeters[i] = measured.getTranslation().getDistance(reference.getTranslation());
            headingErrorDeg[i] = measured.getRotation().minus(reference.getRotation()).getDegrees();
            truthErrorMeters[i] =
                    truth.map(p -> p.getTranslation().getDistance(measured.getTranslation()))
                            .orElse(Double.NaN);
            if (!r.accepted()) {
                continue;
            }
            var std =
                    VecBuilder.fill(
                            r.stdDevs().xyMeters(),
                            r.stdDevs().xyMeters(),
                            r.stdDevs().thetaRadians());
            if (shadow != null) {
                shadow.addVisionMeasurement(measured, t, std);
            }
            if (fuse) {
                fused.addVisionMeasurement(measured, t, std);
            }
        }
        String p = PREFIX + "Sources/" + source.getName();
        Logger.recordOutput(p + "/FusedThisLoop", fuse);
        Logger.recordOutput(p + "/ErrorVsFusedMeters", errorMeters);
        Logger.recordOutput(p + "/HeadingErrorVsFusedDeg", headingErrorDeg);
        if (truth.isPresent()) {
            Logger.recordOutput(p + "/ErrorVsTruthMeters", truthErrorMeters);
        }
    }

    /**
     * Adds one measurement straight to the fused pose and every shadow. For seeding: the one event
     * every track should share.
     *
     * @param pose measured pose
     * @param timestampSeconds capture time
     * @param xyStd translation std-dev, metres
     * @param thetaStd heading std-dev, radians
     */
    public void seed(Pose2d pose, double timestampSeconds, double xyStd, double thetaStd) {
        var std = VecBuilder.fill(xyStd, xyStd, thetaStd);
        fused.addVisionMeasurement(pose, timestampSeconds, std);
        for (SwerveDrivePoseEstimator shadow : shadows.values()) {
            shadow.addVisionMeasurement(pose, timestampSeconds, std);
        }
    }

    /**
     * Hard-resets every estimator to a pose (auto start, driver reorient).
     *
     * @param pose the new pose
     */
    public void resetPose(Pose2d pose) {
        fused.resetPosition(lastGyro, lastPositions, pose);
        odometry.resetPosition(lastGyro, lastPositions, pose);
        for (SwerveDrivePoseEstimator shadow : shadows.values()) {
            shadow.resetPosition(lastGyro, lastPositions, pose);
        }
    }

    /** The fused robot pose. */
    public Pose2d getPose() {
        return fused.getEstimatedPosition();
    }

    /** The fused pose at a past time, if still in the buffer. */
    public Optional<Pose2d> sampleAt(double timestampSeconds) {
        return fused.sampleAt(timestampSeconds);
    }

    /** Wheels-and-gyro-only pose. */
    public Pose2d getOdometryPose() {
        return odometry.getPose();
    }

    /** A source's shadow pose. */
    public Optional<Pose2d> getShadowPose(String source) {
        return Optional.ofNullable(shadows.get(source))
                .map(SwerveDrivePoseEstimator::getEstimatedPosition);
    }

    /**
     * Sets the simulation ground truth used for the {@code ErrorVsTruth} metrics.
     *
     * @param truthPose the true pose, or empty
     */
    public void setTruth(Optional<Pose2d> truthPose) {
        this.truth = truthPose;
    }

    /** Logs every track. Call once per loop after all sources are applied. */
    public void log() {
        Pose2d fusedPose = getPose();
        Logger.recordOutput(PREFIX + "FusedPose", fusedPose);
        Logger.recordOutput(PREFIX + "OdometryPose", getOdometryPose());
        for (var e : shadows.entrySet()) {
            Pose2d shadowPose = e.getValue().getEstimatedPosition();
            String p = PREFIX + "Shadow/" + e.getKey();
            Logger.recordOutput(p + "/Pose", shadowPose);
            Logger.recordOutput(
                    p + "/DistanceFromFusedMeters",
                    shadowPose.getTranslation().getDistance(fusedPose.getTranslation()));
            truth.ifPresent(
                    t ->
                            Logger.recordOutput(
                                    p + "/ErrorVsTruthMeters",
                                    shadowPose.getTranslation().getDistance(t.getTranslation())));
        }
        truth.ifPresent(
                t -> {
                    Logger.recordOutput(PREFIX + "SimTruthPose", t);
                    Logger.recordOutput(
                            PREFIX + "FusedErrorVsTruthMeters",
                            fusedPose.getTranslation().getDistance(t.getTranslation()));
                    Logger.recordOutput(
                            PREFIX + "OdometryErrorVsTruthMeters",
                            getOdometryPose().getTranslation().getDistance(t.getTranslation()));
                });
    }

    private static SwerveModulePosition[] copy(SwerveModulePosition[] positions) {
        SwerveModulePosition[] out = new SwerveModulePosition[positions.length];
        for (int i = 0; i < positions.length; i++) {
            out[i] = positions[i].copy();
        }
        return out;
    }
}
