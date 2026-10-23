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
 *   <li><b>Fused</b> is the robot pose: a WPILib {@link SwerveDrivePoseEstimator}, corrected by the
 *       accepted measurements of every <em>enabled</em> source. This is what the robot drives on.
 *   <li><b>Odometry</b> uses wheels and gyro only, and is never corrected. It is the baseline every
 *       source is trying to beat, and it shows how far the robot drifts without help.
 *   <li><b>Shadow, one per source</b> is odometry plus <em>only</em> that source's accepted
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
 * <p>Fusion runs where the drivetrain loop runs, on SystemCore. The fused track is WPILib's
 * estimator. A {@link TagEkf} runs alongside it on the same odometry, seeds and resets, taking
 * every camera's tags one at a time. It is logged for comparison only and never moves the robot
 * pose.
 */
public class PoseFusion {

    private static final String PREFIX = "Localization/";

    private final SwerveDriveKinematics kinematics;
    private final SwerveDrivePoseEstimator fused;
    private final SwerveDriveOdometry odometry;
    private final Map<String, SwerveDrivePoseEstimator> shadows = new LinkedHashMap<>();
    @Getter private final TagEkf tagEkf;

    /** Vision goes into every estimator through these, so arrival order cannot drop a frame. */
    private final OrderedVisionUpdates fusedVision;

    private final Map<String, OrderedVisionUpdates> shadowVision = new LinkedHashMap<>();

    /** Per-source log keys, built once at registration so no string work runs per loop. */
    private record Keys(
            String fusedThisLoop,
            String errorVsFused,
            String headingErrorVsFused,
            String errorVsTruth,
            String shadowPose,
            String shadowDistance,
            String shadowTruthError) {}

    private final Map<String, Keys> keys = new LinkedHashMap<>();

    /** Observations each source had last loop, so empty arrays are not rewritten every loop. */
    private final Map<String, Integer> lastObservationCount = new LinkedHashMap<>();

    /** Time of the last odometry sample, for starting a shadow with one buffer entry. */
    private double lastSampleTime = 0;

    private Rotation2d lastGyro = Rotation2d.kZero;
    private SwerveModulePosition[] lastPositions;
    private SwerveModulePosition[] lastRawPositions;

    /**
     * While the heading comes from the wheels, vision heading std-devs are divided by this. The
     * estimator's odometry std-devs are fixed at construction, and shrinking the measurement's by k
     * gives the same Kalman gain as growing the odometry's by k.
     */
    private static final double WHEEL_HEADING_STD_SCALE = 3.0;

    private boolean wheelHeading = false;

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
        this.lastRawPositions = copy(positions);
        this.fused = newEstimator(odometryStdDevs, Pose2d.kZero);
        this.fusedVision = new OrderedVisionUpdates(fused);
        this.odometry = new SwerveDriveOdometry(kinematics, gyro, positions, Pose2d.kZero);
        this.tagEkf = new TagEkf(kinematics.getModules());
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
        String n = source.getName();
        String p = PREFIX + "Sources/" + n;
        String sh = PREFIX + "Shadow/" + n;
        keys.put(
                n,
                new Keys(
                        p + "/FusedThisLoop",
                        p + "/ErrorVsFusedMeters",
                        p + "/HeadingErrorVsFusedDeg",
                        p + "/ErrorVsTruthMeters",
                        sh + "/Pose",
                        sh + "/DistanceFromFusedMeters",
                        sh + "/ErrorVsTruthMeters"));
    }

    /**
     * A source's shadow, created the first time the source has an accepted measurement.
     *
     * <p>Lazily, so a source whose device is absent (no Orin plugged in, no Quest) costs nothing:
     * every shadow is a full pose estimator updated with odometry every loop. A new shadow starts
     * from the fused pose with one odometry entry, so its first measurement is applied against it.
     */
    private SwerveDrivePoseEstimator shadowFor(String source) {
        return shadows.computeIfAbsent(
                source,
                n -> {
                    SwerveDrivePoseEstimator e = newEstimator(odometryStdDevs, getPose());
                    e.updateWithTime(lastSampleTime, lastGyro, lastPositions);
                    shadowVision.put(n, new OrderedVisionUpdates(e));
                    return e;
                });
    }

    private void clearVisionHistory() {
        fusedVision.clear();
        for (OrderedVisionUpdates v : shadowVision.values()) {
            v.clear();
        }
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
        addOdometry(timestampSeconds, gyro, positions, true);
    }

    /**
     * Feeds one odometry sample to the fused and odometry tracks, and to the shadows when {@code
     * includeShadows}.
     *
     * <p>The fused pose takes every 250 Hz sample. The shadows are comparison tracks and take one
     * per robot loop (the caller passes {@code true} for the loop's last sample): module positions
     * are absolute, so skipping intermediate samples only coarsens the arc between them, and it
     * cuts the shadows' cost by the number of samples per loop.
     *
     * @param timestampSeconds sample time, {@code Timer.getTimestamp()} base
     * @param gyro gyro yaw at the sample
     * @param positions module positions at the sample
     * @param includeShadows whether the shadow tracks take this sample
     */
    public void addOdometry(
            double timestampSeconds,
            Rotation2d gyro,
            SwerveModulePosition[] positions,
            boolean includeShadows) {
        addOdometry(timestampSeconds, gyro, positions, positions, includeShadows);
    }

    /**
     * As {@link #addOdometry(double, Rotation2d, SwerveModulePosition[], boolean)}, with separate
     * module positions for the odometry-only track: the fused pose and shadows can take adjusted
     * (e.g. tilt-scaled) wheel travel while {@code OdometryPose} keeps the raw wheels and gyro.
     *
     * @param timestampSeconds sample time, {@code Timer.getTimestamp()} base
     * @param gyro gyro yaw at the sample
     * @param positions module positions for the fused pose and the shadows
     * @param rawPositions module positions for the odometry-only track
     * @param includeShadows whether the shadow tracks take this sample
     */
    public void addOdometry(
            double timestampSeconds,
            Rotation2d gyro,
            SwerveModulePosition[] positions,
            SwerveModulePosition[] rawPositions,
            boolean includeShadows) {
        lastGyro = gyro;
        lastPositions = copy(positions);
        lastRawPositions = copy(rawPositions);
        lastSampleTime = timestampSeconds;
        tagEkf.addOdometry(timestampSeconds, gyro, positions);
        if (!haveOdometry) {
            // The estimators were built before any sample existed, with a zero gyro and zero
            // wheel distances. Re-baseline them on the first real sample without moving the pose,
            // or the first update would read the gyro's boot angle and the motors' boot positions
            // as motion.
            haveOdometry = true;
            fused.resetPosition(gyro, positions, fused.getEstimatedPosition());
            odometry.resetPosition(gyro, rawPositions, odometry.getPose());
            clearVisionHistory();
            for (SwerveDrivePoseEstimator shadow : shadows.values()) {
                shadow.resetPosition(gyro, positions, shadow.getEstimatedPosition());
            }
            return;
        }
        fused.updateWithTime(timestampSeconds, gyro, positions);
        odometry.update(gyro, rawPositions);
        if (includeShadows) {
            for (SwerveDrivePoseEstimator shadow : shadows.values()) {
                shadow.updateWithTime(timestampSeconds, gyro, positions);
            }
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
        String name = source.getName();
        Keys k = keys.get(name);
        int n = results.size();
        Integer last = lastObservationCount.put(name, n);
        if (k != null) {
            Logger.recordOutput(k.fusedThisLoop(), fuse);
        }
        if (n == 0 && last != null && last == 0) {
            return; // nothing new; the empty arrays are already in the log
        }
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
                            r.stdDevs().thetaRadians()
                                    / (wheelHeading ? WHEEL_HEADING_STD_SCALE : 1.0));
            shadowFor(name);
            shadowVision.get(name).add(measured, t, std, lastSampleTime);
            if (fuse) {
                fusedVision.add(measured, t, std, lastSampleTime);
            }
        }
        if (k != null) {
            Logger.recordOutput(k.errorVsFused(), errorMeters);
            Logger.recordOutput(k.headingErrorVsFused(), headingErrorDeg);
            if (truth.isPresent()) {
                Logger.recordOutput(k.errorVsTruth(), truthErrorMeters);
            }
        }
    }

    /**
     * Adds one measurement straight to the fused pose and every shadow. Used for seeding, the one
     * event every track should share.
     *
     * @param pose measured pose
     * @param timestampSeconds capture time
     * @param xyStd translation std-dev, metres
     * @param thetaStd heading std-dev, radians
     */
    public void seed(Pose2d pose, double timestampSeconds, double xyStd, double thetaStd) {
        seedFusedOnly(pose, timestampSeconds, xyStd, thetaStd);
        tagEkf.addPose(pose, timestampSeconds, xyStd, thetaStd);
    }

    /**
     * Like {@link #seed} but leaves the tag EKF alone, for a pose built from the fused translation
     * that would otherwise count odometry twice in the EKF.
     *
     * @param pose measured pose
     * @param timestampSeconds capture time
     * @param xyStd translation std-dev, metres
     * @param thetaStd heading std-dev, radians
     */
    public void seedFusedOnly(Pose2d pose, double timestampSeconds, double xyStd, double thetaStd) {
        var std = VecBuilder.fill(xyStd, xyStd, thetaStd);
        fusedVision.add(pose, timestampSeconds, std, lastSampleTime);
        for (OrderedVisionUpdates shadow : shadowVision.values()) {
            shadow.add(pose, timestampSeconds, std, lastSampleTime);
        }
    }

    /**
     * Hard-resets every estimator to a pose (auto start, driver reorient).
     *
     * @param pose the new pose
     */
    public void resetPose(Pose2d pose) {
        fused.resetPosition(lastGyro, lastPositions, pose);
        odometry.resetPosition(lastGyro, lastRawPositions, pose);
        clearVisionHistory();
        for (SwerveDrivePoseEstimator shadow : shadows.values()) {
            shadow.resetPosition(lastGyro, lastPositions, pose);
        }
        tagEkf.reset(pose);
    }

    /** Whether the odometry heading is coming from the wheels, both gyros being out. */
    public void setWheelHeading(boolean fromWheels) {
        wheelHeading = fromWheels;
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
        Logger.recordOutput(PREFIX + "OutOfOrderReapplied", fusedVision.reapplied());
        // Shadow tracks use the 10 Hz tier. At 100 Hz they were a fifth of the log. Every value
        // here comes from logged inputs, so replay regenerates them at any rate.
        if (frc.spectrumLib.telemetry.Telemetry.slowLogThisLoop()) {
            for (var e : shadows.entrySet()) {
                Keys k = keys.get(e.getKey());
                if (k == null) {
                    continue;
                }
                Pose2d shadowPose = e.getValue().getEstimatedPosition();
                Logger.recordOutput(k.shadowPose(), shadowPose);
                Logger.recordOutput(
                        k.shadowDistance(),
                        shadowPose.getTranslation().getDistance(fusedPose.getTranslation()));
                if (truth.isPresent()) {
                    Logger.recordOutput(
                            k.shadowTruthError(),
                            shadowPose.getTranslation().getDistance(truth.get().getTranslation()));
                }
            }
        }
        logTagEkf(fusedPose);
        if (truth.isPresent()) {
            Pose2d t = truth.get();
            Logger.recordOutput(PREFIX + "SimTruthPose", t);
            Logger.recordOutput(
                    PREFIX + "FusedErrorVsTruthMeters",
                    fusedPose.getTranslation().getDistance(t.getTranslation()));
            Logger.recordOutput(
                    PREFIX + "OdometryErrorVsTruthMeters",
                    getOdometryPose().getTranslation().getDistance(t.getTranslation()));
        }
    }

    private long lastEkfAccepted = -1;
    private long lastEkfRejected = -1;

    private void logTagEkf(Pose2d fusedPose) {
        String p = PREFIX + "EKF/";
        Pose2d ekf = tagEkf.getPose();
        double[] std = tagEkf.getStdDevs();
        Logger.recordOutput(p + "Pose", ekf);
        Logger.recordOutput(p + "Seeded", tagEkf.isSeeded());
        Logger.recordOutput(
                p + "StdDevs", new double[] {std[0], std[1], Math.toDegrees(std[2])}); // m, m, deg
        Logger.recordOutput(
                p + "ErrorVsFusedMeters",
                ekf.getTranslation().getDistance(fusedPose.getTranslation()));
        Logger.recordOutput(
                p + "HeadingErrorVsFusedDeg",
                ekf.getRotation().minus(fusedPose.getRotation()).getDegrees());
        truth.ifPresent(
                t ->
                        Logger.recordOutput(
                                p + "ErrorVsTruthMeters",
                                ekf.getTranslation().getDistance(t.getTranslation())));
        if (tagEkf.getAcceptedCount() != lastEkfAccepted
                || tagEkf.getRejectedCount() != lastEkfRejected) {
            lastEkfAccepted = tagEkf.getAcceptedCount();
            lastEkfRejected = tagEkf.getRejectedCount();
            Logger.recordOutput(p + "AcceptedCount", lastEkfAccepted);
            Logger.recordOutput(p + "RejectedCount", lastEkfRejected);
            Logger.recordOutput(p + "Relocalizations", tagEkf.getRelocalizations());
            for (var e : tagEkf.getRejectionCounts().entrySet()) {
                Logger.recordOutput(p + "RejectionCounts/" + e.getKey(), e.getValue());
            }
            for (var e : tagEkf.getCameraStats().entrySet()) {
                String c = p + "Cameras/" + e.getKey() + "/";
                TagEkf.CameraStats st = e.getValue();
                Logger.recordOutput(c + "AcceptedCount", st.accepted);
                Logger.recordOutput(c + "RejectedCount", st.rejected);
            }
        }
        // Residuals of tags that passed, this loop: a camera whose bearing residual sits off zero
        // has a mount yaw error.
        for (var e : tagEkf.getCameraStats().entrySet()) {
            TagEkf.CameraStats st = e.getValue();
            if (st.loopCount() > 0 || st.loggedResiduals) {
                String c = p + "Cameras/" + e.getKey() + "/";
                Logger.recordOutput(c + "BearingResidualsDeg", st.bearingResidualsDeg());
                Logger.recordOutput(c + "RangeResidualsMeters", st.rangeResidualsM());
                st.loggedResiduals = st.loopCount() > 0;
            }
        }
        tagEkf.clearLoopStats();
    }

    private static SwerveModulePosition[] copy(SwerveModulePosition[] positions) {
        SwerveModulePosition[] out = new SwerveModulePosition[positions.length];
        for (int i = 0; i < positions.length; i++) {
            out[i] = positions[i].copy();
        }
        return out;
    }
}
