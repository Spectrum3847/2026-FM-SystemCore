package frc.robot.subsystems.vision;

import frc.spectrumLib.localization.PoseObservation;
import frc.spectrumLib.localization.PoseObservation.Kind;
import frc.spectrumLib.localization.PoseSourceIO;
import frc.spectrumLib.localization.PoseSourceInputs;
import frc.spectrumLib.vision.Limelight;
import frc.spectrumLib.vision.LimelightHelpers.PoseEstimate;
import frc.spectrumLib.vision.LimelightHelpers.RawFiducial;
import java.util.ArrayList;
import java.util.List;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;

/**
 * Reads one Limelight's MegaTag1 or MegaTag2 estimate into {@link PoseSourceInputs}.
 *
 * <p>Each Limelight is two sources, {@code <camera>/MT1} and {@code <camera>/MT2}, so the two
 * algorithms get separate shadow tracks and can be compared frame for frame. Both IOs share one
 * {@link Limelight}, which caches its NetworkTables reads per loop ({@link Limelight#invalidate()}
 * is called once per loop by {@link Vision}).
 *
 * <p>A frame is reported once: an estimate whose timestamp matches the last one reported is the
 * same camera frame read again, and is skipped. (The 2026 code re-fused the latest frame every loop
 * whether or not it was new.)
 *
 * <p>MegaTag2 only publishes a 2-D pose, so its observation carries the same frame's MegaTag1
 * height, roll and pitch -- as the offseason code did -- so the tilt and height gates see a knocked
 * camera or bad solve whichever algorithm is being fused.
 */
public class LimelightIO implements PoseSourceIO {
    private final Limelight limelight;
    private final Kind kind;
    private double lastTimestamp = Double.NaN;

    /**
     * @param limelight the camera
     * @param kind {@link Kind#LIMELIGHT_MT1} or {@link Kind#LIMELIGHT_MT2}
     */
    public LimelightIO(Limelight limelight, Kind kind) {
        this.limelight = limelight;
        this.kind = kind;
    }

    @Override
    public void updateInputs(PoseSourceInputs inputs) {
        inputs.connected = limelight.isCameraConnected();
        List<PoseObservation> observations = new ArrayList<>(1);
        int[] tagIds = new int[0];
        double latencyMs = Double.NaN;

        if (limelight.isAttached() && limelight.targetInView()) {
            PoseEstimate estimate =
                    kind == Kind.LIMELIGHT_MT1
                            ? limelight.getMegaTag1_PoseEstimate()
                            : limelight.getMegaTag2_PoseEstimate();
            if (estimate != null
                    && estimate.pose != null
                    && estimate.tagCount > 0
                    && estimate.timestampSeconds != lastTimestamp) {
                lastTimestamp = estimate.timestampSeconds;
                latencyMs = estimate.latency;
                Pose3d mt1 = limelight.getMegaTag1_Pose3d();
                Pose3d pose =
                        kind == Kind.LIMELIGHT_MT1
                                ? mt1
                                : new Pose3d(
                                        estimate.pose.getX(),
                                        estimate.pose.getY(),
                                        mt1.getZ(),
                                        new Rotation3d(
                                                mt1.getRotation().getX(),
                                                mt1.getRotation().getY(),
                                                estimate.pose.getRotation().getRadians()));
                double maxAmbiguity = Double.NaN;
                RawFiducial[] fiducials = estimate.rawFiducials;
                if (fiducials != null && fiducials.length > 0) {
                    maxAmbiguity = 0;
                    tagIds = new int[fiducials.length];
                    for (int i = 0; i < fiducials.length; i++) {
                        maxAmbiguity = Math.max(maxAmbiguity, fiducials[i].ambiguity);
                        tagIds[i] = fiducials[i].id;
                    }
                }
                observations.add(
                        new PoseObservation(
                                "",
                                kind,
                                estimate.timestampSeconds,
                                pose,
                                estimate.tagCount,
                                estimate.avgTagDist,
                                limelight.getTargetSize(),
                                maxAmbiguity,
                                true));
            }
        }

        inputs.resize(observations.size());
        for (int i = 0; i < observations.size(); i++) {
            inputs.set(i, observations.get(i));
        }
        inputs.tagIds = tagIds;
        // [latency ms] of this loop's frame, NaN when there was none.
        inputs.health = new double[] {latencyMs};
    }
}
