package frc.spectrumLib.localization;

/**
 * Chooses how much the estimator should trust an observation that has passed every gate.
 *
 * <p>The 2026 Limelight code did this with confidence tiers ("Strong Multi integration", "Close
 * integration", ...), and a tier could still refuse ("Integration Criteria not Met"), so the model
 * may also reject. The tier name is logged next to each accepted measurement.
 */
@FunctionalInterface
public interface StdDevModel {

    /**
     * Picks std-devs for an observation.
     *
     * @param observation the gated observation
     * @param context robot state
     * @return the std-devs and tier, or {@code null} to reject as not meeting any tier
     */
    StdDevs choose(PoseObservation observation, GateContext context);

    /**
     * Standard deviations handed to the pose estimator.
     *
     * @param xyMeters translation std-dev, metres
     * @param thetaRadians heading std-dev, radians
     * @param tier name of the confidence tier that chose these
     */
    record StdDevs(double xyMeters, double thetaRadians, String tier) {}

    /** Rejection reason logged when a model returns {@code null}. */
    String NO_TIER = "Integration Criteria not Met";
}
