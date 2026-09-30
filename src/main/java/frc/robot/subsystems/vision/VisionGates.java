package frc.robot.subsystems.vision;

import frc.rebuilt.FieldHelpers;
import frc.spectrumLib.localization.Gate;
import frc.spectrumLib.localization.PoseObservation;
import frc.spectrumLib.localization.PoseObservation.Kind;
import frc.spectrumLib.localization.StdDevModel;
import frc.spectrumLib.localization.StdDevModel.StdDevs;
import java.util.List;
import org.wpilib.math.util.Units;

/**
 * Gate chains and trust models for FM's pose sources.
 *
 * <p>The AprilTag values are ported unchanged, comments and all, from the 2026 Spectrum code:
 * {@code main}'s FM {@code Vision.java} and the offseason branch's evidence-driven additions (the
 * lookback spin gate, the height gate, the one-second age gate). Change a threshold here only with
 * log evidence -- the standard the originals held themselves to -- and AdvantageKit replay is now
 * the way to get it: edit, replay a match, compare the {@code Localization/Shadow/*} tracks.
 */
public final class VisionGates {
    private VisionGates() {}

    // ── AprilTag gate thresholds ────────────────────────────────────────────

    /** Any single tag with ambiguity above this rejects the frame (pose-flip risk). */
    public static final double MAX_AMBIGUITY = 0.9;

    /**
     * Chassis yaw rate (rad/s) above which no estimate is fused. The camera stamps its heading from
     * whatever the robot last pushed, so a fast spin turns timestamp error into heading error, and
     * heading error into MegaTag2 translation error.
     */
    public static final double MAX_YAW_RATE_RAD_PER_SEC = 1.6;

    /**
     * How far back (seconds) the yaw-rate gate looks. A frame that arrives just after a spin stops
     * was captured during it, so the gate rejects on the peak rate over this window, not the rate
     * right now. 254 uses the same 0.3 s; it covers camera latency, NetworkTables latency and the
     * pose estimator's own lag with room to spare.
     */
    public static final double YAW_RATE_LOOKBACK_SECONDS = 0.3;

    /** Limelight target area (% of image) at or below which a frame is too far to trust. */
    public static final double MIN_TARGET_SIZE_PERCENT = 0.025;

    /** Roll or pitch of the solve beyond this means the camera was knocked or the solve is bad. */
    public static final double MAX_TILT_DEGREES = 5.0;

    /**
     * Solves whose robot height is further than this (metres) from the carpet are rejected. The
     * robot cannot leave the floor, so a solve that says it did is a bad solve or a bad mount
     * transform: the 30-versus-60 deg mount pitch of 2026-09-07 would have shown up here as every
     * pose sitting well above or below zero. The value is the AdvantageKit template default, on the
     * loose side on purpose until it has been watched at an event.
     */
    public static final double MAX_Z_ERROR_METERS = 0.75;

    /**
     * Estimates older than this (seconds, capture time to now) are rejected. The pose estimator
     * silently drops anything older than its 1.5 s buffer, so without this gate a camera with a bad
     * clock looks "integrating" while never moving the pose.
     */
    public static final double MAX_ESTIMATE_AGE_SECONDS = 1.0;

    /**
     * Estimates captured more than this (seconds) after now are rejected. A capture time in the
     * future is a clock or conversion fault, and the pose estimator does not reject it: it stores
     * the correction under the future time, where every later frame's out-of-order handling treats
     * it as the newest measurement. The margin covers the loop's own timestamp being read at the
     * start of the loop.
     */
    public static final double MAX_FUTURE_SECONDS = 0.05;

    /**
     * Variance used to effectively ignore a measurement dimension -- here, the heading of every
     * estimate fused while enabled, so the gyro owns heading during a match.
     */
    public static final double LARGE_VARIANCE = 999999.0;

    // ── Gates ───────────────────────────────────────────────────────────────

    /**
     * Rejects a capture time more than {@link #MAX_FUTURE_SECONDS} ahead of now; for every chain.
     */
    public static final Gate FUTURE_TIMESTAMP_GATE =
            Gate.rejectIf(
                    "Future Timestamp Rejection",
                    (o, c) -> o.timestampSeconds() - c.nowSeconds() > MAX_FUTURE_SECONDS);

    /** The shared AprilTag chain, for Limelight MegaTag1/2 and PhotonVision. */
    public static List<Gate> aprilTagGates() {
        return List.of(
                Gate.rejectIf("No Targets in View Rejection", (o, c) -> o.tagCount() <= 0),
                Gate.rejectIf(
                        "High Ambiguity Rejection",
                        (o, c) ->
                                !Double.isNaN(o.maxAmbiguity())
                                        && o.maxAmbiguity() > MAX_AMBIGUITY),
                Gate.rejectIf(
                        "Out of Field Rejection",
                        (o, c) -> FieldHelpers.poseOutOfField(o.pose2d())),
                Gate.rejectIf(
                        "Rotation Speed Rejection",
                        (o, c) -> c.peakYawRateRadPerSec() >= MAX_YAW_RATE_RAD_PER_SEC),
                Gate.rejectIf(
                        "Target Size Rejection",
                        (o, c) ->
                                !Double.isNaN(o.targetSizePercent())
                                        && o.targetSizePercent() <= MIN_TARGET_SIZE_PERCENT),
                Gate.rejectIf(
                        "Roll/Pitch Rejection",
                        (o, c) ->
                                Math.abs(o.pose().getRotation().getX())
                                                > Math.toRadians(MAX_TILT_DEGREES)
                                        || Math.abs(o.pose().getRotation().getY())
                                                > Math.toRadians(MAX_TILT_DEGREES)),
                Gate.rejectIf(
                        "Height Rejection",
                        (o, c) -> Math.abs(o.pose().getZ()) > MAX_Z_ERROR_METERS),
                Gate.rejectIf(
                        "Stale Estimate Rejection",
                        (o, c) -> c.nowSeconds() - o.timestampSeconds() > MAX_ESTIMATE_AGE_SECONDS),
                FUTURE_TIMESTAMP_GATE);
    }

    // ── Trust models ────────────────────────────────────────────────────────

    /**
     * The 2026 Limelight confidence tiers.
     *
     * <p>Heading is never fused while enabled. The Pigeon drifts a small fraction of a degree over
     * a match, while MegaTag1 yaw jitters by a degree or more frame to frame; heading is corrected
     * by the disabled pre-seeding and the gross-heading safety net in {@link Vision}, not here.
     *
     * <p>MegaTag2 relaxes the two pose-difference tiers while disabled, as the 2026 code did.
     */
    public static final StdDevModel LIMELIGHT_TIERS =
            (o, c) -> {
                boolean mt2 = o.kind() == Kind.LIMELIGHT_MT2;
                double size = o.targetSizePercent();
                double poseDifference =
                        c.fusedPose().getTranslation().getDistance(o.pose2d().getTranslation());
                double theta = Units.degreesToRadians(LARGE_VARIANCE);
                if (c.linearSpeed() <= 0.2 && size > 4) {
                    return new StdDevs(0.1, theta, "Stationary close integration");
                } else if (o.multiTag() && size > 2) {
                    return new StdDevs(0.1, theta, "Strong Multi integration");
                } else if (o.multiTag() && size > 0.2) {
                    return new StdDevs(0.25, theta, "Multi integration");
                } else if (size > 2 && (poseDifference < 0.5 || (mt2 && c.disabled()))) {
                    return new StdDevs(0.5, theta, "Close integration");
                } else if (size > 1 && (poseDifference < 0.25 || (mt2 && c.disabled()))) {
                    return new StdDevs(1.0, theta, "Proximity integration");
                } else if ((mt2 || o.maxAmbiguity() < 0.25) && size >= 0.03) {
                    return new StdDevs(1.5, theta, "Stable integration");
                }
                return null;
            };

    /**
     * PhotonVision on the Orin: distance-squared over tag count, the AdvantageKit vision template's
     * model, until the Orin has logs of its own to fit against. Heading is left to the gyro, as for
     * the Limelights.
     */
    public static final StdDevModel PHOTON_MODEL =
            (o, c) -> {
                double d = Double.isNaN(o.avgTagDistanceMeters()) ? 3.0 : o.avgTagDistanceMeters();
                double xy = 0.02 * d * d / Math.max(1, o.tagCount());
                return new StdDevs(
                        Math.max(xy, 0.02),
                        Units.degreesToRadians(LARGE_VARIANCE),
                        o.multiTag() ? "Photon multi-tag" : "Photon single-tag");
            };

    /**
     * QuestNav: QuestNav's own suggested std-devs for translation. Heading is left to the gyro by
     * default so that the Quest can be compared on translation alone first; raise trust in its
     * heading only once logs show it holds up.
     */
    public static final StdDevModel QUEST_MODEL =
            (o, c) -> new StdDevs(0.02, Units.degreesToRadians(LARGE_VARIANCE), "Quest tracking");

    /**
     * Shorthand used by {@link Vision} for sources that report {@link PoseObservation#tracking}.
     */
    static boolean isTracking(PoseObservation o) {
        return o.tracking();
    }
}
