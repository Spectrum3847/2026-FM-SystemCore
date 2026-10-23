package frc.spectrumLib.checks;

import java.util.Optional;
import java.util.function.Supplier;
import org.wpilib.math.geometry.Pose2d;

/**
 * An auto is selected, its file loaded, and the robot sits on its starting pose (254 2025's "Near
 * Auto Starting Pose").
 *
 * <p>The pose comparison needs a vision-seeded pose: before that the robot's pose is only where the
 * code put it, so the check warns that the start is unverified. After the first enable it no longer
 * compares: the robot has driven, and the start of the path is history.
 */
public final class AutoStartCheck implements Check {

    /** Farther than this from the start is the wrong spot, side or alliance. */
    public static final double MAX_DISTANCE_METERS = 0.5;

    public static final double MAX_HEADING_DEG = 10;

    /** How long the robot must stay off the start before it fails: it may be being placed. */
    public static final double HOLD_SECONDS = 1.0;

    /**
     * What the check needs, from logged inputs.
     *
     * @param autoName the selected auto's name
     * @param noAuto whether the selection is the do-nothing auto
     * @param fileFound whether the selection's auto file exists on the robot
     * @param start the first path's starting pose, flipped and mirrored as it will run; empty if no
     *     path loaded
     * @param robotPose the robot's fused pose
     * @param poseSeeded whether vision has seeded the pose's heading
     * @param alliancePresent whether the Driver Station has reported an alliance
     * @param hasBeenEnabled whether the robot has been enabled since boot
     */
    public record State(
            String autoName,
            boolean noAuto,
            boolean fileFound,
            Optional<Pose2d> start,
            Pose2d robotPose,
            boolean poseSeeded,
            boolean alliancePresent,
            boolean hasBeenEnabled) {}

    private final Supplier<State> state;
    private double offSince = Double.NaN;
    private double distance = Double.NaN;
    private double headingError = Double.NaN;

    /**
     * @param state this loop's auto selection and robot pose
     */
    public AutoStartCheck(Supplier<State> state) {
        this.state = state;
    }

    @Override
    public void observe(double now) {
        State s = state.get();
        if (s == null || s.start().isEmpty() || !s.poseSeeded() || s.hasBeenEnabled()) {
            offSince = Double.NaN;
            distance = Double.NaN;
            headingError = Double.NaN;
            return;
        }
        Pose2d start = s.start().get();
        distance = s.robotPose().getTranslation().getDistance(start.getTranslation());
        headingError = s.robotPose().getRotation().minus(start.getRotation()).getDegrees();
        boolean off = distance > MAX_DISTANCE_METERS || Math.abs(headingError) > MAX_HEADING_DEG;
        if (!off) {
            offSince = Double.NaN;
        } else if (Double.isNaN(offSince)) {
            offSince = now;
        }
    }

    @Override
    public CheckResult evaluate(double now) {
        State s = state.get();
        if (s == null) {
            return CheckResult.warn("no auto chooser");
        }
        if (s.noAuto()) {
            return CheckResult.warn("no auto selected (" + s.autoName() + ")");
        }
        if (!s.fileFound()) {
            return CheckResult.fail(
                    "'" + s.autoName() + "' has no auto file on the robot; it will do nothing");
        }
        if (s.start().isEmpty()) {
            return CheckResult.fail("'" + s.autoName() + "' loaded no paths");
        }
        if (!s.alliancePresent()) {
            return CheckResult.warn(
                    "no alliance from the Driver Station, so the start is not flipped yet");
        }
        if (s.hasBeenEnabled()) {
            return CheckResult.pass(
                    s.autoName() + " selected; start pose not checked after the first enable");
        }
        if (!s.poseSeeded()) {
            return CheckResult.warn(
                    "start pose unverified: no camera has seeded the pose yet ("
                            + s.autoName()
                            + ")");
        }
        String where = String.format("%.2f m and %.0f deg", distance, headingError);
        if (!Double.isNaN(offSince) && now - offSince >= HOLD_SECONDS) {
            return CheckResult.fail(
                    "robot is "
                            + where
                            + " from the start of "
                            + s.autoName()
                            + ": wrong spot, side or alliance, or a bad vision seed");
        }
        if (!Double.isNaN(offSince)) {
            return CheckResult.warn("robot is " + where + " from the start of " + s.autoName());
        }
        return CheckResult.pass(s.autoName() + " selected, robot on its start (" + where + ")");
    }

    /** Distance from the start, metres, or NaN when not compared. */
    public double getDistanceMeters() {
        return distance;
    }

    /** Heading error from the start, degrees, or NaN when not compared. */
    public double getHeadingErrorDeg() {
        return headingError;
    }
}
