package frc.spectrumLib.checks;

import java.util.Optional;
import java.util.function.Supplier;
import org.wpilib.math.geometry.Pose2d;

/**
 * An auto is selected, its file loaded, and the robot sits on its starting pose (254 2025's "Near
 * Auto Starting Pose").
 *
 * <p>The pose comparison needs a multi-tag vision seed taken after the code last put the pose on
 * the start. Before that the pose is only where the code put it, and comparing it with the start
 * would compare the start with itself, so the check warns that the start is unverified.
 *
 * <p>Once the robot is enabled it stops comparing: the robot has driven, and the start of the path
 * is history. It re-arms for the next match on a rising edge of FMS attach or a new match number,
 * and then needs a fresh seed too.
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
     * @param placements times the code has put the pose on the start since boot
     * @param multiTagSeeds times vision has seeded the pose from a multi-tag solve since boot
     * @param alliancePresent whether the Driver Station has reported an alliance
     * @param enabled whether the robot is enabled this loop
     * @param fmsAttached whether the FMS is attached this loop
     * @param matchNumber the match number the Driver Station reports
     */
    public record State(
            String autoName,
            boolean noAuto,
            boolean fileFound,
            Optional<Pose2d> start,
            Pose2d robotPose,
            long placements,
            long multiTagSeeds,
            boolean alliancePresent,
            boolean enabled,
            boolean fmsAttached,
            int matchNumber) {}

    private final Supplier<State> state;
    private double offSince = Double.NaN;
    private double distance = Double.NaN;
    private double headingError = Double.NaN;

    private long lastPlacements = 0;
    private boolean lastFmsAttached = false;
    private int lastMatchNumber = 0;
    private long seedsAtBaseline = 0;
    private boolean enabledSinceArmed = false;

    /**
     * @param state this loop's auto selection and robot pose
     */
    public AutoStartCheck(Supplier<State> state) {
        this.state = state;
    }

    /** Moves the seed baseline on a placement or a re-arm, and latches an enable. */
    private void track(State s) {
        boolean rearm = (s.fmsAttached() && !lastFmsAttached) || s.matchNumber() != lastMatchNumber;
        if (rearm) {
            enabledSinceArmed = false;
        }
        if (rearm || s.placements() != lastPlacements) {
            seedsAtBaseline = s.multiTagSeeds();
        }
        enabledSinceArmed |= s.enabled();
        lastPlacements = s.placements();
        lastFmsAttached = s.fmsAttached();
        lastMatchNumber = s.matchNumber();
    }

    private boolean seededSinceBaseline(State s) {
        return s.multiTagSeeds() > seedsAtBaseline;
    }

    @Override
    public void observe(double now) {
        State s = state.get();
        if (s != null) {
            track(s);
        }
        if (s == null || s.start().isEmpty() || !seededSinceBaseline(s) || enabledSinceArmed) {
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
        if (enabledSinceArmed) {
            return CheckResult.pass(
                    s.autoName() + " selected; start pose not checked after an enable this match");
        }
        if (!seededSinceBaseline(s)) {
            return CheckResult.warn(
                    "start pose unverified: no fresh multi-tag vision seed (" + s.autoName() + ")");
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
