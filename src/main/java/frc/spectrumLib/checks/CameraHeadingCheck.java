package frc.spectrumLib.checks;

import frc.spectrumLib.checks.CheckResult.Status;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Every camera that solves a full pose from tags agrees on the robot's heading, with the gyro and
 * with each other.
 *
 * <p>For each multi-tag frame the caller passes the camera's heading and the gyro's yaw at the same
 * moment. The difference is the camera's offset from the gyro, which stays constant while the robot
 * turns, so frames from different moments and cameras can be compared. Each camera's offset is the
 * median over the last {@link #WINDOW_SECONDS}. A wrong mount yaw shifts one camera's offset by the
 * error: TXMAN1 Q30's flipped yaw sign put LL-Left 172 deg from LL-Back, and the pose was seeded
 * backwards from it.
 *
 * <p>The check fails when two cameras' offsets differ by more than {@link #FAIL_DEG}, or once the
 * pose heading has been seeded, when a camera's offset differs from the fused heading's by that
 * much. It cannot tell which camera of a disagreeing pair is wrong; it names both. Only multi-tag
 * frames count: a single tag's heading can flip, and MegaTag2-style solves take the heading from
 * the robot, so they must not be passed in.
 */
public final class CameraHeadingCheck implements Check {

    /**
     * Above this, a heading is wrong enough to send an auto the wrong way, and well above mount
     * error: TXMAN1's LL-Left yaw, 5.5 deg off, read 7-9 deg from LL-Back in matches. Q30's sign
     * error read 172.
     */
    public static final double FAIL_DEG = 15;

    /**
     * Above this, a camera's mount yaw is probably off and worth solving again. Before TXMAN1 Q13
     * and practice r16, LL-Back and LL-Left read 4.4 and 5.0 deg apart.
     */
    public static final double WARN_DEG = 6;

    /** Frames older than this are dropped, so a fixed or re-seeded heading clears soon. */
    public static final double WINDOW_SECONDS = 10;

    /** Frames a camera needs in the window before it is judged. */
    public static final int MIN_FRAMES = 10;

    /** Faster than this (rad/s), latency between the frame and the gyro reading would add error. */
    public static final double MAX_YAW_RATE = 0.3;

    /**
     * A worse verdict must last this long before it is reported. While the windows refill after the
     * heading changes under the cameras (in the simulator, the robot placed without its gyro
     * turning), cameras' medians flip at different moments and briefly disagree; a wrong mount
     * disagrees for good.
     */
    public static final double HOLD_SECONDS = 2.0;

    /** A gyro yaw step larger than this in one loop is a reset, not motion: start over. */
    private static final double GYRO_JUMP_DEG = 30;

    private record Sample(double time, double offsetDeg) {}

    private final Map<String, ArrayDeque<Sample>> cameras = new LinkedHashMap<>();
    private double fusedOffsetDeg = Double.NaN;
    private Status lastStatus = Status.PASS;
    private double lastGyroDeg = Double.NaN;
    private CheckResult reported = noCamera();
    private Status pendingStatus = null;
    private double pendingSince = Double.NaN;

    /**
     * Records one camera frame. Ignored unless it saw two or more tags and the robot was turning
     * slower than {@link #MAX_YAW_RATE}.
     *
     * @param camera the camera's name
     * @param cameraHeadingDeg the robot heading the camera solved
     * @param gyroYawDeg the gyro's yaw at the frame
     * @param tagCount tags in the solve
     * @param yawRateRadPerSec the robot's turn rate now
     * @param now this loop's robot time
     */
    public void addFrame(
            String camera,
            double cameraHeadingDeg,
            double gyroYawDeg,
            int tagCount,
            double yawRateRadPerSec,
            double now) {
        ArrayDeque<Sample> samples = cameras.computeIfAbsent(camera, c -> new ArrayDeque<>());
        if (tagCount < 2 || Math.abs(yawRateRadPerSec) > MAX_YAW_RATE) {
            return;
        }
        if (!Double.isFinite(cameraHeadingDeg) || !Double.isFinite(gyroYawDeg)) {
            return;
        }
        samples.addLast(new Sample(now, wrap(cameraHeadingDeg - gyroYawDeg)));
        // Bounded: a camera at 100 fps for the whole window is plenty of samples for a median.
        while (samples.size() > 200) {
            samples.removeFirst();
        }
    }

    /**
     * The robot's fused heading now, with the gyro's yaw now, so each camera can be compared with
     * the heading the robot will drive by. Call every loop; NaN until the heading is seeded.
     *
     * @param fusedHeadingDeg the fused pose's heading, or NaN if not seeded
     * @param gyroYawDeg the gyro's yaw
     */
    public void setRobotHeading(double fusedHeadingDeg, double gyroYawDeg) {
        if (Double.isFinite(lastGyroDeg)
                && Double.isFinite(gyroYawDeg)
                && Math.abs(wrap(gyroYawDeg - lastGyroDeg)) > GYRO_JUMP_DEG) {
            for (ArrayDeque<Sample> s : cameras.values()) {
                s.clear();
            }
        }
        lastGyroDeg = gyroYawDeg;
        fusedOffsetDeg =
                Double.isFinite(fusedHeadingDeg) && Double.isFinite(gyroYawDeg)
                        ? wrap(fusedHeadingDeg - gyroYawDeg)
                        : Double.NaN;
    }

    @Override
    public CheckResult evaluate(double now) {
        CheckResult r = judge(now);
        if (r.status().ordinal() > reported.status().ordinal()) {
            if (r.status() != pendingStatus) {
                pendingStatus = r.status();
                pendingSince = now;
            }
            if (now - pendingSince < HOLD_SECONDS) {
                return reported;
            }
        }
        pendingStatus = null;
        reported = r;
        return r;
    }

    private static CheckResult noCamera() {
        return CheckResult.warn(
                "no camera has seen 2+ tags in the last "
                        + (int) WINDOW_SECONDS
                        + " s, heading unchecked");
    }

    private CheckResult judge(double now) {
        List<String> names = new ArrayList<>();
        List<Double> offsets = new ArrayList<>();
        for (var e : cameras.entrySet()) {
            ArrayDeque<Sample> s = e.getValue();
            while (!s.isEmpty() && now - s.peekFirst().time() > WINDOW_SECONDS) {
                s.removeFirst();
            }
            if (s.size() >= MIN_FRAMES) {
                names.add(e.getKey());
                offsets.add(circularMedian(s));
            }
        }
        if (names.isEmpty()) {
            return noCamera();
        }

        // The worst pair of cameras, then the worst camera against the fused heading.
        double worstPair = 0;
        String pair = null;
        for (int i = 0; i < names.size(); i++) {
            for (int j = i + 1; j < names.size(); j++) {
                double d = Math.abs(wrap(offsets.get(i) - offsets.get(j)));
                if (d > worstPair) {
                    worstPair = d;
                    pair = names.get(i) + " and " + names.get(j);
                }
            }
        }
        double worstFused = 0;
        String vsFused = null;
        if (Double.isFinite(fusedOffsetDeg)) {
            for (int i = 0; i < names.size(); i++) {
                double d = Math.abs(wrap(offsets.get(i) - fusedOffsetDeg));
                if (d > worstFused) {
                    worstFused = d;
                    vsFused = names.get(i);
                }
            }
        }

        String pairText =
                pair == null
                        ? null
                        : String.format("%s disagree on heading by %.0f deg", pair, worstPair);
        String fusedText =
                vsFused == null
                        ? null
                        : String.format(
                                "%s is %.0f deg from the robot heading", vsFused, worstFused);
        double worst = Math.max(worstPair, worstFused);
        // The pair when it is as bad a verdict, so the line does not flip between the two
        // findings as their medians trade a degree.
        String text =
                pairText != null && severity(worstPair) >= severity(worstFused)
                        ? pairText
                        : fusedText;
        // A verdict holds until the error is a degree back inside its threshold: a median sitting
        // on the line would otherwise flip the alert every evaluation (TXMAN1 Q24 replay).
        double margin = 1;
        if (worst > FAIL_DEG - (lastStatus == Status.FAIL ? margin : 0)) {
            lastStatus = Status.FAIL;
            return CheckResult.fail(text + " - a camera's mount yaw is wrong");
        }
        if (worst > WARN_DEG - (lastStatus.ordinal() >= Status.WARN.ordinal() ? margin : 0)) {
            lastStatus = Status.WARN;
            return CheckResult.warn(text + " - check the camera mount yaw");
        }
        lastStatus = Status.PASS;
        return CheckResult.pass(
                String.join(", ", names)
                        + (names.size() == 1 ? " agrees" : " agree")
                        + String.format(" within %.0f deg", worst)
                        + (Double.isFinite(fusedOffsetDeg) ? "" : " (pose not seeded yet)"));
    }

    private static int severity(double deg) {
        return deg > FAIL_DEG ? 2 : deg > WARN_DEG ? 1 : 0;
    }

    /** The median of angles that may straddle +/-180: measured from the first, then put back. */
    private static double circularMedian(ArrayDeque<Sample> samples) {
        double ref = samples.peekFirst().offsetDeg();
        double[] d = new double[samples.size()];
        int i = 0;
        for (Sample s : samples) {
            d[i++] = wrap(s.offsetDeg() - ref);
        }
        Arrays.sort(d);
        return wrap(ref + d[d.length / 2]);
    }

    /** An angle in degrees, wrapped to [-180, 180). */
    static double wrap(double deg) {
        double w = (deg + 180) % 360;
        if (w < 0) {
            w += 360;
        }
        return w - 180;
    }
}
