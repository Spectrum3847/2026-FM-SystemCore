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
 *
 * <p>Only a camera the robot fuses can fail the check. One that is only logged (a SystemCore camera
 * with a placeholder mount, an Orin camera switched off) warns at most: its heading cannot move the
 * pose, so it is no reason to call the robot NO-GO.
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
    private final Map<String, Boolean> fused = new LinkedHashMap<>();
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
        addFrame(camera, true, cameraHeadingDeg, gyroYawDeg, tagCount, yawRateRadPerSec, now);
    }

    /**
     * Records one camera frame, saying whether the robot fuses that camera. Ignored unless it saw
     * two or more tags and the robot was turning slower than {@link #MAX_YAW_RATE}.
     *
     * @param camera the camera's name
     * @param cameraFused whether any of the camera's sources is fused into the pose now
     * @param cameraHeadingDeg the robot heading the camera solved
     * @param gyroYawDeg the gyro's yaw at the frame
     * @param tagCount tags in the solve
     * @param yawRateRadPerSec the robot's turn rate now
     * @param now this loop's robot time
     */
    public void addFrame(
            String camera,
            boolean cameraFused,
            double cameraHeadingDeg,
            double gyroYawDeg,
            int tagCount,
            double yawRateRadPerSec,
            double now) {
        fused.put(camera, cameraFused);
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

        // A verdict holds until the error is a degree back inside its threshold: a median sitting
        // on the line would otherwise flip the alert every evaluation (TXMAN1 Q24 replay).
        double margin = 1;
        double failAt = FAIL_DEG - (lastStatus == Status.FAIL ? margin : 0);
        double warnAt = WARN_DEG - (lastStatus.ordinal() >= Status.WARN.ordinal() ? margin : 0);

        // The worst pair of cameras, then the worst camera against the fused heading.
        Finding pair = null;
        double worst = 0;
        for (int i = 0; i < names.size(); i++) {
            for (int j = i + 1; j < names.size(); j++) {
                double d = Math.abs(wrap(offsets.get(i) - offsets.get(j)));
                worst = Math.max(worst, d);
                Finding f =
                        new Finding(
                                String.format(
                                        "%s and %s disagree on heading by %.0f deg",
                                        names.get(i), names.get(j), d),
                                d,
                                severity(d, failAt, warnAt, isFused(names.get(i), names.get(j))),
                                unfused(names.get(i), names.get(j)));
                pair = f.worseThan(pair) ? f : pair;
            }
        }
        Finding vsFused = null;
        if (Double.isFinite(fusedOffsetDeg)) {
            for (int i = 0; i < names.size(); i++) {
                double d = Math.abs(wrap(offsets.get(i) - fusedOffsetDeg));
                worst = Math.max(worst, d);
                Finding f =
                        new Finding(
                                String.format(
                                        "%s is %.0f deg from the robot heading", names.get(i), d),
                                d,
                                severity(d, failAt, warnAt, isFused(names.get(i))),
                                unfused(names.get(i)));
                vsFused = f.worseThan(vsFused) ? f : vsFused;
            }
        }

        // The pair when it is as bad a verdict, so the line does not flip between the two
        // findings as their medians trade a degree.
        Finding top =
                pair != null && (vsFused == null || pair.severity() >= vsFused.severity())
                        ? pair
                        : vsFused;
        if (top != null && top.severity() == 2) {
            lastStatus = Status.FAIL;
            return CheckResult.fail(top.text() + " - a camera's mount yaw is wrong");
        }
        if (top != null && top.severity() == 1) {
            lastStatus = Status.WARN;
            return CheckResult.warn(top.text() + " - check the camera mount yaw" + top.note());
        }
        lastStatus = Status.PASS;
        return CheckResult.pass(
                String.join(", ", names)
                        + (names.size() == 1 ? " agrees" : " agree")
                        + String.format(" within %.0f deg", worst)
                        + (Double.isFinite(fusedOffsetDeg) ? "" : " (pose not seeded yet)"));
    }

    /**
     * One disagreement: its line, size, severity (2 fail, 1 warn) and a note on unfused cameras.
     */
    private record Finding(String text, double deg, int severity, String note) {
        boolean worseThan(Finding other) {
            return other == null
                    || severity > other.severity
                    || (severity == other.severity && deg > other.deg);
        }
    }

    private static int severity(double deg, double failAt, double warnAt, boolean canFail) {
        int s = deg > failAt ? 2 : deg > warnAt ? 1 : 0;
        return canFail ? s : Math.min(s, 1);
    }

    private boolean isFused(String... names) {
        for (String n : names) {
            if (!fused.getOrDefault(n, true)) {
                return false;
            }
        }
        return true;
    }

    /** Names the cameras that are not fused, as a suffix for the reason, or "". */
    private String unfused(String... names) {
        List<String> out = new ArrayList<>();
        for (String n : names) {
            if (!fused.getOrDefault(n, true)) {
                out.add(n);
            }
        }
        return out.isEmpty() ? "" : " (" + String.join(", ", out) + " not fused)";
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
