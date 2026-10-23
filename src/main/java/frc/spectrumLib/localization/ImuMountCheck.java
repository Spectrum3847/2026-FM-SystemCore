package frc.spectrumLib.localization;

import frc.spectrumLib.checks.Check;
import frc.spectrumLib.checks.CheckResult;
import java.util.ArrayList;
import java.util.List;
import org.wpilib.math.util.MathUtil;

/**
 * Checks, from logged inputs, that the gyros are mounted the way the code is told, against the
 * drivetrain's own wheel motion. Each check waits for enough evidence, then holds a verdict. As a
 * pre-match {@link Check}, a reversed yaw sense fails, any other wrong mount warns, and a robot
 * that has not driven enough since boot to judge anything is skipped.
 *
 * <ul>
 *   <li><b>Pigeon level:</b> at rest, mount-corrected pitch and roll near 0.
 *   <li><b>Pigeon forward:</b> while the wheels accelerate hard, the Pigeon's X/Y acceleration
 *       points the same way. A Pigeon turned on its mount reads the turn here.
 *   <li><b>Yaw sense</b>, both gyros: the heading turns the way the wheels turn the robot.
 *   <li><b>Onboard IMU orientation:</b> at rest, gravity on the axis its mount setting implies.
 *       WPILib applies the mount orientation to yaw only, so the raw accelerometer shows how
 *       SystemCore really sits.
 * </ul>
 *
 * <p>Evidence counts only when the gyro and the wheels move together. Wheels spinning on a cart,
 * pushing a wall, or a robot carried by hand give none, so a gyro that simply does not follow the
 * wheels is never called wrong. Rests count only shortly after such driving, so the robot is on the
 * floor. A check reads wrong only on clear contrary evidence: a reversed yaw, a consistent
 * acceleration direction far from the wheels', a pitch, roll or gravity axis a mount error away.
 *
 * <p>Each rest is judged on its own, and a rest-based check reads wrong only when every rest does.
 * A mount error tilts every rest alike, while a robot tipped on a bump or lifted with its wheels
 * still tilts only that rest.
 *
 * <p>Call {@link #update} every loop, enabled or not: most evidence comes from driving.
 */
public final class ImuMountCheck implements Check {

    public enum Verdict {
        UNKNOWN,
        OK,
        WRONG
    }

    static final double STILL_SPEED = 0.05;
    static final double STILL_TURN_RATE = 0.05;
    static final double REST_SECONDS = 2.0;

    /** Rests count this long after the wheels and a gyro last moved together. */
    static final double GROUND_RECENT_SECONDS = 60.0;

    /** Mount errors are tens of degrees; a robot parked on a bump is not one. */
    static final double LEVEL_TOLERANCE_DEG = 30.0;

    static final double WHEEL_TURNING_RATE = 1.0;
    static final double GYRO_TURNING_RATE = 0.5;

    /** One full turn of evidence before judging a yaw sense. */
    static final double TURN_EVIDENCE_RADIANS = 2 * Math.PI;

    /**
     * Wheels scrub while pushed or aiming, so a match reads about half the wheels' rate. The gain
     * is measured only over samples where the gyro also turns faster than {@link
     * #GYRO_TURNING_RATE}, which leaves out samples of a gyro barely turning; this threshold
     * assumes that gate, so changing {@code GYRO_TURNING_RATE} changes what it means.
     */
    static final double MIN_YAW_GAIN = 0.3;

    static final double HARD_ACCEL = 2.0;

    /**
     * The Pigeon's acceleration trails the wheel velocity by about 0.1 s (TXMAN1 Q34 and r16 line
     * up best there), so the wheels' window ends that much earlier.
     */
    static final double ACCEL_WINDOW_SECONDS = 0.3;

    static final double PIGEON_LAG_SECONDS = 0.1;
    static final double ACCEL_EVIDENCE_SECONDS = 5.0;

    /**
     * The Pigeon must feel about what the wheels say, or the wheels are not moving the robot:
     * spinning on a cart, slipping while pushed, or the robot taking a hit.
     */
    static final double MIN_FELT_RATIO = 0.5;

    static final double MAX_FELT_RATIO = 2.0;

    private static final double G = 9.81;

    /** How consistently the Pigeon's direction sits from the wheels', 0 (random) to 1. */
    static final double MIN_FORWARD_CONSISTENCY = 0.5;

    /** A forward verdict, once reached, holds until consistency falls below this. */
    static final double KEEP_FORWARD_CONSISTENCY = 0.4;

    static final double FORWARD_TOLERANCE_DEG = 45.0;

    /** Gravity's share on Z for a flat SystemCore, and the most it may have when vertical. */
    static final double FLAT_Z_SHARE = 0.9;

    static final double VERTICAL_Z_SHARE = 0.3;

    /** A rest whose accelerometer reads less than this (m/s^2) is not reading gravity at all. */
    static final double MIN_GRAVITY = 0.5 * G;

    /**
     * One loop's readings.
     *
     * @param time seconds
     * @param wheelsOk every module connected, so the wheel velocity is real
     * @param vx wheel-measured robot-relative velocity, m/s
     * @param vy wheel-measured robot-relative velocity, m/s
     * @param omega wheel-measured turn rate, rad/s
     * @param pigeonOk Pigeon connected
     * @param pigeonYaw radians
     * @param pigeonPitchDeg mount-corrected
     * @param pigeonRollDeg mount-corrected
     * @param pigeonAccelXG g
     * @param pigeonAccelYG g
     * @param imuPresent onboard IMU read this loop
     * @param imuYaw radians, after WPILib's mount orientation
     * @param imuAccelX raw, m/s^2
     * @param imuAccelY raw, m/s^2
     * @param imuAccelZ raw, m/s^2
     */
    public record Inputs(
            double time,
            boolean wheelsOk,
            double vx,
            double vy,
            double omega,
            boolean pigeonOk,
            double pigeonYaw,
            double pigeonPitchDeg,
            double pigeonRollDeg,
            double pigeonAccelXG,
            double pigeonAccelYG,
            boolean imuPresent,
            double imuYaw,
            double imuAccelX,
            double imuAccelY,
            double imuAccelZ) {}

    /** How a gyro's heading follows the wheels' turning, while both are turning. */
    public static final class YawSense {
        private boolean haveLast = false;
        private double lastYaw;
        private double lastTime;
        private double sumGyroWheel = 0;
        private double sumWheelWheel = 0;
        private double turned = 0;

        /** Returns whether this sample showed the gyro and the wheels turning together. */
        boolean update(boolean ok, double yaw, double time, boolean wheelsOk, double omega) {
            if (!ok) {
                haveLast = false;
                return false;
            }
            boolean evidence = false;
            double dt = time - lastTime;
            if (haveLast && wheelsOk && dt > 0 && Math.abs(omega) > WHEEL_TURNING_RATE) {
                double gyroRate = MathUtil.angleModulus(yaw - lastYaw) / dt;
                if (Math.abs(gyroRate) > GYRO_TURNING_RATE) {
                    sumGyroWheel += gyroRate * omega * dt;
                    sumWheelWheel += omega * omega * dt;
                    turned += Math.abs(omega) * dt;
                    evidence = true;
                }
            }
            haveLast = true;
            lastYaw = yaw;
            lastTime = time;
            return evidence;
        }

        /** Gyro turn rate over wheel turn rate, least squares; NaN before any turning. */
        public double gain() {
            return sumWheelWheel > 0 ? sumGyroWheel / sumWheelWheel : Double.NaN;
        }

        public Verdict verdict() {
            if (turned < TURN_EVIDENCE_RADIANS) {
                return Verdict.UNKNOWN;
            }
            if (gain() > MIN_YAW_GAIN) {
                return Verdict.OK;
            }
            return gain() < -MIN_YAW_GAIN ? Verdict.WRONG : Verdict.UNKNOWN;
        }
    }

    private final boolean imuFlat;
    private final double pigeonMountYawDeg;

    private final YawSense pigeonYaw = new YawSense();
    private final YawSense imuYaw = new YawSense();

    private double lastGroundTime = Double.NEGATIVE_INFINITY;

    // The rest under way, and the most level rest judged so far.
    private double pigeonRestTime = 0;
    private double pitchSum = 0;
    private double rollSum = 0;
    private double levelTiltDeg = Double.POSITIVE_INFINITY;
    private double levelPitchDeg = Double.NaN;
    private double levelRollDeg = Double.NaN;

    // The rest under way, and the rest closest to the mount setting judged so far.
    private double imuRestTime = 0;
    private double imuGx = 0;
    private double imuGy = 0;
    private double imuGz = 0;
    private double bestZShare = Double.NaN;

    // Recent wheel velocities, for acceleration over ACCEL_WINDOW_SECONDS.
    private final double[] histTime = new double[128];
    private final double[] histVx = new double[128];
    private final double[] histVy = new double[128];
    private final double[] histPx = new double[128];
    private final double[] histPy = new double[128];
    private int histCount = 0;
    private int histNext = 0;

    // Sum of unit vectors from the wheels' acceleration to the Pigeon's, time-weighted.
    private double forwardCos = 0;
    private double forwardSin = 0;
    private double forwardTime = 0;
    private Verdict forwardVerdict = Verdict.UNKNOWN;

    private double lastTime = Double.NaN;

    /**
     * @param imuFlat the onboard IMU's mount setting is flat (gravity on its Z), else vertical
     * @param pigeonMountYawDeg the Pigeon's MountPoseYaw, for the reason a wrong forward gives
     */
    public ImuMountCheck(boolean imuFlat, double pigeonMountYawDeg) {
        this.imuFlat = imuFlat;
        this.pigeonMountYawDeg = pigeonMountYawDeg;
    }

    public void update(Inputs in) {
        double dt = Double.isNaN(lastTime) ? 0 : Math.max(0, in.time() - lastTime);
        lastTime = in.time();

        boolean turnedTogether =
                pigeonYaw.update(
                        in.pigeonOk(), in.pigeonYaw(), in.time(), in.wheelsOk(), in.omega());
        turnedTogether |=
                imuYaw.update(in.imuPresent(), in.imuYaw(), in.time(), in.wheelsOk(), in.omega());
        boolean feltTogether = updateForward(in, dt);
        boolean movedTogether = turnedTogether || feltTogether;
        forwardVerdict = judgeForward();
        if (movedTogether) {
            lastGroundTime = in.time();
        }

        boolean still =
                in.wheelsOk()
                        && Math.hypot(in.vx(), in.vy()) < STILL_SPEED
                        && Math.abs(in.omega()) < STILL_TURN_RATE
                        && in.time() - lastGroundTime <= GROUND_RECENT_SECONDS;
        if (still
                && in.pigeonOk()
                && Double.isFinite(in.pigeonPitchDeg())
                && Double.isFinite(in.pigeonRollDeg())) {
            pigeonRestTime += dt;
            pitchSum += in.pigeonPitchDeg() * dt;
            rollSum += in.pigeonRollDeg() * dt;
            if (pigeonRestTime >= REST_SECONDS) {
                double pitch = pitchSum / pigeonRestTime;
                double roll = rollSum / pigeonRestTime;
                double tilt = Math.max(Math.abs(pitch), Math.abs(roll));
                if (tilt < levelTiltDeg) {
                    levelTiltDeg = tilt;
                    levelPitchDeg = pitch;
                    levelRollDeg = roll;
                }
            }
        } else {
            pigeonRestTime = 0;
            pitchSum = 0;
            rollSum = 0;
        }
        if (still
                && in.imuPresent()
                && Double.isFinite(in.imuAccelX())
                && Double.isFinite(in.imuAccelY())
                && Double.isFinite(in.imuAccelZ())) {
            imuRestTime += dt;
            imuGx += in.imuAccelX() * dt;
            imuGy += in.imuAccelY() * dt;
            imuGz += in.imuAccelZ() * dt;
            double norm = Math.sqrt(imuGx * imuGx + imuGy * imuGy + imuGz * imuGz);
            if (imuRestTime >= REST_SECONDS && norm / imuRestTime >= MIN_GRAVITY) {
                double share = Math.abs(imuGz) / norm;
                if (Double.isNaN(bestZShare)
                        || (imuFlat ? share > bestZShare : share < bestZShare)) {
                    bestZShare = share;
                }
            }
        } else {
            imuRestTime = 0;
            imuGx = 0;
            imuGy = 0;
            imuGz = 0;
        }
    }

    /** Returns whether this sample showed the Pigeon feeling the wheels' acceleration. */
    private boolean updateForward(Inputs in, double dt) {
        if (!in.wheelsOk() || !in.pigeonOk()) {
            histCount = 0;
            return false;
        }
        histTime[histNext] = in.time();
        histVx[histNext] = in.vx();
        histVy[histNext] = in.vy();
        histPx[histNext] = in.pigeonAccelXG();
        histPy[histNext] = in.pigeonAccelYG();
        int now = histNext;
        histNext = (histNext + 1) % histTime.length;
        histCount = Math.min(histCount + 1, histTime.length);

        int end = pastIndex(in.time() - PIGEON_LAG_SECONDS);
        int start = pastIndex(in.time() - PIGEON_LAG_SECONDS - ACCEL_WINDOW_SECONDS);
        int pigeonStart = pastIndex(in.time() - ACCEL_WINDOW_SECONDS);
        if (end < 0 || start < 0 || pigeonStart < 0) {
            return false;
        }
        double span = histTime[end] - histTime[start];
        if (span <= 0) {
            return false;
        }
        double ax = (histVx[end] - histVx[start]) / span;
        double ay = (histVy[end] - histVy[start]) / span;
        double a = Math.hypot(ax, ay);
        // The Pigeon's mean over its window: one sample is mostly vibration.
        double px = 0;
        double py = 0;
        int n = 0;
        for (int i = now; i != pigeonStart; i = Math.floorMod(i - 1, histTime.length)) {
            px += histPx[i];
            py += histPy[i];
            n++;
        }
        px /= n;
        py /= n;
        double p = Math.hypot(px, py);
        double felt = p * G / a;
        if (a > HARD_ACCEL && felt >= MIN_FELT_RATIO && felt <= MAX_FELT_RATIO) {
            forwardCos += (px * ax + py * ay) / (a * p) * dt;
            forwardSin += (py * ax - px * ay) / (a * p) * dt;
            forwardTime += dt;
            return true;
        }
        return false;
    }

    /** The newest stored sample at or before {@code time}, or -1. */
    private int pastIndex(double time) {
        for (int k = 1; k <= histCount; k++) {
            int i = Math.floorMod(histNext - k, histTime.length);
            if (histTime[i] <= time) {
                return i;
            }
        }
        return -1;
    }

    public YawSense pigeonYawSense() {
        return pigeonYaw;
    }

    public YawSense imuYawSense() {
        return imuYaw;
    }

    /** Mean pitch over the most level rest; NaN before a rest. */
    public double pigeonRestPitchDeg() {
        return levelPitchDeg;
    }

    /** Mean roll over the most level rest; NaN before a rest. */
    public double pigeonRestRollDeg() {
        return levelRollDeg;
    }

    public Verdict pigeonLevel() {
        if (Double.isInfinite(levelTiltDeg)) {
            return Verdict.UNKNOWN;
        }
        return levelTiltDeg <= LEVEL_TOLERANCE_DEG ? Verdict.OK : Verdict.WRONG;
    }

    /** How far the Pigeon's X axis sits from the robot's forward, degrees CCW; NaN before data. */
    public double pigeonForwardErrorDeg() {
        // A Pigeon turned by +a sees the robot's acceleration turned by -a.
        return forwardTime > 0 ? Math.toDegrees(Math.atan2(-forwardSin, forwardCos)) : Double.NaN;
    }

    /** 1 when every sample put the Pigeon at the same angle from the wheels, near 0 if random. */
    public double pigeonForwardConsistency() {
        return forwardTime > 0 ? Math.hypot(forwardCos, forwardSin) / forwardTime : Double.NaN;
    }

    public Verdict pigeonForward() {
        return forwardVerdict;
    }

    private Verdict judgeForward() {
        double needed =
                forwardVerdict == Verdict.UNKNOWN
                        ? MIN_FORWARD_CONSISTENCY
                        : KEEP_FORWARD_CONSISTENCY;
        if (forwardTime < ACCEL_EVIDENCE_SECONDS || pigeonForwardConsistency() < needed) {
            return Verdict.UNKNOWN;
        }
        return Math.abs(pigeonForwardErrorDeg()) <= FORWARD_TOLERANCE_DEG
                ? Verdict.OK
                : Verdict.WRONG;
    }

    /**
     * Gravity's share on the onboard IMU's Z axis, 0 to 1, over the rest closest to its mount
     * setting; NaN before a rest that read gravity.
     */
    public double imuGravityZShare() {
        return bestZShare;
    }

    public Verdict imuOrientation() {
        if (Double.isNaN(bestZShare)) {
            return Verdict.UNKNOWN;
        }
        return (imuFlat ? bestZShare >= FLAT_Z_SHARE : bestZShare <= VERTICAL_Z_SHARE)
                ? Verdict.OK
                : Verdict.WRONG;
    }

    /**
     * A reversed yaw sense fails: the heading, and so field-relative driving, turns the wrong way.
     * Any other wrong mount warns, naming the setting to look at with the measured numbers.
     */
    @Override
    public CheckResult evaluate(double nowSeconds) {
        List<String> ok = new ArrayList<>();
        List<String> warn = new ArrayList<>();
        List<String> fail = new ArrayList<>();
        if (judge(pigeonYaw.verdict(), "Pigeon yaw", ok)) {
            fail.add(reversedYaw("Pigeon", pigeonYaw));
        }
        if (judge(imuYaw.verdict(), "onboard IMU yaw", ok)) {
            fail.add(reversedYaw("onboard IMU", imuYaw));
        }
        if (judge(pigeonLevel(), "Pigeon level", ok)) {
            warn.add(
                    String.format(
                            "Pigeon reads pitch %.0f deg, roll %.0f deg at rest (check"
                                    + " MountPosePitch/Roll)",
                            pigeonRestPitchDeg(), pigeonRestRollDeg()));
        }
        if (judge(pigeonForward(), "Pigeon forward", ok)) {
            warn.add(
                    String.format(
                            "Pigeon forward is %.0f deg from the robot's (check MountPoseYaw, now"
                                    + " %.0f deg)",
                            pigeonForwardErrorDeg(), pigeonMountYawDeg));
        }
        if (judge(imuOrientation(), "onboard IMU orientation", ok)) {
            warn.add(
                    String.format(
                            "onboard IMU has %.0f%% of gravity on Z at rest but is set %s",
                            100 * imuGravityZShare(), imuFlat ? "flat" : "vertical"));
        }
        List<String> wrong = new ArrayList<>(fail);
        wrong.addAll(warn);
        if (!fail.isEmpty()) {
            return CheckResult.fail(String.join("; ", wrong));
        }
        if (!warn.isEmpty()) {
            return CheckResult.warn(String.join("; ", wrong));
        }
        if (ok.isEmpty()) {
            return CheckResult.skip("not enough driving since boot");
        }
        return CheckResult.pass(String.join(", ", ok) + " OK");
    }

    /** Adds a name that read OK to {@code ok}; returns whether the verdict is WRONG. */
    private static boolean judge(Verdict v, String name, List<String> ok) {
        if (v == Verdict.OK) {
            ok.add(name);
        }
        return v == Verdict.WRONG;
    }

    private static String reversedYaw(String gyro, YawSense sense) {
        return String.format(
                "%s yaw turns against the wheels (%.2fx their rate): reversed", gyro, sense.gain());
    }
}
