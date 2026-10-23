package frc.spectrumLib.localization;

import org.wpilib.math.util.MathUtil;

/**
 * Checks, from logged inputs, that the gyros are mounted the way the code is told, against the
 * drivetrain's own wheel motion. Each check waits for enough evidence, then holds a verdict that
 * the caller logs and alerts on.
 *
 * <ul>
 *   <li><b>Pigeon level:</b> at rest, mount-corrected pitch and roll near 0.
 *   <li><b>Pigeon forward:</b> while the wheels accelerate hard, the Pigeon's X/Y acceleration
 *       points the same way. A Pigeon turned on its mount reads the turn here.
 *   <li><b>Yaw sense</b>, both gyros: the heading turns the way the wheels turn the robot. Wheels
 *       scrub while pushed or aiming, so a match reads about half the wheels' rate; only a reversed
 *       or dead yaw fails.
 *   <li><b>Onboard IMU orientation:</b> at rest, gravity on the axis its mount setting implies.
 *       WPILib applies the mount orientation to yaw only, so the raw accelerometer shows how
 *       SystemCore really sits.
 * </ul>
 */
public final class ImuMountCheck {

    public enum Verdict {
        UNKNOWN,
        OK,
        WRONG
    }

    static final double STILL_SPEED = 0.05;
    static final double STILL_TURN_RATE = 0.05;
    static final double REST_SECONDS = 2.0;
    static final double LEVEL_TOLERANCE_DEG = 10.0;

    static final double TURNING_RATE = 1.0;

    /** One full turn of evidence before judging a yaw sense. */
    static final double TURN_EVIDENCE_RADIANS = 2 * Math.PI;

    static final double MIN_YAW_GAIN = 0.3;

    static final double HARD_ACCEL = 2.0;
    static final double ACCEL_WINDOW_SECONDS = 0.1;
    static final double ACCEL_EVIDENCE_SECONDS = 2.0;
    static final double FORWARD_TOLERANCE_DEG = 45.0;

    /** Below this the Pigeon is not reading the acceleration at all (the sim's reads none), g. */
    static final double MIN_FORWARD_ACCEL_G = 0.02;

    /** Gravity's share on Z for a flat SystemCore, and the most it may have when vertical. */
    static final double FLAT_Z_SHARE = 0.9;

    static final double VERTICAL_Z_SHARE = 0.3;

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

    /** How a gyro's heading follows the wheels' turning. */
    public static final class YawSense {
        private boolean haveLast = false;
        private double lastYaw;
        private double lastTime;
        private double sumGyroWheel = 0;
        private double sumWheelWheel = 0;
        private double turned = 0;

        void update(boolean ok, double yaw, double time, boolean wheelsOk, double omega) {
            if (!ok) {
                haveLast = false;
                return;
            }
            if (haveLast && wheelsOk && Math.abs(omega) > TURNING_RATE) {
                double dt = time - lastTime;
                if (dt > 0) {
                    double gyroRate = MathUtil.angleModulus(yaw - lastYaw) / dt;
                    sumGyroWheel += gyroRate * omega * dt;
                    sumWheelWheel += omega * omega * dt;
                    turned += Math.abs(omega) * dt;
                }
            }
            haveLast = true;
            lastYaw = yaw;
            lastTime = time;
        }

        /** Gyro turn rate over wheel turn rate, least squares; NaN before any turning. */
        public double gain() {
            return sumWheelWheel > 0 ? sumGyroWheel / sumWheelWheel : Double.NaN;
        }

        public Verdict verdict() {
            if (turned < TURN_EVIDENCE_RADIANS) {
                return Verdict.UNKNOWN;
            }
            return gain() > MIN_YAW_GAIN ? Verdict.OK : Verdict.WRONG;
        }
    }

    private final boolean imuFlat;

    private final YawSense pigeonYaw = new YawSense();
    private final YawSense imuYaw = new YawSense();

    private double pigeonRestTime = 0;
    private double pitchSum = 0;
    private double rollSum = 0;

    private double imuRestTime = 0;
    private double imuGx = 0;
    private double imuGy = 0;
    private double imuGz = 0;

    // Recent wheel velocities, for acceleration over ACCEL_WINDOW_SECONDS.
    private final double[] histTime = new double[64];
    private final double[] histVx = new double[64];
    private final double[] histVy = new double[64];
    private int histCount = 0;
    private int histNext = 0;

    private double forwardDot = 0;
    private double forwardCross = 0;
    private double forwardTime = 0;

    private double lastTime = Double.NaN;

    /**
     * @param imuFlat the onboard IMU's mount setting is flat (gravity on its Z), else vertical
     */
    public ImuMountCheck(boolean imuFlat) {
        this.imuFlat = imuFlat;
    }

    public void update(Inputs in) {
        double dt = Double.isNaN(lastTime) ? 0 : Math.max(0, in.time() - lastTime);
        lastTime = in.time();

        pigeonYaw.update(in.pigeonOk(), in.pigeonYaw(), in.time(), in.wheelsOk(), in.omega());
        imuYaw.update(in.imuPresent(), in.imuYaw(), in.time(), in.wheelsOk(), in.omega());

        boolean still =
                in.wheelsOk()
                        && Math.hypot(in.vx(), in.vy()) < STILL_SPEED
                        && Math.abs(in.omega()) < STILL_TURN_RATE;
        if (still && in.pigeonOk()) {
            pigeonRestTime += dt;
            pitchSum += in.pigeonPitchDeg() * dt;
            rollSum += in.pigeonRollDeg() * dt;
        }
        if (still && in.imuPresent()) {
            imuRestTime += dt;
            imuGx += in.imuAccelX() * dt;
            imuGy += in.imuAccelY() * dt;
            imuGz += in.imuAccelZ() * dt;
        }

        if (in.wheelsOk()) {
            int past = pastIndex(in.time() - ACCEL_WINDOW_SECONDS);
            if (past >= 0 && in.pigeonOk()) {
                double span = in.time() - histTime[past];
                double ax = (in.vx() - histVx[past]) / span;
                double ay = (in.vy() - histVy[past]) / span;
                double a = Math.hypot(ax, ay);
                if (a > HARD_ACCEL) {
                    double px = in.pigeonAccelXG();
                    double py = in.pigeonAccelYG();
                    // Angle from the wheels' acceleration to the Pigeon's, magnitude-weighted.
                    forwardDot += (px * ax + py * ay) / a * dt;
                    forwardCross += (py * ax - px * ay) / a * dt;
                    forwardTime += dt;
                }
            }
            histTime[histNext] = in.time();
            histVx[histNext] = in.vx();
            histVy[histNext] = in.vy();
            histNext = (histNext + 1) % histTime.length;
            histCount = Math.min(histCount + 1, histTime.length);
        } else {
            histCount = 0;
        }
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

    public double pigeonRestPitchDeg() {
        return pigeonRestTime > 0 ? pitchSum / pigeonRestTime : Double.NaN;
    }

    public double pigeonRestRollDeg() {
        return pigeonRestTime > 0 ? rollSum / pigeonRestTime : Double.NaN;
    }

    public Verdict pigeonLevel() {
        if (pigeonRestTime < REST_SECONDS) {
            return Verdict.UNKNOWN;
        }
        return Math.abs(pigeonRestPitchDeg()) <= LEVEL_TOLERANCE_DEG
                        && Math.abs(pigeonRestRollDeg()) <= LEVEL_TOLERANCE_DEG
                ? Verdict.OK
                : Verdict.WRONG;
    }

    /** How far the Pigeon's X axis sits from the robot's forward, degrees CCW; NaN before data. */
    public double pigeonForwardErrorDeg() {
        // A Pigeon turned by +a sees the robot's acceleration turned by -a.
        return forwardTime > 0 ? Math.toDegrees(Math.atan2(-forwardCross, forwardDot)) : Double.NaN;
    }

    /** The Pigeon's mean acceleration during hard wheel acceleration, g. */
    public double pigeonForwardAccelG() {
        return forwardTime > 0 ? Math.hypot(forwardDot, forwardCross) / forwardTime : Double.NaN;
    }

    public Verdict pigeonForward() {
        if (forwardTime < ACCEL_EVIDENCE_SECONDS || pigeonForwardAccelG() < MIN_FORWARD_ACCEL_G) {
            return Verdict.UNKNOWN;
        }
        return Math.abs(pigeonForwardErrorDeg()) <= FORWARD_TOLERANCE_DEG
                ? Verdict.OK
                : Verdict.WRONG;
    }

    /** Gravity's share on the onboard IMU's Z axis at rest, 0 to 1; NaN before data. */
    public double imuGravityZShare() {
        double g = Math.sqrt(imuGx * imuGx + imuGy * imuGy + imuGz * imuGz);
        return g > 0 ? Math.abs(imuGz) / g : Double.NaN;
    }

    public Verdict imuOrientation() {
        if (imuRestTime < REST_SECONDS) {
            return Verdict.UNKNOWN;
        }
        double share = imuGravityZShare();
        return (imuFlat ? share >= FLAT_Z_SHARE : share <= VERTICAL_Z_SHARE)
                ? Verdict.OK
                : Verdict.WRONG;
    }
}
