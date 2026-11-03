package frc.spectrumLib.localization;

/**
 * Flags wheel slip by comparing the wheels' chassis acceleration with the IMU's. Spinning in place
 * (high-centred on a bump) and being pushed both show as wheels that accelerate when the robot
 * doesn't, or the reverse; the four modules still agree with each other and the gyro, so TagEkf's
 * own scrub term can't see it.
 *
 * <p>Both accelerations are smoothed over {@link #SMOOTH_SECONDS}; the IMU's slow offset from the
 * wheels (gravity on a tilted robot, accelerometer bias) is removed over {@link #BIAS_SECONDS}. A
 * gap above {@code threshold} flags slip, held for {@code holdSeconds} after it clears.
 *
 * <p>Robot frame throughout: x forward, y left. TXMAN1: correlation of the two is only 0.55-0.67
 * (wheel acceleration from 250 Hz encoders is noisy), so the threshold is tuned in replay.
 */
public final class SlipDetector {

    static final double SMOOTH_SECONDS = 0.1;
    static final double BIAS_SECONDS = 2.0;

    private final double threshold;
    private final double holdSeconds;

    private boolean started = false;
    private double lastT;
    private double vx;
    private double vy;
    private double awx;
    private double awy;
    private double aix;
    private double aiy;
    private double bx;
    private double by;
    private double lastSlipT = Double.NEGATIVE_INFINITY;
    private double gap;

    /**
     * @param threshold acceleration gap that counts as slip, m/s²
     * @param holdSeconds how long the flag stays up after the gap clears
     */
    public SlipDetector(double threshold, double holdSeconds) {
        this.threshold = threshold;
        this.holdSeconds = holdSeconds;
    }

    /**
     * One loop.
     *
     * @param t time, seconds
     * @param wheelVx wheel chassis velocity, robot frame, m/s
     * @param wheelVy wheel chassis velocity, robot frame, m/s
     * @param imuAx IMU acceleration, robot frame, m/s² (gravity included is fine)
     * @param imuAy IMU acceleration, robot frame, m/s²
     */
    public void update(double t, double wheelVx, double wheelVy, double imuAx, double imuAy) {
        if (!started) {
            started = true;
            lastT = t;
            vx = wheelVx;
            vy = wheelVy;
            aix = imuAx;
            aiy = imuAy;
            bx = imuAx;
            by = imuAy;
            return;
        }
        double dt = t - lastT;
        if (!(dt > 0)) {
            return;
        }
        lastT = t;
        double k = Math.min(1, dt / SMOOTH_SECONDS);
        double nvx = vx + k * (wheelVx - vx);
        double nvy = vy + k * (wheelVy - vy);
        awx += k * ((nvx - vx) / dt - awx);
        awy += k * ((nvy - vy) / dt - awy);
        vx = nvx;
        vy = nvy;
        aix += k * (imuAx - aix);
        aiy += k * (imuAy - aiy);
        double kb = Math.min(1, dt / BIAS_SECONDS);
        bx += kb * ((aix - awx) - bx);
        by += kb * ((aiy - awy) - by);
        gap = Math.hypot(awx - (aix - bx), awy - (aiy - by));
        if (gap > threshold) {
            lastSlipT = t;
        }
    }

    /** Whether slip was flagged within the hold time of {@code t}. */
    public boolean slipping(double t) {
        return t - lastSlipT < holdSeconds;
    }

    /** The last acceleration gap, m/s². */
    public double gap() {
        return gap;
    }
}
