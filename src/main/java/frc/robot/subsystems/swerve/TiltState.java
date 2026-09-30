package frc.robot.subsystems.swerve;

/**
 * What the Pigeon says about the robot's attitude and knocks, derived from logged inputs so it
 * replays. Used by odometry (tilt scaling), vision gating (sources that assume a level robot), and
 * available to shot gating ({@link Swerve#isTilted()}, {@link Swerve#wasBumpedRecently()}).
 *
 * <ul>
 *   <li><b>Tilt:</b> the angle between the robot's up axis and vertical, from pitch and roll.
 *   <li><b>Impact:</b> how far the total measured acceleration is from 1 g. Standing, driving
 *       smoothly or tilted it stays near 0 (gravity is 1 g in any orientation); a collision, a bump
 *       or landing off one shows as a spike, whatever the direction.
 *   <li><b>Odometry scale</b> (6328's {@code addOdometryObservation}, issue #10 section 8g): 1
 *       while level, falling linearly to 0 between {@link #TILT_DEADBAND_DEG} and {@link
 *       #TILT_ZERO_DEG}. On a bump the wheels slip, lift or roll up a slope, so their travel
 *       overstates the robot's motion across the floor.
 * </ul>
 */
public final class TiltState {
    /** Below this tilt (degrees) the floor and the IMU's own noise; odometry is not scaled. */
    public static final double TILT_DEADBAND_DEG = 2.0;

    /** At this tilt (degrees) and beyond, wheel travel is not counted at all (6328: 25 deg). */
    public static final double TILT_ZERO_DEG = 25.0;

    /** Tilt (degrees) above which the robot counts as tilted, for gating. */
    public static final double TILTED_DEG = 5.0;

    /** Impact (g) above which the robot counts as bumped. */
    public static final double BUMP_G = 0.5;

    /** How long (seconds) a bump keeps counting as recent. */
    public static final double BUMP_HOLD_SECONDS = 0.5;

    private double tiltDeg = 0;
    private double tiltRateDegPerSec = 0;
    private double impactG = 0;
    private double lastBumpSeconds = Double.NEGATIVE_INFINITY;

    /**
     * Updates from this loop's inputs.
     *
     * @param nowSeconds robot time
     * @param pitchDeg Pigeon pitch
     * @param rollDeg Pigeon roll
     * @param pitchRateDegPerSec Pigeon pitch rate
     * @param rollRateDegPerSec Pigeon roll rate
     * @param accelXG acceleration along the Pigeon's x, g
     * @param accelYG acceleration along y, g
     * @param accelZG acceleration along z, g
     */
    public void update(
            double nowSeconds,
            double pitchDeg,
            double rollDeg,
            double pitchRateDegPerSec,
            double rollRateDegPerSec,
            double accelXG,
            double accelYG,
            double accelZG) {
        tiltDeg = tiltDegrees(pitchDeg, rollDeg);
        tiltRateDegPerSec = Math.hypot(pitchRateDegPerSec, rollRateDegPerSec);
        double magnitude = Math.sqrt(accelXG * accelXG + accelYG * accelYG + accelZG * accelZG);
        // All zero means no data (sim, or a missing Pigeon), not free fall.
        impactG = magnitude == 0 ? 0 : Math.abs(magnitude - 1.0);
        if (impactG > BUMP_G) {
            lastBumpSeconds = nowSeconds;
        }
    }

    /** The angle between the robot's up axis and vertical, degrees. */
    public static double tiltDegrees(double pitchDeg, double rollDeg) {
        double c = Math.cos(Math.toRadians(pitchDeg)) * Math.cos(Math.toRadians(rollDeg));
        return Math.toDegrees(Math.acos(Math.max(-1, Math.min(1, c))));
    }

    /** The odometry scale for a tilt: 1 level, 0 at {@link #TILT_ZERO_DEG} and beyond. */
    public static double odometryScale(double tiltDeg) {
        if (tiltDeg <= TILT_DEADBAND_DEG) {
            return 1.0;
        }
        double u = (tiltDeg - TILT_DEADBAND_DEG) / (TILT_ZERO_DEG - TILT_DEADBAND_DEG);
        return Math.max(0.0, 1.0 - u);
    }

    public double tiltDeg() {
        return tiltDeg;
    }

    public double tiltRateDegPerSec() {
        return tiltRateDegPerSec;
    }

    public double impactG() {
        return impactG;
    }

    public boolean tilted() {
        return tiltDeg > TILTED_DEG;
    }

    /** Whether an impact above {@link #BUMP_G} happened in the last {@link #BUMP_HOLD_SECONDS}. */
    public boolean bumpedRecently(double nowSeconds) {
        return nowSeconds - lastBumpSeconds < BUMP_HOLD_SECONDS;
    }

    public double odometryScale() {
        return odometryScale(tiltDeg);
    }
}
