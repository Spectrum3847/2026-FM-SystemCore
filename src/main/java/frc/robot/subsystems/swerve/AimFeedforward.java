package frc.robot.subsystems.swerve;

/**
 * The heading-rate feedforward {@link Swerve} hands its aim request ({@code
 * FieldCentricFacingAngle.withTargetRateFeedforward}) while aiming at the hub.
 *
 * <p>Units and sign line up with no conversion. {@code ShotCalculator} differentiates a
 * field-relative {@code Rotation2d}, so its {@code driveAngularVelocity} is radians per second,
 * counter-clockwise positive. Phoenix adds {@code TargetRateFeedforward} to the heading PID's
 * output, also rad/s counter-clockwise positive, in the request's forward perspective; this code
 * never calls {@code setOperatorPerspectiveForward}, so that perspective is the blue-alliance field
 * frame the target direction is already in, on both alliances.
 *
 * <p>Why it matters: without it the heading PID (kP 5 rad/s per rad) only turns once it has fallen
 * behind, so tracking a target moving at 0.3 rad/s it sits 0.06 rad (3.4 deg) behind. That is about
 * half the aim tolerance at 3 m.
 */
public final class AimFeedforward {
    private AimFeedforward() {}

    /**
     * Largest feedforward handed to the request, rad/s. 2910 clamps theirs to the same +/-2 rad/s
     * so a noisy derivative (a vision correction, a pose reset) cannot whip the robot round. For
     * scale: strafing at 1 m/s past the hub at 1.5 m needs 0.67 rad/s.
     */
    public static final double MAX_RAD_PER_SEC = 2.0;

    /**
     * The feedforward for a shot solution's heading rate.
     *
     * @param rateRadPerSec the shot solution's {@code driveAngularVelocity}, rad/s CCW positive
     * @return the clamped feedforward, rad/s CCW positive; 0 if the rate is not a finite number
     */
    public static double of(double rateRadPerSec) {
        if (!Double.isFinite(rateRadPerSec)) {
            return 0;
        }
        return Math.clamp(rateRadPerSec, -MAX_RAD_PER_SEC, MAX_RAD_PER_SEC);
    }
}
