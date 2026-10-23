package frc.spectrumLib.mechanism;

/**
 * Latches a mechanism to a stop once it has stayed out of tolerance of its closed-loop goal for a
 * debounce time: a jam, a slipped belt, a motor fighting a hard stop. The pattern is 6328's 2025
 * elevator, which stopped itself the same way rather than pushing into whatever held it.
 *
 * <p>Pure logic on values the caller reads from logged inputs, so it replays. The caller holds the
 * motor in neutral while {@link #isStopped()}.
 */
public final class TrackingStop {
    private final double tolerance;
    private final double debounceSeconds;

    /** When the error last went out of tolerance, or NaN while it is in. */
    private double outSinceSeconds = Double.NaN;

    private boolean fault = false;
    private boolean stopped = false;

    /**
     * @param tolerance largest error that counts as tracking, in the units of the error passed to
     *     {@link #update}
     * @param debounceSeconds how long the error must stay out before it is a fault
     */
    public TrackingStop(double tolerance, double debounceSeconds) {
        this.tolerance = tolerance;
        this.debounceSeconds = debounceSeconds;
    }

    /**
     * Advances one loop.
     *
     * @param nowSeconds the robot clock
     * @param enabled the robot is enabled; disabling clears the stop
     * @param connected the motor answered this loop. A disconnected motor's readings are stale or
     *     zero, so they restart the debounce instead of counting toward it.
     * @param closedLoop the motor ran a closed-loop setpoint since the last loop
     * @param error measured minus goal
     * @param act latch the stop on a fault; {@code false} only reports the fault
     * @return {@code true} on the loop the stop latches
     */
    public boolean update(
            double nowSeconds,
            boolean enabled,
            boolean connected,
            boolean closedLoop,
            double error,
            boolean act) {
        if (!enabled) {
            clear();
            return false;
        }
        boolean out = connected && closedLoop && Math.abs(error) > tolerance;
        if (!out) {
            outSinceSeconds = Double.NaN;
        } else if (Double.isNaN(outSinceSeconds)) {
            outSinceSeconds = nowSeconds;
        }
        fault = out && nowSeconds - outSinceSeconds >= debounceSeconds;
        if (fault && act && !stopped) {
            stopped = true;
            return true;
        }
        return false;
    }

    /** Releases the stop: a re-home, or the operator's override. */
    public void clear() {
        stopped = false;
        fault = false;
        outSinceSeconds = Double.NaN;
    }

    /** The error has been out of tolerance for the debounce time, whether or not that stops. */
    public boolean isFault() {
        return fault;
    }

    /** The mechanism is held in neutral until disabled, re-homed or cleared. */
    public boolean isStopped() {
        return stopped;
    }

    public double getTolerance() {
        return tolerance;
    }

    public double getDebounceSeconds() {
        return debounceSeconds;
    }
}
