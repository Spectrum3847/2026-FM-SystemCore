package frc.spectrumLib.mechanism;

/**
 * Tracks whether a motor has rebooted since its position was last set, so a homed mechanism stops
 * trusting a position that restarted from the boot value.
 *
 * <p>A TalonFX that browns out or loses power keeps its flash config, but its position restarts
 * from the boot value and its status-signal rates go back to defaults. Its {@code Connected} input
 * comes back true, so nothing else notices.
 *
 * <p>Two halves, split at the log. {@link #filter} runs on the robot before the reading is logged,
 * and turns Phoenix's raw flag into the logged {@code ResetOccurred} input. {@link #update} and
 * {@link #positionRestored} run on that logged input, so they replay.
 */
public final class MotorResetMonitor {

    /**
     * Loops a motor must be connected before its reset flag counts. Phoenix reports the first
     * contact with a device as a reset; a second of contact is long past that report.
     */
    static final int ARM_LOOPS = 50;

    private int connectedLoops = 0;

    private boolean positionLost = false;
    private int resets = 0;

    /**
     * Turns Phoenix's raw reset flag into the logged input. Robot side only: in replay the logged
     * value replaces the result.
     *
     * @param connected the motor answered this loop
     * @param rawReset {@code hasResetOccurred()} this loop, which must be called every loop so a
     *     first-contact report is consumed
     * @return the flag to log
     */
    public boolean filter(boolean connected, boolean rawReset) {
        boolean armed = connectedLoops >= ARM_LOOPS;
        if (connected && connectedLoops < ARM_LOOPS) {
            connectedLoops++;
        }
        return armed && rawReset;
    }

    /**
     * Advances on this loop's logged reset input.
     *
     * @param resetOccurred the logged {@code ResetOccurred} input
     * @param tracksPosition the mechanism relies on its position (it has soft limits)
     * @return {@code true} on the loop a reset is seen
     */
    public boolean update(boolean resetOccurred, boolean tracksPosition) {
        if (!resetOccurred) {
            return false;
        }
        resets++;
        if (tracksPosition) {
            positionLost = true;
        }
        return true;
    }

    /** The mechanism wrote a known position (it homed), so it can be trusted again. */
    public void positionRestored() {
        positionLost = false;
    }

    /** The motor rebooted since its position was last set. */
    public boolean isPositionLost() {
        return positionLost;
    }

    /** Resets seen since boot. */
    public int getResets() {
        return resets;
    }
}
