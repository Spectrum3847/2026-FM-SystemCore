package frc.spectrumLib.mechanism;

/**
 * A position written to a motor without waiting for the motor to answer, tracked until the motor's
 * readings show it.
 *
 * <p>Phoenix's {@code setPosition} waits up to 100 ms for the device by default, and a homing
 * mechanism writes its position in the middle of a match. Sent with no wait, the new position
 * arrives a status frame or more later. Until then the readings are still measured from the old
 * zero, and logic that treated them as homed would act on the wrong position. While {@link
 * #isPending()} the mechanism holds its setpoints and the tracking stop does not run.
 *
 * <p>The write has landed once the reading is nearer to where the new zero puts the mechanism than
 * to where the old one does. Both move with the mechanism's velocity meanwhile. A write the
 * readings still have not shown after {@link #RESEND_SECONDS} is sent again (a lost CAN frame).
 *
 * <p>Pure logic on logged inputs and the robot clock, so it replays.
 */
public final class PositionWrite {
    /** Send the write again when the readings have not shown it for this long. */
    static final double RESEND_SECONDS = 0.1;

    /**
     * A write this close to the reading, in rotations, changes nothing worth waiting for: 0.036 deg
     * of a mechanism geared to its output, well under any tolerance that acts on position.
     */
    static final double SAME_POSITION_ROTATIONS = 1e-4;

    private boolean pending = false;

    /** Where the motor reads once the write lands, carried along by the velocity since. */
    private double newFrame = Double.NaN;

    /** Where the motor reads while the write has not landed, carried along the same way. */
    private double oldFrame = Double.NaN;

    private double sentAt = Double.NaN;
    private double lastUpdate = Double.NaN;
    private int resends = 0;

    /**
     * The mechanism sent a position write this loop.
     *
     * @param nowSeconds the robot clock
     * @param position the position written, rotations
     * @param reading this loop's position reading, before the write
     */
    public void sent(double nowSeconds, double position, double reading) {
        if (Math.abs(position - reading) <= SAME_POSITION_ROTATIONS) {
            pending = false;
            return;
        }
        pending = true;
        newFrame = position;
        oldFrame = reading;
        sentAt = nowSeconds;
        lastUpdate = nowSeconds;
    }

    /**
     * Advances on one loop's logged readings.
     *
     * @param nowSeconds the robot clock
     * @param connected the motor answered this loop; a disconnected motor's readings are stale or
     *     zero, so they neither confirm the write nor move the frames
     * @param position this loop's position reading, rotations
     * @param velocity this loop's velocity reading, rotations per second
     * @return the position to send again this loop, or NaN
     */
    public double update(double nowSeconds, boolean connected, double position, double velocity) {
        if (!pending) {
            return Double.NaN;
        }
        double dt = nowSeconds - lastUpdate;
        lastUpdate = nowSeconds;
        if (!connected) {
            return Double.NaN;
        }
        newFrame += velocity * dt;
        oldFrame += velocity * dt;
        if (Math.abs(position - newFrame) < Math.abs(position - oldFrame)) {
            pending = false;
            return Double.NaN;
        }
        if (nowSeconds - sentAt >= RESEND_SECONDS - 1e-9) {
            resends++;
            sentAt = nowSeconds;
            // Measure the next try against the reading as it is now.
            oldFrame = position;
            if (Math.abs(newFrame - oldFrame) <= SAME_POSITION_ROTATIONS) {
                pending = false;
            }
            return newFrame;
        }
        return Double.NaN;
    }

    /** A write is out that the readings have not shown yet: the position is not trustworthy. */
    public boolean isPending() {
        return pending;
    }

    /** Writes sent again because the readings did not show them, since boot. */
    public int getResends() {
        return resends;
    }
}
