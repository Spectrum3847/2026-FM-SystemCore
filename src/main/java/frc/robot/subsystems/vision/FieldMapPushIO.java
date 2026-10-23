package frc.robot.subsystems.vision;

/**
 * Hardware layer for pushing the robot's field map to one Limelight. Replay and the simulator use
 * {@link #NONE}: nothing is sent, and replay reads the robot's results from the log.
 */
public interface FieldMapPushIO {
    /** Sends nothing and reports nothing. */
    FieldMapPushIO NONE = (inputs, mayPush) -> {};

    /**
     * Fills {@code inputs} with the latest push results, and starts a push in the background when
     * allowed and none is running. Never blocks.
     *
     * @param inputs the inputs to overwrite
     * @param mayPush whether a new push may start now (disabled, and not yet confirmed)
     */
    void updateInputs(FieldMapPushInputs inputs, boolean mayPush);
}
