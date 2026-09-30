package frc.robot.subsystems.vision;

/**
 * Hardware layer for the Jetson Orin itself (not one camera): its health and Rewind topics in, and
 * the robot's commands to it out. Everything that talks to NetworkTables for the Jetson is here, so
 * replay swaps it for {@link #NONE} and stays clean.
 */
public interface JetsonIO {
    /** Replay / detached: reports nothing and ignores commands. */
    JetsonIO NONE =
            new JetsonIO() {
                @Override
                public void updateInputs(JetsonInputs inputs) {}
            };

    /**
     * Fills {@code inputs} with the Jetson's latest values.
     *
     * @param inputs the inputs to overwrite
     */
    void updateInputs(JetsonInputs inputs);

    /**
     * Sets Rewind's recording name and whether it records. The label is published first: the Jetson
     * reads it when a recording starts (issue #10, section 4).
     *
     * @param record record while true
     * @param label recording name; empty lets the Jetson name it
     */
    default void setRewind(boolean record, String label) {}

    /**
     * Publishes the robot's wall clock ({@code /photonvision/clock/unixMs}) if it has been set, so
     * the Jetson, which has no clock battery, gets the date (section 6). Reads the clock itself so
     * no wall-clock time reaches robot logic.
     */
    default void publishRobotClock() {}

    /**
     * Publishes the tags every camera's multi-tag solve should ignore ({@code
     * /photonvision/excludedTags}, sections 11b and 12).
     *
     * @param tagIds tag ids to exclude
     */
    default void setExcludedTags(long[] tagIds) {}
}
