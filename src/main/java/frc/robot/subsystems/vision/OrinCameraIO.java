package frc.robot.subsystems.vision;

/**
 * Hardware layer for one Orin camera: reads it into {@link OrinCameraInputs}, and carries the few
 * commands the robot sends it. Nothing else touches the {@code PhotonCamera}.
 */
public interface OrinCameraIO {
    /** Replay / detached: reports nothing and ignores commands. */
    OrinCameraIO NONE =
            new OrinCameraIO() {
                @Override
                public void updateInputs(OrinCameraInputs inputs) {}
            };

    /**
     * Fills {@code inputs} with everything reported since the last call.
     *
     * @param inputs the inputs to overwrite
     */
    void updateInputs(OrinCameraInputs inputs);

    /**
     * Asks the camera to run a pipeline (issue #10, 11c: 0 = event, 1 = practice field).
     *
     * @param index pipeline index
     */
    default void setPipelineIndex(int index) {}
}
