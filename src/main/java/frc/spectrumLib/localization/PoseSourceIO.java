package frc.spectrumLib.localization;

/**
 * Hardware layer for one pose source, AdvantageKit style.
 *
 * <p>An implementation reads the device (NetworkTables for a Limelight or QuestNav, PhotonLib for
 * the Orin) and fills {@link PoseSourceInputs}. Nothing downstream touches the device: gating and
 * fusion read only the inputs, so in replay, where the IO is swapped for {@link #NONE}, the same
 * code runs against the logged values.
 */
public interface PoseSourceIO {
    /** Replay / detached: reports nothing, so the logged inputs are what the code sees. */
    PoseSourceIO NONE = inputs -> {};

    /**
     * Fills {@code inputs} with everything reported since the last call.
     *
     * @param inputs the inputs to overwrite
     */
    void updateInputs(PoseSourceInputs inputs);
}
