package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * Everything one Orin (Jetson + PhotonVision) camera reported since the last loop, as an
 * AdvantageKit input: the raw PhotonLib results plus the Jetson's per-camera topics.
 *
 * <p>The results are logged as PhotonLib's own serialized bytes ({@code
 * PhotonPipelineResult.photonStruct}), one entry per result, including results with no targets
 * (about 40 bytes; they show the frame rate and dropouts). {@link OrinSolver} turns them into pose
 * observations from these inputs alone, so replay re-runs the whole Orin pipeline: a log recorded
 * with placeholder mounts can be re-solved with measured ones.
 *
 * <p>Logged under {@code /Vision/Orin/<camera>/...}. Issue #10, sections 2, 7a, 9, 11a.
 */
public class OrinCameraInputs implements LoggableInputs {
    private static final byte[][] NO_RESULTS = new byte[0][];
    private static final double[] EMPTY_DOUBLE = new double[0];

    /** PhotonLib's {@code isConnected()}: the camera's heartbeat is on NetworkTables. */
    public boolean connected = false;

    /** PhotonLib's {@code getEnabled()}: false while turned off with {@code setEnabled(false)}. */
    public boolean enabled = true;

    /** The pipeline the camera reports running. */
    public int pipelineIndex = -1;

    /** Serialized {@code PhotonPipelineResult}s, oldest first. */
    public byte[][] results = NO_RESULTS;

    /** Results PhotonLib failed to read this loop (PhotonVision #2528); they were dropped. */
    public int readFailures = 0;

    /** Camera matrix, row-major 3x3; empty until the camera has connected. */
    public double[] cameraMatrix = EMPTY_DOUBLE;

    /** Distortion coefficients (the Jetson sends all 8); empty until connected. */
    public double[] distCoeffs = EMPTY_DOUBLE;

    // -- /photonvision/<camera>/health/ (section 9a), NaN / -1 until first published --

    public double healthFps = Double.NaN;
    public double healthPipelineMs = Double.NaN;
    public double healthPipelineMsMax = Double.NaN;
    public double healthLatencyMs = Double.NaN;
    public long healthFrames = -1;
    public long healthDecodeFailures = -1;
    public long healthRecoveries = -1;

    // -- /photonvision/<camera>/mount/ (section 9b) --

    public double mountHeightM = Double.NaN;
    public double mountPitchDeg = Double.NaN;
    public double mountRollDeg = Double.NaN;
    public double mountHeightStdM = Double.NaN;
    public double mountPitchStdDeg = Double.NaN;
    public double mountRollStdDeg = Double.NaN;
    public double mountFieldXM = Double.NaN;
    public double mountFieldYM = Double.NaN;
    public double mountFieldYawDeg = Double.NaN;
    public double mountReprojErrorPx = Double.NaN;
    public long mountSamples = 0;

    /** {@code /photonvision/<camera>/settingsJson} (section 11a); written to the log on change. */
    public String settingsJson = "";

    /** Clears the per-loop fields before a read. */
    public void clearResults() {
        results = NO_RESULTS;
        readFailures = 0;
    }

    @Override
    public void toLog(LogTable table) {
        table.put("Connected", connected);
        table.put("Enabled", enabled);
        table.put("PipelineIndex", pipelineIndex);
        table.put("Results", results);
        table.put("ReadFailures", readFailures);
        table.put("CameraMatrix", cameraMatrix);
        table.put("DistCoeffs", distCoeffs);
        table.put("Health/Fps", healthFps);
        table.put("Health/PipelineMs", healthPipelineMs);
        table.put("Health/PipelineMsMax", healthPipelineMsMax);
        table.put("Health/LatencyMs", healthLatencyMs);
        table.put("Health/Frames", healthFrames);
        table.put("Health/DecodeFailures", healthDecodeFailures);
        table.put("Health/Recoveries", healthRecoveries);
        table.put("Mount/HeightM", mountHeightM);
        table.put("Mount/PitchDeg", mountPitchDeg);
        table.put("Mount/RollDeg", mountRollDeg);
        table.put("Mount/HeightStdM", mountHeightStdM);
        table.put("Mount/PitchStdDeg", mountPitchStdDeg);
        table.put("Mount/RollStdDeg", mountRollStdDeg);
        table.put("Mount/FieldXM", mountFieldXM);
        table.put("Mount/FieldYM", mountFieldYM);
        table.put("Mount/FieldYawDeg", mountFieldYawDeg);
        table.put("Mount/ReprojErrorPx", mountReprojErrorPx);
        table.put("Mount/Samples", mountSamples);
        table.put("SettingsJson", settingsJson);
    }

    @Override
    public void fromLog(LogTable table) {
        connected = table.get("Connected", connected);
        enabled = table.get("Enabled", enabled);
        pipelineIndex = table.get("PipelineIndex", pipelineIndex);
        results = table.get("Results", NO_RESULTS);
        readFailures = table.get("ReadFailures", 0);
        cameraMatrix = table.get("CameraMatrix", cameraMatrix);
        distCoeffs = table.get("DistCoeffs", distCoeffs);
        healthFps = table.get("Health/Fps", healthFps);
        healthPipelineMs = table.get("Health/PipelineMs", healthPipelineMs);
        healthPipelineMsMax = table.get("Health/PipelineMsMax", healthPipelineMsMax);
        healthLatencyMs = table.get("Health/LatencyMs", healthLatencyMs);
        healthFrames = table.get("Health/Frames", healthFrames);
        healthDecodeFailures = table.get("Health/DecodeFailures", healthDecodeFailures);
        healthRecoveries = table.get("Health/Recoveries", healthRecoveries);
        mountHeightM = table.get("Mount/HeightM", mountHeightM);
        mountPitchDeg = table.get("Mount/PitchDeg", mountPitchDeg);
        mountRollDeg = table.get("Mount/RollDeg", mountRollDeg);
        mountHeightStdM = table.get("Mount/HeightStdM", mountHeightStdM);
        mountPitchStdDeg = table.get("Mount/PitchStdDeg", mountPitchStdDeg);
        mountRollStdDeg = table.get("Mount/RollStdDeg", mountRollStdDeg);
        mountFieldXM = table.get("Mount/FieldXM", mountFieldXM);
        mountFieldYM = table.get("Mount/FieldYM", mountFieldYM);
        mountFieldYawDeg = table.get("Mount/FieldYawDeg", mountFieldYawDeg);
        mountReprojErrorPx = table.get("Mount/ReprojErrorPx", mountReprojErrorPx);
        mountSamples = table.get("Mount/Samples", mountSamples);
        settingsJson = table.get("SettingsJson", settingsJson);
    }
}
