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

    /**
     * Results the camera sent this loop, including any not logged in {@link #results}: while the
     * robot is disabled only the newest result with targets is kept, at 10 Hz, to keep the log
     * small (the Orin feeds neither the pose nor the seed while disabled). Frame rate and the
     * no-frames alert use this count.
     */
    public int resultCount = 0;

    /** Results PhotonLib failed to read this loop (PhotonVision #2528); they were dropped. */
    public int readFailures = 0;

    /** Camera matrix, row-major 3x3; empty until the camera has connected. */
    public double[] cameraMatrix = EMPTY_DOUBLE;

    /** Distortion coefficients (the Jetson sends all 8); empty until connected. */
    public double[] distCoeffs = EMPTY_DOUBLE;

    // -- /photonvision/<camera>/health/ (section 9a), NaN / -1 until first published --

    /** Pipeline results per second; 0 after 2 s with none. */
    public double healthFps = Double.NaN;

    /** Average pipeline time over the last second. */
    public double healthPipelineMs = Double.NaN;

    /** Worst pipeline time over the last second. */
    public double healthPipelineMsMax = Double.NaN;

    /** Average time from capture to result. */
    public double healthLatencyMs = Double.NaN;

    /** Frames processed since PhotonVision started. */
    public long healthFrames = -1;

    /** Frames the JPEG decoder rejected since start; rising means corrupt frames get through. */
    public long healthDecodeFailures = -1;

    /** Reconnects, USB resets and bandwidth retries the stuck-camera watchdog tried. */
    public long healthRecoveries = -1;

    /**
     * Counts up each second from a timer that doesn't depend on results; stalled means the Jetson's
     * health publisher stopped.
     */
    public long healthHeartbeat = -1;

    /**
     * streaming, stalled (connected, no frame for 1 s), recovering, disconnected, gave-up, paused
     * (disabled by robot code) or starting.
     */
    public String healthState = "";

    /** Time since the last usable frame; -1 before the first. */
    public long healthLastFrameAgeMs = -1;

    /**
     * What the watchdog found wrong, e.g. gone from USB port, no frames after 4 USB resets; stays
     * until frames return.
     */
    public String healthProblem = "";

    // -- /photonvision/<camera>/mount/ (section 9b) --

    /** Camera height above the field, mean over the last 2 s of multi-tag frames. */
    public double mountHeightM = Double.NaN;

    /** Camera pitch on the field, mean; positive points down (WPILib). */
    public double mountPitchDeg = Double.NaN;

    /** Camera roll on the field, mean. */
    public double mountRollDeg = Double.NaN;

    /** Spread of the height over the window. */
    public double mountHeightStdM = Double.NaN;

    /** Spread of the pitch over the window. */
    public double mountPitchStdDeg = Double.NaN;

    /** Spread of the roll over the window. */
    public double mountRollStdDeg = Double.NaN;

    /** Camera position on the field, x. */
    public double mountFieldXM = Double.NaN;

    /** Camera position on the field, y. */
    public double mountFieldYM = Double.NaN;

    /** Camera heading on the field (circular mean). */
    public double mountFieldYawDeg = Double.NaN;

    /** Mean multi-tag reprojection error. */
    public double mountReprojErrorPx = Double.NaN;

    /** Multi-tag frames in the 2 s window; 0 means the other mount values are stale. */
    public long mountSamples = 0;

    /** {@code /photonvision/<camera>/settingsJson} (section 11a); written to the log on change. */
    public String settingsJson = "";

    /** Clears the per-loop fields before a read. */
    public void clearResults() {
        results = NO_RESULTS;
        resultCount = 0;
        readFailures = 0;
    }

    @Override
    public void toLog(LogTable table) {
        table.put("Connected", connected);
        table.put("Enabled", enabled);
        table.put("PipelineIndex", pipelineIndex);
        table.put("Results", results);
        table.put("ResultCount", resultCount);
        table.put("ReadFailures", readFailures);
        table.put("CameraMatrix", cameraMatrix);
        table.put("DistCoeffs", distCoeffs);
        table.put("Health/Fps", healthFps);
        table.put("Health/PipelineMs", healthPipelineMs, "milliseconds");
        table.put("Health/PipelineMsMax", healthPipelineMsMax);
        table.put("Health/LatencyMs", healthLatencyMs, "milliseconds");
        table.put("Health/Frames", healthFrames);
        table.put("Health/DecodeFailures", healthDecodeFailures);
        table.put("Health/Recoveries", healthRecoveries);
        table.put("Health/Heartbeat", healthHeartbeat);
        table.put("Health/State", healthState);
        table.put("Health/LastFrameAgeMs", healthLastFrameAgeMs, "milliseconds");
        table.put("Health/Problem", healthProblem);
        table.put("Mount/HeightM", mountHeightM);
        table.put("Mount/PitchDeg", mountPitchDeg, "degrees");
        table.put("Mount/RollDeg", mountRollDeg, "degrees");
        table.put("Mount/HeightStdM", mountHeightStdM);
        table.put("Mount/PitchStdDeg", mountPitchStdDeg, "degrees");
        table.put("Mount/RollStdDeg", mountRollStdDeg, "degrees");
        table.put("Mount/FieldXM", mountFieldXM);
        table.put("Mount/FieldYM", mountFieldYM);
        table.put("Mount/FieldYawDeg", mountFieldYawDeg, "degrees");
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
        // Logs from before ResultCount logged every result.
        resultCount = table.get("ResultCount", results.length);
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
        // Logs from before patch 81 have none: not reported.
        healthHeartbeat = table.get("Health/Heartbeat", -1);
        healthState = table.get("Health/State", "");
        healthLastFrameAgeMs = table.get("Health/LastFrameAgeMs", -1);
        healthProblem = table.get("Health/Problem", "");
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
