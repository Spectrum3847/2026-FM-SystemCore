package frc.robot.subsystems.vision;

import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.common.dataflow.structures.Packet;
import org.photonvision.targeting.PhotonPipelineResult;
import org.wpilib.networktables.GenericSubscriber;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableValue;
import org.wpilib.networktables.StringSubscriber;

/**
 * One PhotonVision camera on the Jetson Orin, read with PhotonLib alpha-2 (the robot must stay on
 * alpha-2: it is the Jetson's wire format).
 *
 * <p>Reads only; the pose solve is in {@link OrinSolver}, which runs in replay too. Besides the
 * results it reads the Jetson's per-camera topics (SpectrumJetson patches 16, 17, 22-29): {@code
 * health/*}, {@code mount/*} and {@code settingsJson}, all under {@code /photonvision/<camera>/}.
 * The Jetson publishes numbers as double or integer depending on the topic, so they are read
 * generically.
 */
public class PhotonOrinIO implements OrinCameraIO {
    private final PhotonCamera camera;
    private final Packet packet = new Packet(1024);

    private final GenericSubscriber fps;
    private final GenericSubscriber pipelineMs;
    private final GenericSubscriber pipelineMsMax;
    private final GenericSubscriber latencyMs;
    private final GenericSubscriber frames;
    private final GenericSubscriber decodeFailures;
    private final GenericSubscriber recoveries;

    private final GenericSubscriber heightM;
    private final GenericSubscriber pitchDeg;
    private final GenericSubscriber rollDeg;
    private final GenericSubscriber heightStdM;
    private final GenericSubscriber pitchStdDeg;
    private final GenericSubscriber rollStdDeg;
    private final GenericSubscriber fieldXM;
    private final GenericSubscriber fieldYM;
    private final GenericSubscriber fieldYawDeg;
    private final GenericSubscriber reprojErrorPx;
    private final GenericSubscriber samples;

    private final StringSubscriber settingsJson;

    /**
     * @param cameraName the camera's name in PhotonVision (the Jetson names them after the USB
     *     port: TopLeft, TopRight, BottomLeft, BottomRight)
     */
    public PhotonOrinIO(String cameraName) {
        camera = new PhotonCamera(cameraName);
        NetworkTable table = camera.getCameraTable();
        NetworkTable health = table.getSubTable("health");
        fps = number(health, "fps");
        pipelineMs = number(health, "pipelineMs");
        pipelineMsMax = number(health, "pipelineMsMax");
        latencyMs = number(health, "latencyMs");
        frames = number(health, "frames");
        decodeFailures = number(health, "decodeFailures");
        recoveries = number(health, "recoveries");
        NetworkTable mount = table.getSubTable("mount");
        heightM = number(mount, "heightM");
        pitchDeg = number(mount, "pitchDeg");
        rollDeg = number(mount, "rollDeg");
        heightStdM = number(mount, "heightStdM");
        pitchStdDeg = number(mount, "pitchStdDeg");
        rollStdDeg = number(mount, "rollStdDeg");
        fieldXM = number(mount, "fieldXM");
        fieldYM = number(mount, "fieldYM");
        fieldYawDeg = number(mount, "fieldYawDeg");
        reprojErrorPx = number(mount, "reprojErrorPx");
        samples = number(mount, "samples");
        settingsJson = table.getStringTopic("settingsJson").subscribe("");
    }

    private static GenericSubscriber number(NetworkTable table, String name) {
        return table.getTopic(name).genericSubscribe();
    }

    /** A number topic's value whether published as double, float or integer; else {@code dflt}. */
    private static double read(GenericSubscriber sub, double dflt) {
        NetworkTableValue v = sub.get();
        if (v.isDouble()) {
            return v.getDouble();
        } else if (v.isInteger()) {
            return v.getInteger();
        } else if (v.isFloat()) {
            return v.getFloat();
        }
        return dflt;
    }

    /** While disabled, at most one result is logged per this long (seconds). */
    private static final double DISABLED_RESULT_PERIOD = 0.1;

    private double lastDisabledResultSeconds = Double.NEGATIVE_INFINITY;
    private double lastSlowReadSeconds = Double.NEGATIVE_INFINITY;
    private double lastCalibrationTrySeconds = Double.NEGATIVE_INFINITY;

    @Override
    public void updateInputs(OrinCameraInputs inputs) {
        inputs.clearResults();
        double now = org.wpilib.system.Timer.getTimestamp();
        inputs.connected = camera.isConnected();
        inputs.enabled = camera.getEnabled();
        inputs.pipelineIndex = camera.getPipelineIndex();

        // PhotonLib decodes inside getAllUnreadResults(); alpha-2's decoder can throw on a
        // truncated packet (PhotonVision #2528), which would lose that loop's batch. Count it and
        // carry on, so one bad packet cannot stop the vision loop.
        List<PhotonPipelineResult> unread;
        try {
            unread = camera.getAllUnreadResults();
        } catch (RuntimeException e) {
            unread = List.of();
            inputs.readFailures++;
        }
        inputs.resultCount = unread.size();
        if (org.wpilib.driverstation.RobotState.isDisabled()) {
            unread = newestWithTargets(unread, now);
        }
        byte[][] out = new byte[unread.size()][];
        int n = 0;
        for (PhotonPipelineResult r : unread) {
            try {
                // The solver uses the detected corners only; the min-area-rect corners are about a
                // fifth of each result's bytes.
                for (var t : r.targets) {
                    t.minAreaRectCorners = List.of();
                }
                packet.clear();
                PhotonPipelineResult.photonStruct.pack(packet, r);
                out[n++] = packet.getWrittenDataCopy();
            } catch (RuntimeException e) {
                inputs.readFailures++;
            }
        }
        inputs.results = n == out.length ? out : java.util.Arrays.copyOf(out, n);

        if ((inputs.cameraMatrix.length == 0 || inputs.distCoeffs.length == 0)
                && now - lastCalibrationTrySeconds >= 1.0) {
            lastCalibrationTrySeconds = now;
            camera.getCameraMatrix().ifPresent(m -> inputs.cameraMatrix = m.getData());
            camera.getDistCoeffs().ifPresent(m -> inputs.distCoeffs = m.getData());
        }

        // The Jetson publishes health once a second and the mount estimate every 0.5 s: 10 Hz
        // reads catch every update without ~18 NetworkTables calls per camera every loop.
        if (now - lastSlowReadSeconds < 0.1) {
            return;
        }
        lastSlowReadSeconds = now;

        inputs.healthFps = read(fps, Double.NaN);
        inputs.healthPipelineMs = read(pipelineMs, Double.NaN);
        inputs.healthPipelineMsMax = read(pipelineMsMax, Double.NaN);
        inputs.healthLatencyMs = read(latencyMs, Double.NaN);
        inputs.healthFrames = (long) read(frames, -1);
        inputs.healthDecodeFailures = (long) read(decodeFailures, -1);
        inputs.healthRecoveries = (long) read(recoveries, -1);

        inputs.mountHeightM = read(heightM, Double.NaN);
        inputs.mountPitchDeg = read(pitchDeg, Double.NaN);
        inputs.mountRollDeg = read(rollDeg, Double.NaN);
        inputs.mountHeightStdM = read(heightStdM, Double.NaN);
        inputs.mountPitchStdDeg = read(pitchStdDeg, Double.NaN);
        inputs.mountRollStdDeg = read(rollStdDeg, Double.NaN);
        inputs.mountFieldXM = read(fieldXM, Double.NaN);
        inputs.mountFieldYM = read(fieldYM, Double.NaN);
        inputs.mountFieldYawDeg = read(fieldYawDeg, Double.NaN);
        inputs.mountReprojErrorPx = read(reprojErrorPx, Double.NaN);
        inputs.mountSamples = (long) read(samples, 0);

        inputs.settingsJson = settingsJson.get();
    }

    /**
     * While disabled: the newest result with targets, at most every {@link
     * #DISABLED_RESULT_PERIOD}.
     */
    private List<PhotonPipelineResult> newestWithTargets(
            List<PhotonPipelineResult> unread, double now) {
        if (now - lastDisabledResultSeconds < DISABLED_RESULT_PERIOD) {
            return List.of();
        }
        for (int i = unread.size() - 1; i >= 0; i--) {
            if (unread.get(i).hasTargets()) {
                lastDisabledResultSeconds = now;
                return List.of(unread.get(i));
            }
        }
        return List.of();
    }

    @Override
    public void setPipelineIndex(int index) {
        camera.setPipelineIndex(index);
    }
}
