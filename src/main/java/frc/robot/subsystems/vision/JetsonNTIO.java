package frc.robot.subsystems.vision;

import org.wpilib.networktables.BooleanPublisher;
import org.wpilib.networktables.BooleanSubscriber;
import org.wpilib.networktables.GenericSubscriber;
import org.wpilib.networktables.IntegerArrayPublisher;
import org.wpilib.networktables.IntegerArraySubscriber;
import org.wpilib.networktables.IntegerPublisher;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.NetworkTableValue;
import org.wpilib.networktables.StringPublisher;
import org.wpilib.networktables.StringSubscriber;

/**
 * The Jetson's NetworkTables topics under {@code /photonvision} (SpectrumJetson docs/TECHNICAL.md
 * and docs/REWIND.md). Numbers are read generically because the Jetson publishes some as double and
 * some as integer.
 */
public class JetsonNTIO implements JetsonIO {
    /** The Jetson's hostname, the last part of its metrics topic. */
    private static final String METRICS_HOST = "photonvision-3847";

    /** 2026-01-01 UTC: before this the robot's clock has not been set by the Driver Station. */
    private static final long CLOCK_VALID_AFTER_MS = 1_767_225_600_000L;

    /** 2100-01-01 UTC: the Jetson ignores anything later, so don't send it. */
    private static final long CLOCK_VALID_BEFORE_MS = 4_102_444_800_000L;

    private final GenericSubscriber heartbeat;
    private final GenericSubscriber gpuLoadPct;
    private final GenericSubscriber cpuTempC;
    private final GenericSubscriber gpuTempC;
    private final GenericSubscriber socTempC;
    private final GenericSubscriber tjTempC;
    private final GenericSubscriber fanRpm;
    private final GenericSubscriber powerW;
    private final GenericSubscriber cpuGpuPowerW;
    private final GenericSubscriber socPowerW;
    private final StringSubscriber jpegDecoder;
    private final BooleanSubscriber jpegHardwareOff;
    private final GenericSubscriber jpegChecksOk;
    private final GenericSubscriber jpegChecksDiffer;
    private final StringSubscriber throttle;
    private final GenericSubscriber overCurrentEvents;
    private final StringSubscriber settingsJson;
    private final IntegerArraySubscriber excludedTagsActive;

    private final BooleanSubscriber recording;
    private final StringSubscriber session;
    private final GenericSubscriber freeGB;
    private final GenericSubscriber framesDropped;

    /**
     * PhotonVision's metrics protobuf. It builds the name as {@code getSubTable("/metrics")} under
     * {@code /photonvision}, hence the double slash; the single-slash form is watched too in case
     * that changes. CHECK the host part on the robot network.
     */
    private final GenericSubscriber[] metrics;

    private final BooleanPublisher record;
    private final StringPublisher label;
    private final IntegerPublisher clock;
    private final IntegerArrayPublisher excludedTags;

    public JetsonNTIO() {
        NetworkTable pv = NetworkTableInstance.getDefault().getTable("photonvision");
        NetworkTable jetson = pv.getSubTable("jetson");
        heartbeat = number(jetson, "heartbeat");
        gpuLoadPct = number(jetson, "gpuLoadPct");
        cpuTempC = number(jetson, "cpuTempC");
        gpuTempC = number(jetson, "gpuTempC");
        socTempC = number(jetson, "socTempC");
        tjTempC = number(jetson, "tjTempC");
        fanRpm = number(jetson, "fanRpm");
        powerW = number(jetson, "powerW");
        cpuGpuPowerW = number(jetson, "cpuGpuPowerW");
        socPowerW = number(jetson, "socPowerW");
        jpegDecoder = jetson.getStringTopic("jpegDecoder").subscribe("");
        jpegHardwareOff = jetson.getBooleanTopic("jpegHardwareOff").subscribe(false);
        jpegChecksOk = number(jetson, "jpegChecksOk");
        jpegChecksDiffer = number(jetson, "jpegChecksDiffer");
        throttle = jetson.getStringTopic("throttle").subscribe("");
        overCurrentEvents = number(jetson, "overCurrentEvents");
        settingsJson = jetson.getStringTopic("settingsJson").subscribe("");
        excludedTagsActive = pv.getIntegerArrayTopic("excludedTagsActive").subscribe(new long[0]);

        NetworkTable rewind = pv.getSubTable("rewind");
        recording = rewind.getBooleanTopic("recording").subscribe(false);
        session = rewind.getStringTopic("session").subscribe("");
        freeGB = number(rewind, "freeGB");
        framesDropped = number(rewind, "framesDropped");
        record = rewind.getBooleanTopic("record").publish();
        label = rewind.getStringTopic("label").publish();

        NetworkTableInstance nt = NetworkTableInstance.getDefault();
        metrics =
                new GenericSubscriber[] {
                    nt.getTopic("/photonvision//metrics/" + METRICS_HOST).genericSubscribe(),
                    nt.getTopic("/photonvision/metrics/" + METRICS_HOST).genericSubscribe()
                };

        clock = pv.getSubTable("clock").getIntegerTopic("unixMs").publish();
        excludedTags = pv.getIntegerArrayTopic("excludedTags").publish();
    }

    private static GenericSubscriber number(NetworkTable table, String name) {
        return table.getTopic(name).genericSubscribe();
    }

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

    @Override
    public void updateInputs(JetsonInputs inputs) {
        inputs.heartbeat = (long) read(heartbeat, -1);
        inputs.gpuLoadPct = read(gpuLoadPct, Double.NaN);
        inputs.cpuTempC = read(cpuTempC, Double.NaN);
        inputs.gpuTempC = read(gpuTempC, Double.NaN);
        inputs.socTempC = read(socTempC, Double.NaN);
        inputs.tjTempC = read(tjTempC, Double.NaN);
        inputs.fanRpm = read(fanRpm, Double.NaN);
        inputs.powerW = read(powerW, Double.NaN);
        inputs.cpuGpuPowerW = read(cpuGpuPowerW, Double.NaN);
        inputs.socPowerW = read(socPowerW, Double.NaN);
        inputs.jpegDecoder = jpegDecoder.get();
        inputs.jpegHardwareOff = jpegHardwareOff.get();
        inputs.jpegChecksOk = (long) read(jpegChecksOk, -1);
        inputs.jpegChecksDiffer = (long) read(jpegChecksDiffer, -1);
        inputs.throttle = throttle.get();
        inputs.overCurrentEvents = (long) read(overCurrentEvents, -1);
        inputs.settingsJson = settingsJson.get();
        inputs.excludedTagsActive = excludedTagsActive.get();
        readMetrics(inputs);
        inputs.rewindRecording = recording.get();
        inputs.rewindSession = session.get();
        inputs.rewindFreeGB = read(freeGB, Double.NaN);
        inputs.rewindFramesDropped = (long) read(framesDropped, -1);
    }

    private long lastMetricsTime = 0;

    /** Parses the newest metrics message, if one arrived since the last loop. */
    private void readMetrics(JetsonInputs inputs) {
        for (GenericSubscriber sub : metrics) {
            NetworkTableValue v = sub.get();
            if (!v.isRaw() || v.getTime() == lastMetricsTime) {
                continue;
            }
            lastMetricsTime = v.getTime();
            try {
                var m = org.photonvision.proto.Photon.ProtobufDeviceMetrics.parseFrom(v.getRaw());
                inputs.uptimeSeconds = m.getUptime();
                inputs.cpuUtilPct = m.getCpuUtil();
                inputs.ramUtilPct = m.getRamUtil();
                inputs.diskUsableSpace = m.getDiskUsableSpace();
            } catch (Exception e) {
                // A malformed message: keep the last values.
            }
        }
    }

    @Override
    public void setRewind(boolean record, String label) {
        this.label.set(label);
        this.record.set(record);
    }

    private long lastClockPublishMs = 0;

    /**
     * At 1 Hz: the value changes every millisecond, so NetworkTables would send every call, and the
     * Jetson only needs it fresher than 5 s.
     */
    @Override
    public void publishRobotClock() {
        long now = System.currentTimeMillis();
        if (now > CLOCK_VALID_AFTER_MS
                && now < CLOCK_VALID_BEFORE_MS
                && now - lastClockPublishMs >= 1000) {
            lastClockPublishMs = now;
            clock.set(now);
        }
    }

    @Override
    public void setExcludedTags(long[] tagIds) {
        excludedTags.set(tagIds);
    }
}
