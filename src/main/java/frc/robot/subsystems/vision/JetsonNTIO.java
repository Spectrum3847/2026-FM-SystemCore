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
import org.wpilib.networktables.PubSubOption;
import org.wpilib.networktables.StringPublisher;
import org.wpilib.networktables.StringSubscriber;

/**
 * The Jetson's NetworkTables topics under {@code /photonvision} (SpectrumJetson docs/TECHNICAL.md
 * and docs/REWIND.md). Numbers are read generically because the Jetson publishes some as double and
 * some as integer.
 */
public class JetsonNTIO implements JetsonIO {
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
    private final GenericSubscriber inputVoltageVSub;
    private final GenericSubscriber inputVoltageMinVSub;
    private final GenericSubscriber inputVoltageLowestVSub;
    private final GenericSubscriber usbEventsSub;
    private final StringSubscriber lastUsbEventSub;
    private final StringSubscriber ethInterfaceSub;
    private final BooleanSubscriber ethLinkUpSub;
    private final GenericSubscriber ethSpeedMbpsSub;
    private final GenericSubscriber ethCarrierChangesSub;
    private final GenericSubscriber ntDisconnectsSub;
    private final StringSubscriber bootIdSub;
    private final GenericSubscriber uptimeSecondsSub;
    private final StringSubscriber clockSourceSub;
    private final GenericSubscriber clockLastJumpSecondsSub;
    private final GenericSubscriber cpuUtilPctSub;
    private final GenericSubscriber ramUsedPctSub;
    private final GenericSubscriber ssdTempCSub;
    private final GenericSubscriber fsErrorsSub;
    private final BooleanSubscriber settingsReadOnlySub;
    private final BooleanSubscriber scratchReadOnlySub;
    private final GenericSubscriber usbBandwidthUsedBytesSub;
    private final GenericSubscriber usbBandwidthBudgetBytesSub;
    private final GenericSubscriber ssdMediaErrorsSub;
    private final GenericSubscriber ssdUnsafeShutdownsSub;
    private final GenericSubscriber ssdPercentUsedSub;
    private final GenericSubscriber ssdCriticalWarningSub;
    private final GenericSubscriber ssdAvailableSparePctSub;

    private final BooleanSubscriber recording;
    private final StringSubscriber session;
    private final GenericSubscriber freeGB;
    private final GenericSubscriber framesDropped;

    private final StringSubscriber fieldMapHash;
    private final StringSubscriber fieldMapName;
    private final GenericSubscriber fieldMapLoadedAt;
    private final StringSubscriber fieldMapError;
    private final StringSubscriber logLines;

    private final NetworkTableInstance nt = NetworkTableInstance.getDefault();

    private final BooleanPublisher record;
    private final StringPublisher label;
    private final IntegerPublisher clock;
    private final IntegerArrayPublisher excludedTags;
    private final StringPublisher robotFieldMapJson;
    private final StringPublisher robotFieldMapHash;
    private final StringPublisher robotFieldMapName;

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
        inputVoltageVSub = number(jetson, "inputVoltageV");
        inputVoltageMinVSub = number(jetson, "inputVoltageMinV");
        inputVoltageLowestVSub = number(jetson, "inputVoltageLowestV");
        usbEventsSub = number(jetson, "usbEvents");
        lastUsbEventSub = jetson.getStringTopic("lastUsbEvent").subscribe("");
        ethInterfaceSub = jetson.getStringTopic("ethInterface").subscribe("");
        ethLinkUpSub = jetson.getBooleanTopic("ethLinkUp").subscribe(false);
        ethSpeedMbpsSub = number(jetson, "ethSpeedMbps");
        ethCarrierChangesSub = number(jetson, "ethCarrierChanges");
        ntDisconnectsSub = number(jetson, "ntDisconnects");
        bootIdSub = jetson.getStringTopic("bootId").subscribe("");
        uptimeSecondsSub = number(jetson, "uptimeSeconds");
        clockSourceSub = jetson.getStringTopic("clockSource").subscribe("");
        clockLastJumpSecondsSub = number(jetson, "clockLastJumpSeconds");
        cpuUtilPctSub = number(jetson, "cpuUtilPct");
        ramUsedPctSub = number(jetson, "ramUsedPct");
        ssdTempCSub = number(jetson, "ssdTempC");
        fsErrorsSub = number(jetson, "fsErrors");
        settingsReadOnlySub = jetson.getBooleanTopic("settingsReadOnly").subscribe(false);
        scratchReadOnlySub = jetson.getBooleanTopic("scratchReadOnly").subscribe(false);
        usbBandwidthUsedBytesSub = number(jetson, "usbBandwidthUsedBytes");
        usbBandwidthBudgetBytesSub = number(jetson, "usbBandwidthBudgetBytes");
        ssdMediaErrorsSub = number(jetson, "ssdMediaErrors");
        ssdUnsafeShutdownsSub = number(jetson, "ssdUnsafeShutdowns");
        ssdPercentUsedSub = number(jetson, "ssdPercentUsed");
        ssdCriticalWarningSub = number(jetson, "ssdCriticalWarning");
        ssdAvailableSparePctSub = number(jetson, "ssdAvailableSparePct");
        excludedTagsActive = pv.getIntegerArrayTopic("excludedTagsActive").subscribe(new long[0]);
        NetworkTable jetsonMap = jetson.getSubTable("FieldMap");
        fieldMapHash = jetsonMap.getStringTopic("Hash").subscribe("");
        fieldMapName = jetsonMap.getStringTopic("Name").subscribe("");
        fieldMapLoadedAt = number(jetsonMap, "LoadedAt");
        fieldMapError = jetsonMap.getStringTopic("Error").subscribe("");
        // Every line, in order: the Jetson sends at most 20 a second, read every cycle.
        logLines =
                jetson.getStringTopic("log")
                        .subscribe(
                                "",
                                PubSubOption.SEND_ALL,
                                PubSubOption.KEEP_DUPLICATES,
                                PubSubOption.pollStorage(100));

        NetworkTable rewind = pv.getSubTable("rewind");
        recording = rewind.getBooleanTopic("recording").subscribe(false);
        session = rewind.getStringTopic("session").subscribe("");
        freeGB = number(rewind, "freeGB");
        framesDropped = number(rewind, "framesDropped");
        record = rewind.getBooleanTopic("record").publish();
        label = rewind.getStringTopic("label").publish();

        clock = pv.getSubTable("clock").getIntegerTopic("unixMs").publish();
        excludedTags = pv.getIntegerArrayTopic("excludedTags").publish();

        NetworkTable robotMap =
                NetworkTableInstance.getDefault()
                        .getTable("SpectrumVision")
                        .getSubTable("FieldMap");
        robotFieldMapJson = robotMap.getStringTopic("Json").publish();
        robotFieldMapHash = robotMap.getStringTopic("Hash").publish();
        robotFieldMapName = robotMap.getStringTopic("Name").publish();
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
        inputs.inputVoltageV = read(inputVoltageVSub, Double.NaN);
        inputs.inputVoltageMinV = read(inputVoltageMinVSub, Double.NaN);
        inputs.inputVoltageLowestV = read(inputVoltageLowestVSub, Double.NaN);
        inputs.usbEvents = (long) read(usbEventsSub, -1);
        inputs.lastUsbEvent = lastUsbEventSub.get();
        inputs.ethInterface = ethInterfaceSub.get();
        inputs.ethLinkUp = ethLinkUpSub.get();
        inputs.ethSpeedMbps = (long) read(ethSpeedMbpsSub, -1);
        inputs.ethCarrierChanges = (long) read(ethCarrierChangesSub, -1);
        inputs.ntDisconnects = (long) read(ntDisconnectsSub, -1);
        inputs.bootId = bootIdSub.get();
        inputs.uptimeSeconds = read(uptimeSecondsSub, Double.NaN);
        inputs.clockSource = clockSourceSub.get();
        inputs.clockLastJumpSeconds = read(clockLastJumpSecondsSub, Double.NaN);
        inputs.cpuUtilPct = read(cpuUtilPctSub, Double.NaN);
        inputs.ramUsedPct = read(ramUsedPctSub, Double.NaN);
        inputs.ssdTempC = read(ssdTempCSub, Double.NaN);
        inputs.fsErrors = (long) read(fsErrorsSub, -1);
        inputs.settingsReadOnly = settingsReadOnlySub.get();
        inputs.scratchReadOnly = scratchReadOnlySub.get();
        inputs.usbBandwidthUsedBytes = (long) read(usbBandwidthUsedBytesSub, -1);
        inputs.usbBandwidthBudgetBytes = (long) read(usbBandwidthBudgetBytesSub, -1);
        inputs.ssdMediaErrors = (long) read(ssdMediaErrorsSub, -1);
        inputs.ssdUnsafeShutdowns = (long) read(ssdUnsafeShutdownsSub, -1);
        inputs.ssdPercentUsed = (long) read(ssdPercentUsedSub, -1);
        inputs.ssdCriticalWarning = (long) read(ssdCriticalWarningSub, -1);
        inputs.ssdAvailableSparePct = (long) read(ssdAvailableSparePctSub, -1);
        inputs.rewindRecording = recording.get();
        inputs.rewindSession = session.get();
        inputs.rewindFreeGB = read(freeGB, Double.NaN);
        inputs.rewindFramesDropped = (long) read(framesDropped, -1);
        inputs.fieldMapHash = fieldMapHash.get();
        inputs.fieldMapName = fieldMapName.get();
        inputs.fieldMapLoadedAt = read(fieldMapLoadedAt, Double.NaN);
        inputs.fieldMapError = fieldMapError.get();
        inputs.logLines = logLines.readQueueValues();
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

    @Override
    public void publishFieldMap(String name, String hash, String json) {
        robotFieldMapJson.set(json);
        robotFieldMapName.set(name);
        robotFieldMapHash.set(hash);
    }
}
