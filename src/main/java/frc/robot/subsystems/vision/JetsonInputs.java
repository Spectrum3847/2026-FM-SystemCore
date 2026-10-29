package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * The Jetson Orin's own topics, as an AdvantageKit input: board health (issue #10, 9a and the
 * throttle addition), Rewind status (4, 7a), the Jetson's settings (11a) and the active
 * excluded-tag list (11b). Logged under {@code /Vision/Jetson/...}. NaN / -1 / empty until first
 * published.
 */
public class JetsonInputs implements LoggableInputs {
    private static final long[] NO_TAGS = new long[0];

    // -- /photonvision/jetson/ --
    /** +1 each second while PhotonVision runs. */
    public long heartbeat = -1;

    public double gpuLoadPct = Double.NaN;
    public double cpuTempC = Double.NaN;
    public double gpuTempC = Double.NaN;
    public double socTempC = Double.NaN;
    public double tjTempC = Double.NaN;
    public double fanRpm = Double.NaN;
    public double powerW = Double.NaN;
    public double cpuGpuPowerW = Double.NaN;
    public double socPowerW = Double.NaN;
    public String jpegDecoder = "";
    public boolean jpegHardwareOff = false;
    public long jpegChecksOk = -1;
    public long jpegChecksDiffer = -1;

    /** Why the Jetson is slowing itself down: None, OVER-CURRENT, HIGH TEMP (...), ... */
    public String throttle = "";

    public long overCurrentEvents = -1;
    public String settingsJson = "";

    // -- /photonvision/jetson/, JetsonDiagnostics (SpectrumJetson patch 82) --
    public double inputVoltageV = Double.NaN;
    public double inputVoltageMinV = Double.NaN;
    public double inputVoltageLowestV = Double.NaN;
    public long usbEvents = -1;
    public String lastUsbEvent = "";
    public String ethInterface = "";
    public boolean ethLinkUp = false;
    public long ethSpeedMbps = -1;
    public long ethCarrierChanges = -1;
    public long ntDisconnects = -1;
    public String bootId = "";

    /** The Jetson's {@code /proc/uptime}; {@code bootId} changes on a reboot. */
    public double uptimeSeconds = Double.NaN;

    public String clockSource = "";
    public double clockLastJumpSeconds = Double.NaN;
    public double cpuUtilPct = Double.NaN;
    public double ramUsedPct = Double.NaN;
    public double ssdTempC = Double.NaN;
    public long fsErrors = -1;
    public boolean settingsReadOnly = false;
    public boolean scratchReadOnly = false;
    public long usbBandwidthUsedBytes = -1;
    public long usbBandwidthBudgetBytes = -1;
    public long ssdMediaErrors = -1;
    public long ssdUnsafeShutdowns = -1;
    public long ssdPercentUsed = -1;
    public long ssdCriticalWarning = -1;
    public long ssdAvailableSparePct = -1;

    // -- /photonvision/ --
    /** The Jetson's saved excluded tags plus the robot's ({@code excludedTagsActive}). */
    public long[] excludedTagsActive = NO_TAGS;

    // -- /photonvision/rewind/ --
    public boolean rewindRecording = false;
    public String rewindSession = "";
    public double rewindFreeGB = Double.NaN;
    public long rewindFramesDropped = -1;

    // -- /photonvision/jetson/FieldMap/ --
    /** Canonical hash of the field map the Jetson loaded and solves with; empty until reported. */
    public String fieldMapHash = "";

    public String fieldMapName = "";

    /** When the Jetson loaded it, Jetson unix seconds. */
    public double fieldMapLoadedAt = Double.NaN;

    /** Why the Jetson could not load the robot's map, or empty. */
    public String fieldMapError = "";

    @Override
    public void toLog(LogTable table) {
        table.put("Heartbeat", heartbeat);
        table.put("GpuLoadPct", gpuLoadPct, "percent");
        table.put("CpuTempC", cpuTempC, "celsius");
        table.put("GpuTempC", gpuTempC, "celsius");
        table.put("SocTempC", socTempC, "celsius");
        table.put("TjTempC", tjTempC, "celsius");
        table.put("FanRpm", fanRpm, "RPM");
        table.put("PowerW", powerW, "watts");
        table.put("CpuGpuPowerW", cpuGpuPowerW, "watts");
        table.put("SocPowerW", socPowerW, "watts");
        table.put("JpegDecoder", jpegDecoder);
        table.put("JpegHardwareOff", jpegHardwareOff);
        table.put("JpegChecksOk", jpegChecksOk);
        table.put("JpegChecksDiffer", jpegChecksDiffer);
        table.put("Throttle", throttle);
        table.put("OverCurrentEvents", overCurrentEvents);
        table.put("SettingsJson", settingsJson);
        table.put("InputVoltageV", inputVoltageV, "volts");
        table.put("InputVoltageMinV", inputVoltageMinV, "volts");
        table.put("InputVoltageLowestV", inputVoltageLowestV, "volts");
        table.put("UsbEvents", usbEvents);
        table.put("LastUsbEvent", lastUsbEvent);
        table.put("Eth/Interface", ethInterface);
        table.put("Eth/LinkUp", ethLinkUp);
        table.put("Eth/SpeedMbps", ethSpeedMbps);
        table.put("Eth/CarrierChanges", ethCarrierChanges);
        table.put("NtDisconnects", ntDisconnects);
        table.put("BootId", bootId);
        table.put("UptimeSeconds", uptimeSeconds, "seconds");
        table.put("ClockSource", clockSource);
        table.put("ClockLastJumpSeconds", clockLastJumpSeconds, "seconds");
        table.put("CpuUtilPct", cpuUtilPct, "percent");
        table.put("RamUsedPct", ramUsedPct, "percent");
        table.put("Ssd/TempC", ssdTempC, "celsius");
        table.put("FsErrors", fsErrors);
        table.put("SettingsReadOnly", settingsReadOnly);
        table.put("ScratchReadOnly", scratchReadOnly);
        table.put("UsbBandwidthUsedBytes", usbBandwidthUsedBytes);
        table.put("UsbBandwidthBudgetBytes", usbBandwidthBudgetBytes);
        table.put("Ssd/MediaErrors", ssdMediaErrors);
        table.put("Ssd/UnsafeShutdowns", ssdUnsafeShutdowns);
        table.put("Ssd/PercentUsed", ssdPercentUsed);
        table.put("Ssd/CriticalWarning", ssdCriticalWarning);
        table.put("Ssd/AvailableSparePct", ssdAvailableSparePct, "percent");
        table.put("ExcludedTagsActive", excludedTagsActive);
        table.put("Rewind/Recording", rewindRecording);
        table.put("Rewind/Session", rewindSession);
        table.put("Rewind/FreeGB", rewindFreeGB, "gigabytes");
        table.put("Rewind/FramesDropped", rewindFramesDropped);
        table.put("FieldMap/Hash", fieldMapHash);
        table.put("FieldMap/Name", fieldMapName);
        table.put("FieldMap/LoadedAt", fieldMapLoadedAt);
        table.put("FieldMap/Error", fieldMapError);
    }

    @Override
    public void fromLog(LogTable table) {
        heartbeat = table.get("Heartbeat", heartbeat);
        gpuLoadPct = table.get("GpuLoadPct", gpuLoadPct);
        cpuTempC = table.get("CpuTempC", cpuTempC);
        gpuTempC = table.get("GpuTempC", gpuTempC);
        socTempC = table.get("SocTempC", socTempC);
        tjTempC = table.get("TjTempC", tjTempC);
        fanRpm = table.get("FanRpm", fanRpm);
        powerW = table.get("PowerW", powerW);
        cpuGpuPowerW = table.get("CpuGpuPowerW", cpuGpuPowerW);
        socPowerW = table.get("SocPowerW", socPowerW);
        jpegDecoder = table.get("JpegDecoder", jpegDecoder);
        jpegHardwareOff = table.get("JpegHardwareOff", jpegHardwareOff);
        jpegChecksOk = table.get("JpegChecksOk", jpegChecksOk);
        jpegChecksDiffer = table.get("JpegChecksDiffer", jpegChecksDiffer);
        throttle = table.get("Throttle", throttle);
        overCurrentEvents = table.get("OverCurrentEvents", overCurrentEvents);
        settingsJson = table.get("SettingsJson", settingsJson);
        // Logs from before patch 82 have none: not reported.
        inputVoltageV = table.get("InputVoltageV", Double.NaN);
        inputVoltageMinV = table.get("InputVoltageMinV", Double.NaN);
        inputVoltageLowestV = table.get("InputVoltageLowestV", Double.NaN);
        usbEvents = table.get("UsbEvents", -1);
        lastUsbEvent = table.get("LastUsbEvent", "");
        ethInterface = table.get("Eth/Interface", "");
        ethLinkUp = table.get("Eth/LinkUp", false);
        ethSpeedMbps = table.get("Eth/SpeedMbps", -1);
        ethCarrierChanges = table.get("Eth/CarrierChanges", -1);
        ntDisconnects = table.get("NtDisconnects", -1);
        bootId = table.get("BootId", "");
        uptimeSeconds = table.get("UptimeSeconds", Double.NaN);
        clockSource = table.get("ClockSource", "");
        clockLastJumpSeconds = table.get("ClockLastJumpSeconds", Double.NaN);
        cpuUtilPct = table.get("CpuUtilPct", Double.NaN);
        ramUsedPct = table.get("RamUsedPct", Double.NaN);
        ssdTempC = table.get("Ssd/TempC", Double.NaN);
        fsErrors = table.get("FsErrors", -1);
        settingsReadOnly = table.get("SettingsReadOnly", false);
        scratchReadOnly = table.get("ScratchReadOnly", false);
        usbBandwidthUsedBytes = table.get("UsbBandwidthUsedBytes", -1);
        usbBandwidthBudgetBytes = table.get("UsbBandwidthBudgetBytes", -1);
        ssdMediaErrors = table.get("Ssd/MediaErrors", -1);
        ssdUnsafeShutdowns = table.get("Ssd/UnsafeShutdowns", -1);
        ssdPercentUsed = table.get("Ssd/PercentUsed", -1);
        ssdCriticalWarning = table.get("Ssd/CriticalWarning", -1);
        ssdAvailableSparePct = table.get("Ssd/AvailableSparePct", -1);
        excludedTagsActive = table.get("ExcludedTagsActive", excludedTagsActive);
        rewindRecording = table.get("Rewind/Recording", rewindRecording);
        rewindSession = table.get("Rewind/Session", rewindSession);
        rewindFreeGB = table.get("Rewind/FreeGB", rewindFreeGB);
        rewindFramesDropped = table.get("Rewind/FramesDropped", rewindFramesDropped);
        // Logs from before these were recorded have none: not reported.
        fieldMapHash = table.get("FieldMap/Hash", "");
        fieldMapName = table.get("FieldMap/Name", "");
        fieldMapLoadedAt = table.get("FieldMap/LoadedAt", Double.NaN);
        fieldMapError = table.get("FieldMap/Error", "");
    }
}
