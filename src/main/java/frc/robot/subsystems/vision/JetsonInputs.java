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

    /** GPU load, from the Jetson's GPU load file. */
    public double gpuLoadPct = Double.NaN;

    /** CPU thermal zone. */
    public double cpuTempC = Double.NaN;

    /** GPU thermal zone. */
    public double gpuTempC = Double.NaN;

    /** Hottest of the SoC thermal zones. */
    public double socTempC = Double.NaN;

    /** Junction temperature, the hottest point on the chip; the Jetson throttles in the 80s. */
    public double tjTempC = Double.NaN;

    /** Fan speed; 0 while the board is warm means the fan stopped. */
    public double fanRpm = Double.NaN;

    /** Whole board's input power (INA3221 VDD_IN). */
    public double powerW = Double.NaN;

    /** CPU, GPU and vision accelerator rail power (VDD_CPU_GPU_CV). */
    public double cpuGpuPowerW = Double.NaN;

    /** SoC rail power (VDD_SOC). */
    public double socPowerW = Double.NaN;

    /** Camera JPEG decoder in use: nvjpg (hardware) or libjpeg-turbo (CPU). */
    public String jpegDecoder = "";

    /** The hardware decoder was switched off: a check differed from the CPU, or CUDA failed. */
    public boolean jpegHardwareOff = false;

    /** Hardware-vs-CPU decode checks that matched, since PhotonVision started. */
    public long jpegChecksOk = -1;

    /** Hardware-vs-CPU decode checks that differed, since PhotonVision started. */
    public long jpegChecksDiffer = -1;

    /** Why the Jetson is slowing itself down: None, OVER-CURRENT, HIGH TEMP (...), ... */
    public String throttle = "";

    /** Over-current throttle events since the Jetson booted (power supply too weak). */
    public long overCurrentEvents = -1;

    /**
     * PhotonVision version, SPECTRUM_* settings, excluded tags and field-layout fingerprint, as
     * JSON.
     */
    public String settingsJson = "";

    // -- /photonvision/jetson/, JetsonDiagnostics (SpectrumJetson patch 82) --
    /** Board input voltage (INA3221 VDD_IN), latest. */
    public double inputVoltageV = Double.NaN;

    /** Lowest input voltage in the last second, sampled every 20 ms. */
    public double inputVoltageMinV = Double.NaN;

    /** Lowest input voltage since PhotonVision started. */
    public double inputVoltageLowestV = Double.NaN;

    /**
     * Kernel USB trouble lines since start: disconnects, EMI disables, enumeration, over-current,
     * bandwidth.
     */
    public long usbEvents = -1;

    /** The latest kernel USB trouble line. */
    public String lastUsbEvent = "";

    /** The wired network interface's name. */
    public String ethInterface = "";

    /** The wired link is up. */
    public boolean ethLinkUp = false;

    /** Wired link speed. */
    public long ethSpeedMbps = -1;

    /** Link state changes since boot; two make one drop, which means the cable or switch port. */
    public long ethCarrierChanges = -1;

    /** NetworkTables disconnects PhotonVision has seen. */
    public long ntDisconnects = -1;

    /** Changes on every Jetson reboot, not on a reconnect; the robot's reboot alert watches it. */
    public String bootId = "";

    /** The Jetson's {@code /proc/uptime}; {@code bootId} changes on a reboot. */
    public double uptimeSeconds = Double.NaN;

    /**
     * Where the Jetson's clock came from: robot, internet (NTP), or unset (still the date restored
     * at boot).
     */
    public String clockSource = "";

    /** Size of the last clock correction. */
    public double clockLastJumpSeconds = Double.NaN;

    /** CPU busy share over the last second. */
    public double cpuUtilPct = Double.NaN;

    /** RAM in use (total minus available). */
    public double ramUsedPct = Double.NaN;

    /** SSD temperature. */
    public double ssdTempC = Double.NaN;

    /** ext4 filesystem errors on the mounted partitions. */
    public long fsErrors = -1;

    /** The settings partition is read-only; it never should be. */
    public boolean settingsReadOnly = false;

    /** The scratch partition is read-only; normal in quiet mode. */
    public boolean scratchReadOnly = false;

    /** USB 2.0 bandwidth the cameras have reserved. */
    public long usbBandwidthUsedBytes = -1;

    /** USB 2.0 bandwidth available to the cameras. */
    public long usbBandwidthBudgetBytes = -1;

    /** SSD media errors (SMART), read every 5 min. */
    public long ssdMediaErrors = -1;

    /** SSD unsafe shutdowns, e.g. power cut without a shutdown (SMART). */
    public long ssdUnsafeShutdowns = -1;

    /** SSD wear, percent of its rated life used (SMART). */
    public long ssdPercentUsed = -1;

    /** SSD critical-warning flags (SMART); nonzero needs a look. */
    public long ssdCriticalWarning = -1;

    /** SSD spare blocks left (SMART). */
    public long ssdAvailableSparePct = -1;

    // -- /photonvision/ --
    /** The Jetson's saved excluded tags plus the robot's ({@code excludedTagsActive}). */
    public long[] excludedTagsActive = NO_TAGS;

    // -- /photonvision/rewind/ --
    /** Rewind is recording the cameras right now. */
    public boolean rewindRecording = false;

    /** The current Rewind recording's name. */
    public String rewindSession = "";

    /** Free space on the Jetson's SSD. */
    public double rewindFreeGB = Double.NaN;

    /** Frames Rewind couldn't save: bad JPEGs, write errors, a camera not in MJPEG mode. */
    public long rewindFramesDropped = -1;

    // -- /photonvision/jetson/FieldMap/ --
    /** Canonical hash of the field map the Jetson loaded and solves with; empty until reported. */
    public String fieldMapHash = "";

    /** The robot's name for the field map the Jetson loaded, or photonvision-settings. */
    public String fieldMapName = "";

    /** When the Jetson loaded it, Jetson unix seconds. */
    public double fieldMapLoadedAt = Double.NaN;

    /** Why the Jetson could not load the robot's map, or empty. */
    public String fieldMapError = "";

    /**
     * PhotonVision's log lines since the last cycle ({@code /photonvision/jetson/log}): warnings
     * always, every line while the Jetson is quiet. Its own logs of that time are lost at
     * power-off.
     */
    public String[] logLines = NO_LINES;

    private static final String[] NO_LINES = new String[0];

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
        table.put("LogLines", logLines);
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
        logLines = table.get("LogLines", NO_LINES);
    }
}
