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

    // -- PhotonVision's metrics protobuf, /photonvision//metrics/<host> (section 7a) --
    /** Seconds since the Jetson booted; a drop means it rebooted. */
    public double uptimeSeconds = Double.NaN;

    public double cpuUtilPct = Double.NaN;
    public double ramUtilPct = Double.NaN;
    public double diskUsableSpace = Double.NaN;

    // -- /photonvision/ --
    /** The Jetson's saved excluded tags plus the robot's ({@code excludedTagsActive}). */
    public long[] excludedTagsActive = NO_TAGS;

    // -- /photonvision/rewind/ --
    public boolean rewindRecording = false;
    public String rewindSession = "";
    public double rewindFreeGB = Double.NaN;
    public long rewindFramesDropped = -1;

    @Override
    public void toLog(LogTable table) {
        table.put("Heartbeat", heartbeat);
        table.put("GpuLoadPct", gpuLoadPct);
        table.put("CpuTempC", cpuTempC);
        table.put("GpuTempC", gpuTempC);
        table.put("SocTempC", socTempC);
        table.put("TjTempC", tjTempC);
        table.put("FanRpm", fanRpm);
        table.put("PowerW", powerW);
        table.put("CpuGpuPowerW", cpuGpuPowerW);
        table.put("SocPowerW", socPowerW);
        table.put("JpegDecoder", jpegDecoder);
        table.put("JpegHardwareOff", jpegHardwareOff);
        table.put("JpegChecksOk", jpegChecksOk);
        table.put("JpegChecksDiffer", jpegChecksDiffer);
        table.put("Throttle", throttle);
        table.put("OverCurrentEvents", overCurrentEvents);
        table.put("SettingsJson", settingsJson);
        table.put("UptimeSeconds", uptimeSeconds);
        table.put("CpuUtilPct", cpuUtilPct);
        table.put("RamUtilPct", ramUtilPct);
        table.put("DiskUsableSpace", diskUsableSpace);
        table.put("ExcludedTagsActive", excludedTagsActive);
        table.put("Rewind/Recording", rewindRecording);
        table.put("Rewind/Session", rewindSession);
        table.put("Rewind/FreeGB", rewindFreeGB);
        table.put("Rewind/FramesDropped", rewindFramesDropped);
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
        uptimeSeconds = table.get("UptimeSeconds", uptimeSeconds);
        cpuUtilPct = table.get("CpuUtilPct", cpuUtilPct);
        ramUtilPct = table.get("RamUtilPct", ramUtilPct);
        diskUsableSpace = table.get("DiskUsableSpace", diskUsableSpace);
        excludedTagsActive = table.get("ExcludedTagsActive", excludedTagsActive);
        rewindRecording = table.get("Rewind/Recording", rewindRecording);
        rewindSession = table.get("Rewind/Session", rewindSession);
        rewindFreeGB = table.get("Rewind/FreeGB", rewindFreeGB);
        rewindFramesDropped = table.get("Rewind/FramesDropped", rewindFramesDropped);
    }
}
