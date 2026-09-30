package frc.robot.subsystems.vision;

import frc.spectrumLib.telemetry.Alert;
import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkString;
import org.wpilib.driverstation.Alert.Level;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.util.Units;
import org.wpilib.system.Timer;

/**
 * The Jetson Orin as a whole: what the robot tells it, and what the robot checks about it before a
 * match. Issue #10, sections 4, 6, 7, 8a, 8f, 9, 11 and 12.
 *
 * <ul>
 *   <li><b>Rewind</b> (8a): with the FMS attached, records from enable until 10 s after the last
 *       disable, so a match is one recording (auto, the gap and teleop), named {@code
 *       <event>-Q12-r0}. Without the FMS it leaves {@code record} alone unless the dashboard's
 *       manual switch changes, so bench tests can drive it themselves.
 *   <li><b>Clock</b> (6): the robot's wall clock every loop, for the Jetson's dates.
 *   <li><b>Excluded tags</b> (11b, 12): a dashboard list, defaulting to the event config, published
 *       to the Jetson and also applied to single-tag solves.
 *   <li><b>Profile</b> (11c): one dashboard switch sets every camera to pipeline 0 (event) or 1
 *       (practice field); event is the default, so a forgotten switch can't carry practice settings
 *       into a match.
 *   <li><b>Alerts</b> (7b, 8f, 9a, 9b): Jetson down, cameras out or slow, overheating, throttling,
 *       JPEG fallback, Rewind problems, stale timestamps, and a bumped camera mount.
 * </ul>
 *
 * <p>Everything here reads logged inputs, and everything it sends goes through {@link JetsonIO} or
 * {@link OrinCameraIO}, which are no-ops in replay.
 */
public class Jetson {

    /** One configured Orin camera, as {@link Jetson} sees it. */
    public record Camera(
            String name, OrinPoseSourceIO source, Transform3d robotToCamera, boolean installed) {}

    private static final String PREFIX = "Vision/Jetson/";

    /** Rewind keeps recording this long after disable, so a match is one recording (8a). */
    private static final double REWIND_TAIL_SECONDS = 10.0;

    /** How often the pipeline request is re-sent, for a camera that reconnected. */
    private static final double PIPELINE_RESEND_SECONDS = 2.0;

    // Alert thresholds: the issue's starting points; tune with logs.
    private static final double HEARTBEAT_TIMEOUT_SECONDS = 3.0;
    private static final double MIN_FPS = 100; // bench: 122
    private static final double MAX_TJ_TEMP_C = 75;
    private static final double MIN_FAN_RPM = 1000;
    private static final double MIN_REWIND_FREE_GB = 20;
    private static final double NO_FRAMES_SECONDS = 1.5;
    private static final double MAX_RESULT_AGE_SECONDS = 0.5;
    private static final long MOUNT_MIN_SAMPLES = 60;
    private static final double MOUNT_MAX_PITCH_STD_DEG = 0.3;
    private static final double MOUNT_MAX_ANGLE_ERROR_DEG = 1.0;
    private static final double MOUNT_MAX_HEIGHT_ERROR_M = 0.02;

    private final JetsonIO io;
    @Getter private final JetsonInputs inputs = new JetsonInputs();
    private final List<Camera> cameras;
    private final boolean expectJetson;

    private final LoggedNetworkString excludedTagsDashboard;
    private final LoggedNetworkBoolean practiceProfile =
            new LoggedNetworkBoolean("/Vision/Orin/PracticeFieldProfile", false);
    private final LoggedNetworkBoolean manualRecord =
            new LoggedNetworkBoolean("/Vision/Orin/Rewind/ManualRecord", false);
    private final LoggedNetworkString manualLabel =
            new LoggedNetworkString("/Vision/Orin/Rewind/ManualLabel", "");

    @Getter private Set<Integer> excludedTags = Set.of();
    private String lastExcludedText = null;
    private int requestedPipeline = -1;
    private double lastPipelineSendSeconds = Double.NEGATIVE_INFINITY;
    private double lastEnabledSeconds = Double.NEGATIVE_INFINITY;
    private boolean lastManualRecord = false;
    private boolean recordRequested = false;
    private double recordingMismatchSince = Double.NaN;

    private long lastHeartbeat = Long.MIN_VALUE;
    private double lastHeartbeatChangeSeconds = Double.NaN;
    private long lastFramesDropped = -1;
    private double framesDroppedRoseSeconds = Double.NaN;
    private long overCurrentAtEnable = -1;
    private boolean wasEnabled = false;

    private final Alert jetsonDown =
            new Alert("Jetson not responding (no heartbeat): Orin cameras are out", Level.HIGH);
    private final Alert hot = new Alert("", Level.MEDIUM);
    private final Alert fan = new Alert("", Level.MEDIUM);
    private final Alert jpeg =
            new Alert(
                    "Jetson fell back to CPU JPEG decoding (same results, more CPU)", Level.MEDIUM);
    private final Alert throttle = new Alert("", Level.MEDIUM);
    private final Alert overCurrent = new Alert("", Level.MEDIUM);
    private final Alert rewindSpace = new Alert("", Level.MEDIUM);
    private final Alert rewindDropping =
            new Alert("Rewind is dropping frames while recording", Level.MEDIUM);
    private final Alert rebooted = new Alert("", Level.MEDIUM);
    private double lastUptime = Double.NaN;
    private final Alert rewindNotRecording =
            new Alert("Rewind should be recording this match but isn't", Level.MEDIUM);

    /** Per-camera alerts and the state they need. */
    private final class CameraCheck {
        final Camera camera;
        final Alert notOnNt;
        final Alert noFrames;
        final Alert stuck;
        final Alert slow;
        final Alert decode;
        final Alert recovered;
        final Alert stale;
        final Alert mount;
        final org.wpilib.math.filter.Debouncer notOnNtDebounce =
                new org.wpilib.math.filter.Debouncer(2.0);
        final org.wpilib.math.filter.Debouncer stuckDebounce =
                new org.wpilib.math.filter.Debouncer(1.0);
        final org.wpilib.math.filter.Debouncer slowDebounce =
                new org.wpilib.math.filter.Debouncer(2.0);
        final org.wpilib.math.filter.Debouncer staleDebounce =
                new org.wpilib.math.filter.Debouncer(2.0);
        long decodeBaseline = -1;
        long recoveriesAtEnable = -1;

        CameraCheck(Camera camera) {
            this.camera = camera;
            String n = camera.name();
            notOnNt = new Alert("Orin camera " + n + " not on NetworkTables", Level.HIGH);
            noFrames =
                    new Alert("Orin camera " + n + " connected but sending no frames", Level.HIGH);
            stuck = new Alert("Orin camera " + n + " at 0 fps", Level.HIGH);
            slow = new Alert("", Level.MEDIUM);
            decode = new Alert("Orin camera " + n + " has JPEG decode failures", Level.MEDIUM);
            recovered =
                    new Alert(
                            "Orin camera "
                                    + n
                                    + " was reset this match (out ~3-10 s): check its cable",
                            Level.MEDIUM);
            stale =
                    new Alert(
                            "Orin camera "
                                    + n
                                    + " timestamps look wrong (time sync?): results far from now",
                            Level.MEDIUM);
            mount = new Alert("", Level.MEDIUM);
        }
    }

    private final List<CameraCheck> checks = new ArrayList<>();

    /**
     * @param io the Jetson ({@link JetsonIO#NONE} in replay)
     * @param cameras the Orin cameras
     * @param defaultExcludedTags the event's bad tags, until changed on the dashboard
     * @param expectJetson whether the Jetson should be there (real robot); off in sim, so its
     *     alerts stay quiet
     */
    public Jetson(
            JetsonIO io, List<Camera> cameras, int[] defaultExcludedTags, boolean expectJetson) {
        this.io = io;
        this.cameras = List.copyOf(cameras);
        this.expectJetson = expectJetson;
        StringBuilder text = new StringBuilder();
        for (int id : defaultExcludedTags) {
            text.append(text.length() == 0 ? "" : ",").append(id);
        }
        excludedTagsDashboard =
                new LoggedNetworkString("/Vision/Orin/ExcludedTags", text.toString());
        for (Camera c : cameras) {
            checks.add(new CameraCheck(c));
        }
    }

    /** Parses "7, 12,13" into tag ids; anything that isn't a number is skipped. */
    static Set<Integer> parseTags(String text) {
        Set<Integer> ids = new LinkedHashSet<>();
        for (String part : text.split("[,\\s]+")) {
            if (part.isEmpty()) {
                continue;
            }
            try {
                ids.add(Integer.parseInt(part));
            } catch (NumberFormatException e) {
                // skip it; the logged list shows what was used
            }
        }
        return Collections.unmodifiableSet(ids);
    }

    /** Reads the Jetson. Call before the Orin sources update, so they see this loop's tag list. */
    public void updateInputs() {
        io.updateInputs(inputs);
        Logger.processInputs("Vision/Jetson", inputs);
        String text = excludedTagsDashboard.get();
        if (!text.equals(lastExcludedText)) {
            lastExcludedText = text;
            excludedTags = parseTags(text);
            io.setExcludedTags(excludedTags.stream().mapToLong(Integer::longValue).toArray());
            Logger.recordOutput(
                    PREFIX + "ExcludedTags",
                    excludedTags.stream().mapToLong(Integer::longValue).toArray());
        }
    }

    /**
     * Sends the Jetson this loop's commands and updates the alerts.
     *
     * @param robotStill whether the robot is still enough for the mount check
     */
    public void periodic(boolean robotStill) {
        double now = Timer.getTimestamp();
        boolean enabled = RobotState.isEnabled();
        boolean disabled = !enabled;

        io.publishRobotClock();
        updatePipeline(now);
        updateRewind(now, enabled);

        if (enabled && !wasEnabled) {
            rebooted.set(false);
            overCurrentAtEnable = inputs.overCurrentEvents;
            for (CameraCheck c : checks) {
                c.recoveriesAtEnable = c.camera.source().getCameraInputs().healthRecoveries;
            }
        }
        wasEnabled = enabled;

        updateJetsonAlerts(now);
        for (CameraCheck c : checks) {
            updateCameraAlerts(c, now, disabled && robotStill);
        }
    }

    private void updatePipeline(double now) {
        int wanted = practiceProfile.get() ? 1 : 0;
        boolean resend = now - lastPipelineSendSeconds >= PIPELINE_RESEND_SECONDS;
        if (wanted != requestedPipeline || resend) {
            requestedPipeline = wanted;
            lastPipelineSendSeconds = now;
            for (Camera c : cameras) {
                c.source().setPipelineIndex(wanted);
            }
        }
        Logger.recordOutput(PREFIX + "RequestedPipeline", requestedPipeline);
    }

    private void updateRewind(double now, boolean enabled) {
        if (enabled) {
            lastEnabledSeconds = now;
        }
        if (RobotState.isFMSAttached()) {
            String label = rewindLabel();
            recordRequested = now - lastEnabledSeconds < REWIND_TAIL_SECONDS;
            io.setRewind(recordRequested, label);
            Logger.recordOutput(PREFIX + "Rewind/Label", label);
        } else {
            boolean manual = manualRecord.get();
            if (manual != lastManualRecord) {
                io.setRewind(manual, manualLabel.get());
            }
            lastManualRecord = manual;
            recordRequested = manual;
        }
        Logger.recordOutput(PREFIX + "Rewind/RecordRequested", recordRequested);
    }

    private String cachedLabel = "";
    private String cachedLabelKey = "";

    /** {@code <event>-Q12-r0}, rebuilt only when the match changes. */
    private String rewindLabel() {
        var type = MatchState.getMatchType();
        int number = MatchState.getMatchNumber();
        int replay = MatchState.getReplayNumber();
        String event = MatchState.getEventName();
        String key = event + type + number + ":" + replay;
        if (!key.equals(cachedLabelKey)) {
            cachedLabelKey = key;
            String letter =
                    switch (type) {
                        case PRACTICE -> "P";
                        case QUALIFICATION -> "Q";
                        case ELIMINATION -> "E";
                        default -> "M";
                    };
            cachedLabel = String.format("%s-%s%d-r%d", event, letter, number, replay);
        }
        return cachedLabel;
    }

    /** Sets an alert, building its text only while it is active (no per-loop strings). */
    private static void show(
            Alert alert, boolean active, java.util.function.Supplier<String> text) {
        if (active) {
            alert.setText(text.get());
        }
        alert.set(active);
    }

    private void updateJetsonAlerts(double now) {
        if (inputs.heartbeat != lastHeartbeat) {
            lastHeartbeat = inputs.heartbeat;
            lastHeartbeatChangeSeconds = now;
        }
        boolean down =
                inputs.heartbeat < 0
                        || now - lastHeartbeatChangeSeconds > HEARTBEAT_TIMEOUT_SECONDS;
        Logger.recordOutput(PREFIX + "HeartbeatAgeSeconds", now - lastHeartbeatChangeSeconds);
        jetsonDown.set(expectJetson && down);
        frc.spectrumLib.telemetry.Telemetry.logDash("Vision/Jetson/Connected", !down);
        boolean up = expectJetson && !down;

        // Uptime dropping means the Jetson rebooted (power or watchdog); its own log survives power
        // cuts, so look up why after the match. Stays up until the next enable.
        if (!Double.isNaN(lastUptime) && inputs.uptimeSeconds < lastUptime - 1.0) {
            rebooted.setText(
                    String.format(
                            "Jetson rebooted at robot time %.0f s: check its log and power", now));
            rebooted.set(expectJetson);
        }
        if (!Double.isNaN(inputs.uptimeSeconds)) {
            lastUptime = inputs.uptimeSeconds;
        }

        show(
                hot,
                up && inputs.tjTempC > MAX_TJ_TEMP_C,
                () -> String.format("Jetson hot: %.0f C (throttles in the 80s)", inputs.tjTempC));
        show(
                fan,
                up && inputs.fanRpm < MIN_FAN_RPM,
                () -> String.format("Jetson fan stopped? %.0f rpm", inputs.fanRpm));
        jpeg.set(up && (inputs.jpegChecksDiffer > 0 || inputs.jpegHardwareOff));
        String t = inputs.throttle;
        boolean throttling =
                t.contains("OVER-CURRENT") || t.contains("HIGH TEMP") || t.contains("CLOCK CAPPED");
        show(throttle, up && throttling, () -> "Jetson throttling: " + t);
        boolean overCurrentRose =
                overCurrentAtEnable >= 0 && inputs.overCurrentEvents > overCurrentAtEnable;
        show(
                overCurrent,
                up && overCurrentRose,
                () ->
                        String.format(
                                "Jetson over-current events this match: %d (check its power"
                                        + " wiring)",
                                inputs.overCurrentEvents - Math.max(overCurrentAtEnable, 0)));
        show(
                rewindSpace,
                up && inputs.rewindFreeGB < MIN_REWIND_FREE_GB,
                () -> String.format("Rewind SSD low: %.0f GB free", inputs.rewindFreeGB));
        if (inputs.rewindFramesDropped > lastFramesDropped
                && lastFramesDropped >= 0
                && inputs.rewindRecording) {
            framesDroppedRoseSeconds = now;
        }
        lastFramesDropped = inputs.rewindFramesDropped;
        rewindDropping.set(
                up
                        && !Double.isNaN(framesDroppedRoseSeconds)
                        && now - framesDroppedRoseSeconds < 10.0);
        boolean mismatch = recordRequested && !inputs.rewindRecording;
        if (!mismatch) {
            recordingMismatchSince = Double.NaN;
        } else if (Double.isNaN(recordingMismatchSince)) {
            recordingMismatchSince = now;
        }
        rewindNotRecording.set(
                up && RobotState.isFMSAttached() && mismatch && now - recordingMismatchSince > 2.0);
    }

    private void updateCameraAlerts(CameraCheck c, double now, boolean mountCheckAllowed) {
        OrinCameraInputs in = c.camera.source().getCameraInputs();
        // A camera turned off on purpose reads as disconnected after 0.5 s: no alerts for it (8f).
        boolean watch = expectJetson && c.camera.installed() && in.enabled;
        c.notOnNt.set(c.notOnNtDebounce.calculate(watch && !in.connected));
        double lastResult = c.camera.source().getLastResultSeconds();
        boolean noFrames =
                in.connected && (Double.isNaN(lastResult) || now - lastResult > NO_FRAMES_SECONDS);
        c.noFrames.set(watch && noFrames);
        c.stuck.set(c.stuckDebounce.calculate(watch && in.connected && in.healthFps == 0));
        show(
                c.slow,
                c.slowDebounce.calculate(
                        watch && in.connected && in.healthFps > 0 && in.healthFps < MIN_FPS),
                () ->
                        String.format(
                                "Orin camera %s slow: %.0f fps", c.camera.name(), in.healthFps));

        if (c.decodeBaseline < 0 && in.healthDecodeFailures >= 0) {
            c.decodeBaseline = in.healthDecodeFailures;
        }
        c.decode.set(watch && c.decodeBaseline >= 0 && in.healthDecodeFailures > c.decodeBaseline);

        boolean recovered = c.recoveriesAtEnable >= 0 && in.healthRecoveries > c.recoveriesAtEnable;
        c.recovered.set(watch && recovered);

        double age = now - c.camera.source().getLastResultTimestampSeconds();
        boolean fresh = !Double.isNaN(lastResult) && now - lastResult < 0.25;
        c.stale.set(
                c.staleDebounce.calculate(
                        watch && fresh && Math.abs(age) > MAX_RESULT_AGE_SECONDS));

        c.mount.set(watch && mountCheckAllowed && mountLooksBumped(c, in));
    }

    /** 9b: compares the Jetson's measured mount with the configured one, while still. */
    private boolean mountLooksBumped(CameraCheck c, OrinCameraInputs in) {
        if (in.mountSamples < MOUNT_MIN_SAMPLES
                || !(in.mountPitchStdDeg < MOUNT_MAX_PITCH_STD_DEG)) {
            return false;
        }
        Transform3d mount = c.camera.robotToCamera();
        double pitch = Units.radiansToDegrees(mount.getRotation().getY());
        double roll = Units.radiansToDegrees(mount.getRotation().getX());
        double height = mount.getZ();
        double dPitch = in.mountPitchDeg - pitch;
        double dRoll = in.mountRollDeg - roll;
        double dHeight = in.mountHeightM - height;
        boolean bumped =
                Math.abs(dPitch) > MOUNT_MAX_ANGLE_ERROR_DEG
                        || Math.abs(dRoll) > MOUNT_MAX_ANGLE_ERROR_DEG
                        || Math.abs(dHeight) > MOUNT_MAX_HEIGHT_ERROR_M;
        if (!bumped) {
            return false;
        }
        c.mount.setText(
                String.format(
                        "Orin camera %s mount differs from config: pitch %+.1f deg, roll %+.1f"
                                + " deg, height %+.1f cm",
                        c.camera.name(), dPitch, dRoll, dHeight * 100));
        return bumped;
    }
}
