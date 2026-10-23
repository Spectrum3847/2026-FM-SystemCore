package frc.spectrumLib.telemetry;

import frc.spectrumLib.framework.RobotLoop;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.mechanism.MotorInputs;
import java.io.File;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.LongSupplier;
import org.littletonrobotics.junction.Logger;
import org.wpilib.driverstation.Alert.Level;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.MatchType;
import org.wpilib.driverstation.RobotState;
import org.wpilib.system.Timer;

/**
 * A short summary of the run, kept as it happens: battery, brownouts, loop overruns, CAN drops and
 * reboots, Driver Station drops, vision counts, how auto went, alerts raised and peak motor
 * temperatures. Every value is logged under {@code Summary/...} once a second, in every log.
 *
 * <p>When an FMS match ends (match type not {@code NONE}, teleop enabled then not), the summary is
 * also written as {@code <log name>.summary.json} next to the log, so the pit laptop and the log
 * site can read it without parsing the log. Power is usually cut on the way off the field, so the
 * file is written at once: the JSON is built on the main thread at the end of teleop and a
 * background thread writes it (tmp file, fsync, rename). Practice runs get the log keys only. The
 * keys and the file are documented in {@code docs/match-summary.md}.
 *
 * <p>Everything is fed from logged inputs (motor inputs, the Driver Station, the robot clock,
 * battery and brownout inputs the robot passes in), so a replayed log recomputes the same summary.
 * The scope is the whole log: one log is one boot, and the robot is powered off between matches.
 *
 * <p>Main thread only, apart from the file write. Per loop it is a few compares per registered
 * device; logging is once a second.
 */
public final class MatchSummary {
    /** Bumped when a field changes meaning or is removed. Adding a field does not bump it. */
    public static final int VERSION = 1;

    /** Battery thresholds whose time below is totalled. */
    public static final double LOW_VOLTS = 8.0;

    public static final double VERY_LOW_VOLTS = 7.0;

    /**
     * Readings at or below this are no reading (the controller reports 0 V before its first
     * sample).
     */
    static final double NO_READING_VOLTS = 1.0;

    /**
     * Longest step credited to a duration (time under a voltage, without the Driver Station, a bus
     * down). A stall longer than this is not time the condition was observed.
     */
    static final double MAX_STEP_SECONDS = 0.5;

    /** A loop longer than this many periods is an overrun, as in {@link SystemLoadMonitor}. */
    static final double OVERRUN_FACTOR = SystemLoadMonitor.LOOP_OVERRUN_FACTOR;

    static final double LOG_PERIOD_SECONDS = 1.0;

    /** Caps that keep the file a few KB however bad the match was. */
    static final int MAX_EVENTS_PER_DEVICE = 8;

    static final int MAX_ALERT_TEXTS = 20;

    /** Device names in the file are for people: nothing in the summary is keyed on a CAN ID. */
    private static final class Device {
        final String name;
        final int bus;
        final BooleanSupplier connected;
        final BooleanSupplier reset;
        final DoubleSupplier tempCelsius;
        final String peakTempKey;

        boolean everConnected;
        boolean lastConnected;
        int drops;
        int reboots;
        double peakTemp = Double.NaN;
        final List<String> events = new ArrayList<>(2);
        int eventsDropped;

        Device(
                String name,
                int bus,
                BooleanSupplier connected,
                BooleanSupplier reset,
                DoubleSupplier tempCelsius) {
            this.name = name;
            this.bus = bus;
            this.connected = connected;
            this.reset = reset;
            this.tempCelsius = tempCelsius;
            this.peakTempKey = tempCelsius == null ? null : "Summary/PeakTempC/" + name;
        }

        void event(double t, String what) {
            if (events.size() < MAX_EVENTS_PER_DEVICE) {
                events.add(fmt(t) + " " + what);
            } else {
                eventsDropped++;
            }
        }
    }

    private static final class Bus {
        final String label;
        final String dropsKey;
        final String downSecondsKey;
        int devices;

        // Per loop, from the device pass.
        int tracked;
        int up;

        boolean down;
        int drops;
        double downSeconds;
        final List<String> events = new ArrayList<>(2);

        Bus(String label) {
            this.label = label;
            this.dropsKey = "Summary/CAN/" + label + "/Drops";
            this.downSecondsKey = "Summary/CAN/" + label + "/DownSeconds";
        }
    }

    private static final class VisionSource {
        final String name;
        final LongSupplier accepted;
        final LongSupplier rejected;
        final String acceptedKey;
        final String rejectedKey;

        VisionSource(String name, LongSupplier accepted, LongSupplier rejected) {
            this.name = name;
            this.accepted = accepted;
            this.rejected = rejected;
            this.acceptedKey = "Summary/Vision/" + name + "/Accepted";
            this.rejectedKey = "Summary/Vision/" + name + "/Rejected";
        }
    }

    private static final class AlertRecord {
        final String level;
        final double firstAt;
        int count;

        AlertRecord(String level, double firstAt) {
            this.level = level;
            this.firstAt = firstAt;
        }
    }

    /** Who and what the match was, read from the Driver Station when it ends. */
    record MatchInfo(
            String event,
            String type,
            int number,
            int replay,
            String alliance,
            int station,
            double matchTime) {}

    private final List<Device> devices = new ArrayList<>();
    private final List<Bus> buses = new ArrayList<>();
    private final Map<String, Integer> busIndex = new LinkedHashMap<>();
    private final List<VisionSource> vision = new ArrayList<>();
    private final Thread mainThread = Thread.currentThread();

    /** Writes the file; null where no file is wanted (replay, tests). */
    private final SummaryFileWriter writer;

    // ── Accumulators ─────────────────────────────────────────────────────────
    private double lastNow = Double.NaN;

    private double minVolts = Double.NaN;
    private double minVoltsAt = Double.NaN;
    private double secondsUnderLow;
    private double secondsUnderVeryLow;

    private boolean lastBrownedOut;
    private int brownouts;
    private double brownoutSeconds;
    private double firstBrownoutAt = Double.NaN;

    private long enabledLoops;
    private long overruns;
    private double longestLoopSeconds;
    private double longestLoopAt = Double.NaN;

    private boolean dsSeen;
    private boolean lastDs;
    private int dsDrops;
    private int dsDropsWhileEnabled;
    private boolean lastEnabled;
    private double secondsWithoutDs;
    private final List<String> dsEvents = new ArrayList<>(2);

    private boolean lastAutoEnabled;
    private boolean lastTeleopEnabled;
    private double autoStart = Double.NaN;
    private double autoEnd = Double.NaN;
    private double teleopStart = Double.NaN;
    private double teleopEnd = Double.NaN;

    private String autoRoutine = "";
    private BooleanSupplier autoRunning;
    private double autoCommandStart = Double.NaN;
    private String autoEnded = "";
    private double autoSeconds = Double.NaN;

    private int errorsRaised;
    private int warningsRaised;
    private int infosRaised;
    private final Map<String, AlertRecord> alertTexts = new LinkedHashMap<>();
    private int alertTextsDropped;
    private boolean alertTextsChanged;

    private int matchesEnded;

    /** Last file write result, logged on the main thread. */
    private String loggedFileResult = "";

    /**
     * Creates the summary and starts listening for alerts. Create it early in the robot's
     * constructor, on the main thread, so alerts raised while the robot starts are counted.
     *
     * @param logFolder the folder the log is written to, for the file; null for no file (replay)
     */
    public MatchSummary(File logFolder) {
        this.writer = logFolder == null ? null : new SummaryFileWriter(logFolder);
        Alert.setRaisedListener(this::alertRaised);
    }

    // ── Registration ─────────────────────────────────────────────────────────

    private int bus(String label) {
        Integer i = busIndex.get(label);
        if (i == null) {
            i = buses.size();
            buses.add(new Bus(label));
            busIndex.put(label, i);
        }
        buses.get(i).devices++;
        return i;
    }

    /**
     * Tracks a CAN device: when it drops off and comes back, reboots, and its peak temperature.
     * More than half the devices on a bus quiet at once counts as a drop of that bus.
     *
     * @param name shown in the summary
     * @param busLabel the bus, as its health is logged (e.g. {@code CANivore})
     * @param connected the device's logged connected input
     * @param reset its logged reboot input, or null
     * @param tempCelsius its logged temperature, or null
     */
    public void addDevice(
            String name,
            String busLabel,
            BooleanSupplier connected,
            BooleanSupplier reset,
            DoubleSupplier tempCelsius) {
        devices.add(new Device(name, bus(busLabel), connected, reset, tempCelsius));
    }

    /**
     * Tracks a mechanism's leader and followers from its {@link MotorInputs}. Does nothing for a
     * mechanism with no hardware attached.
     *
     * @param mechanism the mechanism
     * @param busLabel turns a config's bus name into the label its health is logged under
     */
    public void addMechanism(Mechanism mechanism, Function<String, String> busLabel) {
        if (!mechanism.isAttached()) {
            return;
        }
        MotorInputs in = mechanism.getInputs();
        var config = mechanism.config;
        addDevice(
                config.getName(),
                busLabel.apply(config.getId().getBus()),
                () -> in.connected,
                () -> in.resetOccurred,
                () -> in.tempCelsius);
        var followers = config.getFollowerConfigs();
        for (int i = 0; i < followers.length; i++) {
            final int f = i;
            addDevice(
                    followers[i].getName(),
                    busLabel.apply(followers[i].getId().getBus()),
                    () -> f < in.followerConnected.length && in.followerConnected[f],
                    () -> f < in.followerResetOccurred.length && in.followerResetOccurred[f],
                    null);
        }
    }

    /**
     * Tracks a vision source's running accepted and rejected counts.
     *
     * @param name the source, e.g. {@code LL-Back/MT2}
     * @param accepted observations accepted since boot
     * @param rejected observations rejected since boot
     */
    public void addVisionSource(String name, LongSupplier accepted, LongSupplier rejected) {
        vision.add(new VisionSource(name, accepted, rejected));
    }

    /**
     * Records the auto routine that just started. Call from {@code autonomousInit()}, after it is
     * scheduled.
     *
     * @param routine its name
     * @param running whether it is still running (e.g. {@code command::isScheduled})
     */
    public void autoStarted(String routine, BooleanSupplier running) {
        autoStarted(routine, running, Timer.getTimestamp());
    }

    void autoStarted(String routine, BooleanSupplier running, double now) {
        autoRoutine = routine;
        autoRunning = running;
        autoCommandStart = now;
        autoEnded = "";
        autoSeconds = Double.NaN;
    }

    // ── Per loop ─────────────────────────────────────────────────────────────

    /**
     * Advances every accumulator, logs the summary once a second, and on the loop an FMS match ends
     * hands the file to the writer. Call once per loop, after the command scheduler, so this loop's
     * motor inputs have been read.
     *
     * @param batteryVolts the logged battery voltage
     * @param brownedOut the logged brownout state
     */
    public void periodic(double batteryVolts, boolean brownedOut) {
        double now = Timer.getTimestamp();
        boolean ended =
                update(
                        now,
                        batteryVolts,
                        brownedOut,
                        RobotState.isDSAttached(),
                        RobotState.isEnabled(),
                        RobotState.isAutonomous(),
                        RobotState.isTeleop(),
                        MatchState.getMatchType() != MatchType.NONE);
        if (ended || RobotLoop.everySeconds(LOG_PERIOD_SECONDS)) {
            log();
        }
        if (ended && writer != null) {
            writer.submit(toJson(matchInfo(), now));
        }
    }

    private static MatchInfo matchInfo() {
        return new MatchInfo(
                MatchState.getEventName(),
                MatchState.getMatchType().name(),
                MatchState.getMatchNumber(),
                MatchState.getReplayNumber(),
                MatchState.getAlliance().map(Enum::name).orElse("NONE"),
                MatchState.getLocation().orElse(0),
                MatchState.getMatchTime());
    }

    /**
     * Advances the accumulators by one loop.
     *
     * @return whether an FMS match's teleop ended this loop
     */
    boolean update(
            double now,
            double batteryVolts,
            boolean brownedOut,
            boolean dsAttached,
            boolean enabled,
            boolean autonomous,
            boolean teleop,
            boolean fmsMatch) {
        double dt = Double.isNaN(lastNow) ? 0 : Math.max(0, now - lastNow);
        double step = Math.min(dt, MAX_STEP_SECONDS);
        lastNow = now;

        // Loop timing, enabled only: disabled loops include PathPlanner's warmup on purpose.
        if (enabled && dt > 0) {
            enabledLoops++;
            if (dt > RobotLoop.periodSeconds() * OVERRUN_FACTOR) {
                overruns++;
            }
            if (dt > longestLoopSeconds) {
                longestLoopSeconds = dt;
                longestLoopAt = now;
            }
        }

        if (batteryVolts > NO_READING_VOLTS) {
            if (Double.isNaN(minVolts) || batteryVolts < minVolts) {
                minVolts = batteryVolts;
                minVoltsAt = now;
            }
            if (batteryVolts < LOW_VOLTS) {
                secondsUnderLow += step;
            }
            if (batteryVolts < VERY_LOW_VOLTS) {
                secondsUnderVeryLow += step;
            }
        }

        if (brownedOut) {
            brownoutSeconds += step;
            if (!lastBrownedOut) {
                brownouts++;
                if (Double.isNaN(firstBrownoutAt)) {
                    firstBrownoutAt = now;
                }
            }
        }
        lastBrownedOut = brownedOut;

        if (dsAttached) {
            if (dsSeen && !lastDs && dsEvents.size() < MAX_EVENTS_PER_DEVICE) {
                dsEvents.add(fmt(now) + " back");
            }
            dsSeen = true;
        } else if (dsSeen) {
            secondsWithoutDs += step;
            if (lastDs) {
                dsDrops++;
                // A drop in the queue or on the cart is harmless; one while enabled cost the match.
                if (lastEnabled) {
                    dsDropsWhileEnabled++;
                }
                if (dsEvents.size() < MAX_EVENTS_PER_DEVICE) {
                    dsEvents.add(fmt(now) + " drop");
                }
            }
        }
        lastDs = dsAttached;
        lastEnabled = enabled;

        updateDevices(now, step);

        boolean autoEnabled = enabled && autonomous;
        boolean teleopEnabled = enabled && teleop;
        if (autoEnabled && !lastAutoEnabled) {
            autoStart = now;
        }
        if (autoEnabled && autoRunning != null && autoEnded.isEmpty()) {
            if (!autoRunning.getAsBoolean()) {
                autoEnded = "finished";
                autoSeconds = now - autoCommandStart;
            }
        }
        if (!autoEnabled && lastAutoEnabled) {
            autoEnd = now;
            if (autoRunning != null && autoEnded.isEmpty()) {
                autoEnded = "cut off";
                autoSeconds = now - autoCommandStart;
            }
        }
        if (teleopEnabled && !lastTeleopEnabled) {
            teleopStart = now;
        }
        boolean matchEnded = false;
        if (!teleopEnabled && lastTeleopEnabled) {
            teleopEnd = now;
            if (fmsMatch) {
                matchEnded = true;
                matchesEnded++;
            }
        }
        lastAutoEnabled = autoEnabled;
        lastTeleopEnabled = teleopEnabled;
        return matchEnded;
    }

    private void updateDevices(double now, double step) {
        for (Bus b : buses) {
            b.tracked = 0;
            b.up = 0;
        }
        for (Device d : devices) {
            boolean c = d.connected.getAsBoolean();
            if (c && !d.lastConnected) {
                if (d.everConnected) {
                    d.event(now, "back");
                }
                d.everConnected = true;
            } else if (!c && d.lastConnected) {
                d.drops++;
                d.event(now, "drop");
            }
            d.lastConnected = c;
            if (d.reset != null && d.reset.getAsBoolean()) {
                d.reboots++;
                d.event(now, "reboot");
            }
            // A disconnected device's temperature is a stale or zeroed reading.
            if (c && d.tempCelsius != null) {
                double t = d.tempCelsius.getAsDouble();
                if (Double.isNaN(d.peakTemp) || t > d.peakTemp) {
                    d.peakTemp = t;
                }
            }
            if (d.everConnected) {
                Bus b = buses.get(d.bus);
                b.tracked++;
                if (c) {
                    b.up++;
                }
            }
        }
        for (Bus b : buses) {
            // More than half: all of them when a CANivore drops, those past the break when a chain
            // is cut, and a device whose connected input never goes false cannot hide it.
            boolean down = b.tracked > 0 && b.up * 2 < b.tracked;
            if (down) {
                b.downSeconds += step;
                if (!b.down) {
                    b.drops++;
                    if (b.events.size() < MAX_EVENTS_PER_DEVICE) {
                        b.events.add(fmt(now) + " drop");
                    }
                }
            } else if (b.down && b.events.size() < MAX_EVENTS_PER_DEVICE) {
                b.events.add(fmt(now) + " back");
            }
            b.down = down;
        }
    }

    /** Called by {@link Alert#periodic()} for each alert that turned on since the last loop. */
    void alertRaised(Level level, String text) {
        if (Thread.currentThread() != mainThread) {
            return; // the main thread owns this; an off-thread alert still shows on the DS
        }
        String name;
        switch (level) {
            case HIGH -> {
                errorsRaised++;
                name = "error";
            }
            case MEDIUM -> {
                warningsRaised++;
                name = "warning";
            }
            default -> {
                infosRaised++;
                return; // counted, not listed
            }
        }
        AlertRecord r = alertTexts.get(text);
        if (r == null) {
            if (alertTexts.size() >= MAX_ALERT_TEXTS) {
                alertTextsDropped++;
                return;
            }
            r = new AlertRecord(name, Double.isNaN(lastNow) ? Timer.getTimestamp() : lastNow);
            alertTexts.put(text, r);
        }
        r.count++;
        alertTextsChanged = true;
    }

    // ── Log keys ─────────────────────────────────────────────────────────────

    private void log() {
        Logger.recordOutput("Summary/Battery/MinVolts", nanToZero(minVolts));
        Logger.recordOutput("Summary/Battery/SecondsUnder8V", secondsUnderLow);
        Logger.recordOutput("Summary/Battery/SecondsUnder7V", secondsUnderVeryLow);
        Logger.recordOutput("Summary/Brownouts", brownouts);
        Logger.recordOutput("Summary/Loop/EnabledLoops", enabledLoops);
        Logger.recordOutput("Summary/Loop/Overruns", overruns);
        Logger.recordOutput("Summary/Loop/LongestMs", longestLoopSeconds * 1000);
        Logger.recordOutput("Summary/DS/Drops", dsDrops);
        Logger.recordOutput("Summary/DS/DropsWhileEnabled", dsDropsWhileEnabled);
        Logger.recordOutput("Summary/DS/SecondsWithout", secondsWithoutDs);
        int deviceDrops = 0;
        int reboots = 0;
        for (Device d : devices) {
            deviceDrops += d.drops;
            reboots += d.reboots;
            if (d.peakTempKey != null && !Double.isNaN(d.peakTemp)) {
                Logger.recordOutput(d.peakTempKey, d.peakTemp);
            }
        }
        Logger.recordOutput("Summary/Devices/Drops", deviceDrops);
        Logger.recordOutput("Summary/Devices/Reboots", reboots);
        for (Bus b : buses) {
            Logger.recordOutput(b.dropsKey, b.drops);
            Logger.recordOutput(b.downSecondsKey, b.downSeconds);
        }
        for (VisionSource v : vision) {
            Logger.recordOutput(v.acceptedKey, v.accepted.getAsLong());
            Logger.recordOutput(v.rejectedKey, v.rejected.getAsLong());
        }
        Logger.recordOutput("Summary/Auto/Routine", autoRoutine);
        Logger.recordOutput("Summary/Auto/Ended", autoEnded);
        Logger.recordOutput("Summary/Auto/Seconds", nanToZero(autoSeconds));
        Logger.recordOutput("Summary/Alerts/ErrorsRaised", errorsRaised);
        Logger.recordOutput("Summary/Alerts/WarningsRaised", warningsRaised);
        if (alertTextsChanged) {
            alertTextsChanged = false;
            String[] raised = new String[alertTexts.size()];
            int i = 0;
            for (var e : alertTexts.entrySet()) {
                raised[i++] = e.getValue().level + " x" + e.getValue().count + ": " + e.getKey();
            }
            Logger.recordOutput("Summary/Alerts/Raised", raised);
        }
        Logger.recordOutput("Summary/MatchesEnded", matchesEnded);
        if (writer != null) {
            String result = writer.lastResult();
            if (!result.equals(loggedFileResult)) {
                loggedFileResult = result;
                Logger.recordOutput("Summary/File", result);
            }
        }
    }

    private static double nanToZero(double v) {
        return Double.isNaN(v) ? 0 : v;
    }

    // ── File ─────────────────────────────────────────────────────────────────

    /** The summary as JSON, a few KB. Main thread: it reads every accumulator. */
    String toJson(MatchInfo match, double now) {
        Json j = new Json();
        j.open();
        j.field("version", VERSION);
        j.field("writtenAt", now);
        j.key("match").open();
        j.field("event", match.event());
        j.field("type", match.type());
        j.field("number", match.number());
        j.field("replay", match.replay());
        j.field("alliance", match.alliance());
        j.field("station", match.station());
        j.field("matchTimeAtEnd", match.matchTime());
        j.close();
        j.key("phases").open();
        j.field("autoStart", autoStart);
        j.field("autoEnd", autoEnd);
        j.field("teleopStart", teleopStart);
        j.field("teleopEnd", teleopEnd);
        j.close();
        j.key("battery").open();
        j.field("minVolts", minVolts);
        j.field("minAt", minVoltsAt);
        j.field("secondsUnder8V", secondsUnderLow);
        j.field("secondsUnder7V", secondsUnderVeryLow);
        j.close();
        j.key("brownouts").open();
        j.field("count", brownouts);
        j.field("seconds", brownoutSeconds);
        j.field("firstAt", firstBrownoutAt);
        j.close();
        j.key("loop").open();
        j.field("enabledLoops", enabledLoops);
        j.field("overruns", overruns);
        j.field("overrunMs", RobotLoop.periodSeconds() * OVERRUN_FACTOR * 1000);
        j.field("longestMs", longestLoopSeconds * 1000);
        j.field("longestAt", longestLoopAt);
        j.close();
        j.key("ds").open();
        j.field("drops", dsDrops);
        j.field("dropsWhileEnabled", dsDropsWhileEnabled);
        j.field("secondsWithout", secondsWithoutDs);
        j.strings("events", dsEvents);
        j.close();
        j.key("can").open();
        for (Bus b : buses) {
            j.key(b.label).open();
            j.field("devices", b.devices);
            j.field("drops", b.drops);
            j.field("downSeconds", b.downSeconds);
            j.strings("events", b.events);
            j.close();
        }
        j.close();
        j.key("devices").open();
        List<String> neverConnected = new ArrayList<>();
        for (Device d : devices) {
            if (!d.everConnected) {
                neverConnected.add(d.name);
            } else if (d.drops > 0 || d.reboots > 0) {
                j.key(d.name).open();
                j.field("bus", buses.get(d.bus).label);
                j.field("drops", d.drops);
                j.field("reboots", d.reboots);
                j.strings("events", d.events);
                if (d.eventsDropped > 0) {
                    j.field("moreEvents", d.eventsDropped);
                }
                j.close();
            }
        }
        j.close();
        j.strings("neverConnected", neverConnected);
        j.key("vision").open();
        for (VisionSource v : vision) {
            j.key(v.name).open();
            j.field("accepted", v.accepted.getAsLong());
            j.field("rejected", v.rejected.getAsLong());
            j.close();
        }
        j.close();
        j.key("auto").open();
        j.field("routine", autoRoutine);
        j.field("ended", autoEnded);
        j.field("seconds", autoSeconds);
        j.close();
        j.key("alerts").open();
        j.field("errorsRaised", errorsRaised);
        j.field("warningsRaised", warningsRaised);
        j.field("infosRaised", infosRaised);
        j.key("raised").array();
        for (var e : alertTexts.entrySet()) {
            AlertRecord r = e.getValue();
            j.open();
            j.field("level", r.level);
            j.field("firstAt", r.firstAt);
            j.field("count", r.count);
            j.field("text", e.getKey());
            j.close();
        }
        j.endArray();
        if (alertTextsDropped > 0) {
            j.field("moreTexts", alertTextsDropped);
        }
        j.close();
        j.key("peakTempC").open();
        for (Device d : devices) {
            if (d.tempCelsius != null && !Double.isNaN(d.peakTemp)) {
                j.field(d.name, d.peakTemp);
            }
        }
        j.close();
        j.close();
        return j.toString();
    }

    static String fmt(double seconds) {
        return String.format("%.2f", seconds);
    }

    /** Just enough JSON writing for a flat, small document. */
    static final class Json {
        private final StringBuilder sb = new StringBuilder(4096);
        private boolean first = true;

        /** Opens an object: after {@link #key}, or as the next element of an array. */
        Json open() {
            sep();
            sb.append('{');
            first = true;
            return this;
        }

        Json close() {
            sb.append('}');
            first = false;
            return this;
        }

        Json array() {
            sep();
            sb.append('[');
            first = true;
            return this;
        }

        Json endArray() {
            sb.append(']');
            first = false;
            return this;
        }

        private void sep() {
            if (!first) {
                sb.append(',');
            }
            first = false;
        }

        Json key(String k) {
            sep();
            string(k);
            sb.append(':');
            first = true; // the value that follows needs no comma
            return this;
        }

        void field(String k, String v) {
            key(k);
            string(v);
            first = false;
        }

        void field(String k, long v) {
            key(k);
            sb.append(v);
            first = false;
        }

        void field(String k, double v) {
            key(k);
            number(v);
            first = false;
        }

        void strings(String k, List<String> values) {
            key(k);
            sb.append('[');
            for (int i = 0; i < values.size(); i++) {
                if (i > 0) {
                    sb.append(',');
                }
                string(values.get(i));
            }
            sb.append(']');
            first = false;
        }

        private void number(double v) {
            if (Double.isNaN(v) || Double.isInfinite(v)) {
                sb.append("null");
            } else {
                sb.append(Math.round(v * 1000) / 1000.0);
            }
        }

        private void string(String s) {
            sb.append('"');
            for (int i = 0; i < s.length(); i++) {
                char c = s.charAt(i);
                switch (c) {
                    case '"' -> sb.append("\\\"");
                    case '\\' -> sb.append("\\\\");
                    case '\n' -> sb.append("\\n");
                    case '\r' -> sb.append("\\r");
                    case '\t' -> sb.append("\\t");
                    default -> {
                        if (c < 0x20) {
                            sb.append(String.format("\\u%04x", (int) c));
                        } else {
                            sb.append(c);
                        }
                    }
                }
            }
            sb.append('"');
        }

        @Override
        public String toString() {
            return sb.toString();
        }
    }
}
