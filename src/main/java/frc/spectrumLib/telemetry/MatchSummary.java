package frc.spectrumLib.telemetry;

import frc.spectrumLib.framework.RobotLoop;
import frc.spectrumLib.hardware.CanBusInputs;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.mechanism.MotorInputs;
import java.io.File;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.LongSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.conduit.ConduitApi;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.MatchType;
import org.wpilib.driverstation.RobotState;
import org.wpilib.system.Timer;
import org.wpilib.util.Alert.Level;

/**
 * A short summary of the run, kept as it happens: battery, brownouts, loop overruns, CAN drops and
 * reboots, Driver Station drops, vision counts, how auto went, alerts raised and peak motor
 * temperatures. Every value is logged under {@code Summary/...} once a second, in every log.
 *
 * <p>In an FMS match (match type not {@code NONE}) the summary is also written as {@code <log
 * name>.summary.json} next to the log, so the pit laptop and the log site can read it without
 * parsing the log. It is written when auto ends, when the Driver Station drops, on an E-stop and
 * when teleop ends, and each write replaces the last: the matches most worth a summary are the ones
 * that never reach the end of teleop. Power is usually cut on the way off the field, so the file is
 * written at once: the JSON is built on the main thread and a background thread writes it (tmp
 * file, fsync, rename, folder fsync). Practice runs get the log keys only. The keys and the file
 * are documented in {@code docs/match-summary.md}.
 *
 * <p>Everything is fed from logged inputs (motor inputs, the Driver Station, the robot clock,
 * battery and brownout inputs the robot passes in, the controller's brownout fault count logged
 * here as {@code SummaryInputs/Brownout}), so a replayed log recomputes the same summary. The scope
 * is the whole log: one log is one boot, and the robot is powered off between matches.
 *
 * <p>Main thread only, apart from the file write. Per loop it is a few compares per registered
 * device; logging is once a second.
 */
public final class MatchSummary {
    /** Bumped when a field changes meaning or is removed. Adding a field does not bump it. */
    public static final int VERSION = 2;

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

    /** Texts kept for alerts raised before the first enable, apart from the match's own. */
    static final int MAX_BEFORE_ENABLE_TEXTS = 10;

    /**
     * How {@code PreMatchChecks} starts every alert it raises. Their text carries live numbers (a
     * voltage, a distance), so each reading is a new text; they are counted, never listed.
     */
    static final String PRE_MATCH_PREFIX = "Pre-match";

    /** Device names in the file are for people: nothing in the summary is keyed on a CAN ID. */
    private static final class Device {
        final String name;
        final int bus;
        final BooleanSupplier connected;
        final BooleanSupplier reset;
        final DoubleSupplier tempCelsius;
        final Supplier<String[]> troubleFaults;
        final String peakTempKey;

        boolean everConnected;
        boolean lastConnected;
        int drops;
        int reboots;
        double peakTemp = Double.NaN;

        /** Every trouble fault seen while connected, in the order first seen. */
        final Set<String> faults = new LinkedHashSet<>();

        final List<String> events = new ArrayList<>(2);
        int eventsDropped;

        Device(
                String name,
                int bus,
                BooleanSupplier connected,
                BooleanSupplier reset,
                DoubleSupplier tempCelsius,
                Supplier<String[]> troubleFaults) {
            this.name = name;
            this.bus = bus;
            this.connected = connected;
            this.reset = reset;
            this.tempCelsius = tempCelsius;
            this.troubleFaults = troubleFaults;
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

        // From the bus's own counters (CanBusInputs), where they are read.
        long firstBusOffCount = -1;
        long busOffs;
        long peakTransmitErrors;
        long peakReceiveErrors;
        final String busOffsKey;

        Bus(String label) {
            this.label = label;
            this.dropsKey = "Summary/CAN/" + label + "/Drops";
            this.downSecondsKey = "Summary/CAN/" + label + "/DownSeconds";
            this.busOffsKey = "Summary/CAN/" + label + "/BusOffs";
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

    /** Error and warning alerts raised in one part of the run, each text listed once. */
    private static final class AlertTally {
        final int maxTexts;
        int errors;
        int warnings;
        int infos;
        final Map<String, AlertRecord> texts = new LinkedHashMap<>();
        int textsDropped;

        AlertTally(int maxTexts) {
            this.maxTexts = maxTexts;
        }

        /** Returns whether the list of texts changed. */
        boolean add(Level level, String text, double now) {
            String name;
            switch (level) {
                case HIGH -> {
                    errors++;
                    name = "error";
                }
                case MEDIUM -> {
                    warnings++;
                    name = "warning";
                }
                default -> {
                    infos++;
                    return false; // counted, not listed
                }
            }
            AlertRecord r = texts.get(text);
            if (r == null) {
                if (texts.size() >= maxTexts) {
                    textsDropped++;
                    return false;
                }
                r = new AlertRecord(name, now);
                texts.put(text, r);
            }
            r.count++;
            return true;
        }

        void raisedToJson(Json j) {
            j.key("raised").array();
            for (var e : texts.entrySet()) {
                AlertRecord r = e.getValue();
                j.open();
                j.field("level", r.level);
                j.field("firstAt", r.firstAt);
                j.field("count", r.count);
                j.field("text", e.getKey());
                j.close();
            }
            j.endArray();
            if (textsDropped > 0) {
                j.field("moreTexts", textsDropped);
            }
        }
    }

    /** An alert that turned on, held until this loop's enable state is known. */
    private record PendingAlert(Level level, String text) {}

    /**
     * The brownout inputs, logged so a replay counts the same brownouts: AdvantageKit's controller
     * fault count, and whether the robot's browned-out flag comes from an API that works (it did
     * not on the 2026-09 SystemCore bench unit, where the flag read false always).
     */
    static final class BrownoutInputs implements LoggableInputs {
        /** Where AdvantageKit logs the fault count, for logs recorded before this input. */
        static final String SYSTEM_STATS_KEY = "/SystemStats/FaultCounts/Brownout";

        /** The controller's brownout fault count, or -1 where there is none. */
        long faultCount = -1;

        boolean brownedOutValid;

        @Override
        public void toLog(LogTable table) {
            // The controller's brownout fault count, or -1 where there is none.
            table.put("FaultCount", faultCount);
            // The brownout flag API works on this controller, so the flag counts too.
            table.put("BrownedOutValid", brownedOutValid);
        }

        @Override
        public void fromLog(LogTable table) {
            LogTable.LogValue own = table.get("FaultCount");
            if (own != null) {
                faultCount = own.getInteger();
            } else {
                // Older logs: AdvantageKit's own input, which it records at the end of each cycle.
                LogTable.LogValue stats = table.getAll(false).get(SYSTEM_STATS_KEY);
                faultCount =
                        stats != null && stats.type == LogTable.LoggableType.Integer
                                ? stats.getInteger()
                                : -1;
            }
            brownedOutValid = table.get("BrownedOutValid", false);
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
    private Supplier<Map<String, CanBusInputs.Bus>> canBuses;
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

    /** Filled and logged in {@link #periodic}; tests set it directly. */
    final BrownoutInputs brownoutInputs = new BrownoutInputs();

    private boolean brownedOutAvailable;
    private boolean lastBrownedOut;
    private int flagBrownouts;
    private int faultCountReadings;
    private long faultCountAtStart = -1;
    private long faultBrownouts = -1;
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
    private String autoEnded = "";
    private double autoSeconds = Double.NaN;

    /** Alerts raised once the robot had been enabled: the match's own. */
    private final AlertTally inMatch = new AlertTally(MAX_ALERT_TEXTS);

    /** Alerts raised before the first enable: boot, the queue, the field before auto. */
    private final AlertTally beforeFirstEnable = new AlertTally(MAX_BEFORE_ENABLE_TEXTS);

    private int preMatchChecksRaised;
    private final List<PendingAlert> pendingAlerts = new ArrayList<>();
    private boolean everEnabled;

    private boolean lastEStopped;
    private boolean lastFmsMatch;
    private boolean alertTextsChanged;

    private int matchesEnded;

    /** Last file write result, logged on the main thread. */
    private String loggedFileResult = "";

    /**
     * Creates the summary and starts listening for alerts. Create it early in the robot's
     * constructor, on the main thread, so alerts raised while the robot starts are counted, and
     * before the main thread is made real-time, since the file writer's thread starts here.
     *
     * @param logFolder the folder the log is written to, for the file; null for no file (replay)
     */
    public MatchSummary(File logFolder) {
        this.writer = logFolder == null ? null : new SummaryFileWriter(logFolder);
        Alert.setRaisedListener(this::alertRaised);
    }

    /**
     * Says whether the browned-out flag passed to {@link #periodic} comes from an API that works
     * here. Until this is called it is taken not to, and brownouts are counted only from the
     * controller's fault count, or not at all.
     */
    public void setBrownedOutAvailable(boolean available) {
        brownedOutAvailable = available;
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
        addDevice(name, busLabel, connected, reset, tempCelsius, null);
    }

    /**
     * Tracks a CAN device as {@link #addDevice(String, String, BooleanSupplier, BooleanSupplier,
     * DoubleSupplier)} does, plus the sticky faults it reports that mean trouble.
     *
     * @param name shown in the summary
     * @param busLabel the bus, as its health is logged (e.g. {@code CANivore})
     * @param connected the device's logged connected input
     * @param reset its logged reboot input, or null
     * @param tempCelsius its logged temperature, or null
     * @param troubleFaults its logged trouble fault names ({@link MotorInputs#troubleFaults}), or
     *     null
     */
    public void addDevice(
            String name,
            String busLabel,
            BooleanSupplier connected,
            BooleanSupplier reset,
            DoubleSupplier tempCelsius,
            Supplier<String[]> troubleFaults) {
        devices.add(new Device(name, bus(busLabel), connected, reset, tempCelsius, troubleFaults));
    }

    /**
     * Tracks each bus's own error counters: bus-off events and the peak transmit and receive error
     * counts. Buses are matched to the devices' buses by label.
     *
     * @param canBuses the logged CAN bus inputs, by label
     */
    public void addCanBuses(Supplier<Map<String, CanBusInputs.Bus>> canBuses) {
        this.canBuses = canBuses;
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
                () -> in.tempCelsius,
                () -> in.troubleFaults);
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
     * Records the auto routine that just started. Call from {@code autonomousInit()}.
     *
     * @param routine its name
     */
    public void autoStarted(String routine) {
        autoRoutine = routine;
        autoEnded = "";
        autoSeconds = Double.NaN;
    }

    /**
     * Records why the auto ended, as the robot's own auto tracking logged it, so the summary and
     * that log agree. Main thread.
     *
     * @param reason the logged reason, recorded as is
     * @param seconds how long the auto ran
     */
    public void autoEnded(String reason, double seconds) {
        autoEnded = reason;
        autoSeconds = seconds;
    }

    // ── Per loop ─────────────────────────────────────────────────────────────

    /**
     * Advances every accumulator, logs the summary once a second, and on a loop that calls for the
     * file (see {@link Write}) hands it to the writer. Call once per loop, after the command
     * scheduler, so this loop's motor inputs have been read and the auto's end has been reported.
     *
     * @param batteryVolts the logged battery voltage
     * @param brownedOut the logged brownout state (see {@link #setBrownedOutAvailable})
     */
    public void periodic(double batteryVolts, boolean brownedOut) {
        double now = Timer.getTimestamp();
        if (!Logger.hasReplaySource()) {
            brownoutInputs.faultCount = readBrownoutFaultCount();
            brownoutInputs.brownedOutValid = brownedOutAvailable;
        }
        Logger.processInputs("SummaryInputs/Brownout", brownoutInputs);
        Write write =
                update(
                        now,
                        batteryVolts,
                        brownedOut,
                        RobotState.isDSAttached(),
                        RobotState.isEnabled(),
                        RobotState.isAutonomous(),
                        RobotState.isTeleop(),
                        RobotState.isEStopped(),
                        MatchState.getMatchType() != MatchType.NONE);
        if (write != null || RobotLoop.everySeconds(LOG_PERIOD_SECONDS)) {
            log();
        }
        if (write != null && writer != null) {
            writer.submit(toJson(matchInfo(), now, write));
        }
    }

    /** What AdvantageKit logs as {@code SystemStats/FaultCounts/Brownout}, or -1 if unreadable. */
    private static long readBrownoutFaultCount() {
        try {
            return ConduitApi.getInstance().getFaultCountBrownout();
        } catch (RuntimeException | LinkageError e) {
            return -1;
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

    /** Why the file was written; in the file as {@code writtenOn}. */
    enum Write {
        /** An E-stop: the robot is done for the match. */
        E_STOP("eStop"),
        /** The Driver Station stopped answering: the robot may be dead from here on. */
        DS_LOST("dsLost"),
        /** Teleop was disabled: the end of the match, or a comm drop within it. */
        TELEOP_END("teleopEnd"),
        /** Auto was disabled: the first summary of the match, in case nothing follows. */
        AUTO_END("autoEnd");

        final String key;

        Write(String key) {
            this.key = key;
        }
    }

    /**
     * Advances the accumulators by one loop.
     *
     * @return why the file is due this loop, or null; only ever in an FMS match
     */
    Write update(
            double now,
            double batteryVolts,
            boolean brownedOut,
            boolean dsAttached,
            boolean enabled,
            boolean autonomous,
            boolean teleop,
            boolean eStopped,
            boolean fmsMatch) {
        double dt = Double.isNaN(lastNow) ? 0 : Math.max(0, now - lastNow);
        double step = Math.min(dt, MAX_STEP_SECONDS);
        lastNow = now;

        // Alert.periodic() reports, at the top of a loop, alerts set during the last one: judged
        // by whether the robot had been enabled before this loop.
        for (PendingAlert a : pendingAlerts) {
            if (a.text().startsWith(PRE_MATCH_PREFIX)) {
                preMatchChecksRaised++;
            } else if (everEnabled) {
                alertTextsChanged |= inMatch.add(a.level(), a.text(), now);
            } else {
                beforeFirstEnable.add(a.level(), a.text(), now);
            }
        }
        pendingAlerts.clear();
        everEnabled |= enabled;

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

        updateBrownouts(now, step, brownedOut);

        boolean dsLost = dsSeen && lastDs && !dsAttached;
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
        boolean autoPeriodEnded = !autoEnabled && lastAutoEnabled;
        if (autoPeriodEnded) {
            autoEnd = now;
        }
        if (teleopEnabled && !lastTeleopEnabled) {
            teleopStart = now;
        }
        boolean teleopEnded = !teleopEnabled && lastTeleopEnabled;
        if (teleopEnded) {
            teleopEnd = now;
            if (fmsMatch) {
                matchesEnded++;
            }
        }
        boolean eStop = eStopped && !lastEStopped;
        // The match type may go with the Driver Station when it drops, so last loop's counts too.
        boolean inFmsMatch = fmsMatch || lastFmsMatch;
        lastAutoEnabled = autoEnabled;
        lastTeleopEnabled = teleopEnabled;
        lastEStopped = eStopped;
        lastFmsMatch = fmsMatch;
        if (!inFmsMatch) {
            return null;
        }
        if (eStop) {
            return Write.E_STOP;
        }
        if (dsLost) {
            return Write.DS_LOST;
        }
        if (teleopEnded) {
            return Write.TELEOP_END;
        }
        return autoPeriodEnded ? Write.AUTO_END : null;
    }

    /**
     * Counts brownouts from the controller's fault count where there is one, else from edges of the
     * browned-out flag where that API works; the flag also gives the time spent browned out.
     */
    private void updateBrownouts(double now, double step, boolean brownedOut) {
        long faultCount = brownoutInputs.faultCount;
        // The first loop runs before AdvantageKit first captures the controller's data, so its
        // count reads zero rather than the count since power-on.
        if (faultCount >= 0 && ++faultCountReadings > 1) {
            if (faultCountAtStart < 0) {
                faultCountAtStart = faultCount;
            }
            long n = Math.max(0, faultCount - faultCountAtStart);
            if (n > Math.max(0, faultBrownouts) && Double.isNaN(firstBrownoutAt)) {
                firstBrownoutAt = now;
            }
            faultBrownouts = n;
        }
        boolean flag = brownoutInputs.brownedOutValid && brownedOut;
        if (flag) {
            brownoutSeconds += step;
            if (!lastBrownedOut) {
                flagBrownouts++;
                if (Double.isNaN(firstBrownoutAt)) {
                    firstBrownoutAt = now;
                }
            }
        }
        lastBrownedOut = flag;
    }

    /** Where the brownout count comes from: {@code faultCount}, {@code brownedOut}, or null. */
    private String brownoutSource() {
        if (faultBrownouts >= 0) {
            return "faultCount";
        }
        return brownoutInputs.brownedOutValid ? "brownedOut" : null;
    }

    /** Brownouts in this log, or -1 where nothing working can tell. */
    long brownoutCount() {
        if (faultBrownouts >= 0) {
            return faultBrownouts;
        }
        return brownoutInputs.brownedOutValid ? flagBrownouts : -1;
    }

    private void updateBusCounters(double now, String label, CanBusInputs.Bus reading) {
        Integer i = busIndex.get(label);
        // A failed status read zeroes every counter.
        if (i == null || !reading.statusOk()) {
            return;
        }
        Bus b = buses.get(i);
        if (b.firstBusOffCount < 0) {
            b.firstBusOffCount = reading.busOffCount();
        }
        long busOffs = reading.busOffCount() - b.firstBusOffCount;
        if (busOffs > b.busOffs) {
            b.busOffs = busOffs;
            b.events.add(fmt(now) + " bus off");
        }
        b.peakTransmitErrors = Math.max(b.peakTransmitErrors, reading.transmitErrorCount());
        b.peakReceiveErrors = Math.max(b.peakReceiveErrors, reading.receiveErrorCount());
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
            // A disconnected device's readings are stale or zeroed.
            if (c && d.troubleFaults != null) {
                for (String f : d.troubleFaults.get()) {
                    if (d.faults.add(f)) {
                        d.event(now, "fault " + f);
                    }
                }
            }
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
        if (canBuses != null) {
            canBuses.get().forEach((label, reading) -> updateBusCounters(now, label, reading));
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

    /**
     * Called by {@link Alert#periodic()} for each alert that turned on since the last loop. Held
     * until the next {@link #update}, which sorts it: pre-match check results are counted only,
     * alerts from before the first enable are kept apart, and the rest are the match's.
     */
    void alertRaised(Level level, String text) {
        if (Thread.currentThread() != mainThread) {
            return; // the main thread owns this; an off-thread alert still shows on the DS
        }
        pendingAlerts.add(new PendingAlert(level, text));
    }

    // ── Log keys ─────────────────────────────────────────────────────────────

    private void log() {
        Logger.recordOutput("Summary/Battery/MinVolts", nanToZero(minVolts), "volts");
        Logger.recordOutput("Summary/Battery/SecondsUnder8V", secondsUnderLow, "seconds");
        Logger.recordOutput("Summary/Battery/SecondsUnder7V", secondsUnderVeryLow, "seconds");
        Logger.recordOutput("Summary/Brownouts", brownoutCount());
        Logger.recordOutput("Summary/Loop/EnabledLoops", enabledLoops);
        Logger.recordOutput("Summary/Loop/Overruns", overruns);
        Logger.recordOutput("Summary/Loop/LongestMs", longestLoopSeconds * 1000);
        Logger.recordOutput("Summary/DS/Drops", dsDrops);
        Logger.recordOutput("Summary/DS/DropsWhileEnabled", dsDropsWhileEnabled);
        Logger.recordOutput("Summary/DS/SecondsWithout", secondsWithoutDs);
        int deviceDrops = 0;
        int reboots = 0;
        int faulted = 0;
        for (Device d : devices) {
            deviceDrops += d.drops;
            reboots += d.reboots;
            if (!d.faults.isEmpty()) {
                faulted++;
            }
            if (d.peakTempKey != null && !Double.isNaN(d.peakTemp)) {
                // Summary/PeakTempC/<device>: hottest reading while the motor was connected.
                Logger.recordOutput(d.peakTempKey, d.peakTemp);
            }
        }
        Logger.recordOutput("Summary/Devices/Drops", deviceDrops);
        Logger.recordOutput("Summary/Devices/Reboots", reboots);
        Logger.recordOutput("Summary/Devices/Faulted", faulted);
        for (Bus b : buses) {
            Logger.recordOutput(b.busOffsKey, b.busOffs);
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
        Logger.recordOutput("Summary/Alerts/ErrorsRaised", inMatch.errors);
        Logger.recordOutput("Summary/Alerts/WarningsRaised", inMatch.warnings);
        Logger.recordOutput(
                "Summary/Alerts/BeforeFirstEnableRaised",
                beforeFirstEnable.errors + beforeFirstEnable.warnings);
        Logger.recordOutput("Summary/Alerts/PreMatchChecksRaised", preMatchChecksRaised);
        if (alertTextsChanged) {
            alertTextsChanged = false;
            String[] raised = new String[inMatch.texts.size()];
            int i = 0;
            for (var e : inMatch.texts.entrySet()) {
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
    String toJson(MatchInfo match, double now, Write write) {
        Json j = new Json();
        j.open();
        j.field("version", VERSION);
        j.field("writtenAt", now);
        j.field("writtenOn", write.key);
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
        long count = brownoutCount();
        if (count < 0) {
            j.field("count", (String) null);
        } else {
            j.field("count", count);
        }
        String source = brownoutSource();
        j.field("source", source);
        j.field("seconds", brownoutInputs.brownedOutValid ? brownoutSeconds : Double.NaN);
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
            j.field("busOffs", b.busOffs);
            j.field("peakTransmitErrors", b.peakTransmitErrors);
            j.field("peakReceiveErrors", b.peakReceiveErrors);
            j.strings("events", b.events);
            j.close();
        }
        j.close();
        j.key("devices").open();
        List<String> neverConnected = new ArrayList<>();
        for (Device d : devices) {
            if (!d.everConnected) {
                neverConnected.add(d.name);
            } else if (d.drops > 0 || d.reboots > 0 || !d.faults.isEmpty()) {
                j.key(d.name).open();
                j.field("bus", buses.get(d.bus).label);
                j.field("drops", d.drops);
                j.field("reboots", d.reboots);
                if (!d.faults.isEmpty()) {
                    j.strings("faults", new ArrayList<>(d.faults));
                }
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
        if (autoEnded.isEmpty()) {
            j.nullField("ended");
        } else {
            j.field("ended", autoEnded);
        }
        j.field("seconds", autoSeconds);
        j.close();
        j.key("alerts").open();
        j.field("errorsRaised", inMatch.errors);
        j.field("warningsRaised", inMatch.warnings);
        j.field("infosRaised", inMatch.infos);
        inMatch.raisedToJson(j);
        j.key("beforeFirstEnable").open();
        j.field("errorsRaised", beforeFirstEnable.errors);
        j.field("warningsRaised", beforeFirstEnable.warnings);
        j.field("infosRaised", beforeFirstEnable.infos);
        beforeFirstEnable.raisedToJson(j);
        j.close();
        j.field("preMatchChecksRaised", preMatchChecksRaised);
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

        /** A string field; null writes {@code null}. */
        void field(String k, String v) {
            key(k);
            if (v == null) {
                sb.append("null");
            } else {
                string(v);
            }
            first = false;
        }

        void nullField(String k) {
            key(k);
            sb.append("null");
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
