package frc.spectrumLib.telemetry;

import frc.spectrumLib.framework.RobotLoop;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.system.Timer;
import org.wpilib.units.Measure;
import org.wpilib.units.Unit;
import org.wpilib.util.WPISerializable;
import org.wpilib.util.struct.StructSerializable;

/**
 * Telemetry and logging facade, backed by AdvantageKit.
 *
 * <p>In 2026 this class extended DogLog. On SystemCore it keeps the same static API -- {@code log},
 * {@code logDash}, {@code logDashAlways}, {@code print}, {@code time}/{@code timeEnd}, {@code
 * slowLogThisLoop} -- so every existing call site compiles unchanged, but every value is now an
 * AdvantageKit <em>output</em> ({@link Logger#recordOutput}). Outputs land in the same wpilog as
 * the replayed inputs, under {@code /RealOutputs/...} on the robot and {@code /ReplayOutputs/...}
 * in replay, so a replayed match regenerates every one of these keys from the code under test.
 *
 * <h2>Rules that come with AdvantageKit</h2>
 *
 * <ul>
 *   <li><b>Main thread only.</b> {@link Logger} is not thread-safe. Nothing on a Notifier, the CTRE
 *       odometry thread or a vision callback may call this class; hand the data to the main loop
 *       and log it there (see {@code Swerve}'s odometry queue).
 *   <li><b>Outputs never drive logic.</b> Anything a decision depends on must come from a logged
 *       input ({@link Logger#processInputs}), or replay will diverge.
 * </ul>
 *
 * <h2>Tiers</h2>
 *
 * <ul>
 *   <li>{@link #log} records to the log. AdvantageKit's NT4 publisher also mirrors every output to
 *       NetworkTables on a real robot, which SystemCore has the CPU for (the 2026 roboRIO did not;
 *       see the offseason notes on 2026-09-05 CPU load).
 *   <li>{@link #logDash} / {@link #logDashAlways} are kept for call-site compatibility. They are
 *       the keys a dashboard reads, and are identical to {@link #log} here.
 *   <li>{@link #slowLogThisLoop()} is true every fifth loop. Wrap logs that do not need loop-rate
 *       resolution (currents, temperatures, vision status) in it.
 * </ul>
 */
public class Telemetry {

    /** Named fault conditions that can be surfaced as structured log entries. */
    public enum Fault {
        CAMERA_OFFLINE,
        AUTO_SHOT_TIMEOUT_TRIGGERED,
        BROWNOUT,
    }

    /**
     * Priority levels for printing to the console.
     *
     * <ul>
     *   <li>{@link #NORMAL} — only printed when the global priority is also {@code NORMAL}.
     *   <li>{@link #HIGH} — always printed regardless of the global priority setting.
     * </ul>
     */
    public enum PrintPriority {
        NORMAL,
        HIGH
    }

    /** Minimum priority level a message must have to be written to the console. */
    private static PrintPriority priority = PrintPriority.HIGH;

    /** Seconds between slow-tier publishes (10 Hz at any loop rate). */
    public static final double SLOW_LOG_PERIOD_SECONDS = 0.1;

    private Telemetry() {}

    /**
     * Returns {@code true} on the loops the slow telemetry tier publishes on. Every caller in the
     * same loop agrees, so related values land on the same records.
     *
     * @return whether slow-tier values should be logged this loop
     */
    public static boolean slowLogThisLoop() {
        return RobotLoop.everySeconds(SLOW_LOG_PERIOD_SECONDS);
    }

    /**
     * Sets the console print priority. Data receivers and the replay source are configured in
     * {@code Robot} before {@link Logger#start()}, not here.
     *
     * @param priority The minimum priority level for console output.
     */
    public static void start(PrintPriority priority) {
        Telemetry.priority = priority;
    }

    // ── Plain log ───────────────────────────────────────────────────────────

    public static void log(String key, double value) {
        Logger.recordOutput(key, value);
    }

    public static void log(String key, double value, String unit) {
        Logger.recordOutput(key, value, unit);
    }

    public static void log(String key, float value) {
        Logger.recordOutput(key, value);
    }

    public static void log(String key, boolean value) {
        Logger.recordOutput(key, value);
    }

    public static void log(String key, int value) {
        Logger.recordOutput(key, value);
    }

    public static void log(String key, long value) {
        Logger.recordOutput(key, value);
    }

    public static void log(String key, String value) {
        Logger.recordOutput(key, value);
    }

    public static void log(String key, double[] value) {
        Logger.recordOutput(key, value);
    }

    public static void log(String key, boolean[] value) {
        Logger.recordOutput(key, value);
    }

    public static void log(String key, long[] value) {
        Logger.recordOutput(key, value);
    }

    public static void log(String key, int[] value) {
        Logger.recordOutput(key, value);
    }

    public static void log(String key, String[] value) {
        Logger.recordOutput(key, value);
    }

    public static <E extends Enum<E>> void log(String key, E value) {
        Logger.recordOutput(key, value);
    }

    public static <T extends WPISerializable> void log(String key, T value) {
        Logger.recordOutput(key, value);
    }

    @SafeVarargs
    public static <T extends StructSerializable> void log(String key, T... value) {
        Logger.recordOutput(key, value);
    }

    public static <U extends Unit> void log(String key, Measure<U> value) {
        Logger.recordOutputMeasure(key, value);
    }

    // ── Dashboard tier ───────────────────────────────────────────────────────
    //
    // As in the 2026 offseason code, NetworkTables carries only what a dashboard shows; the log
    // file carries everything. logDash and logDashAlways record exactly like log, and also mark
    // the key as a dashboard key, which DashboardReceiver lets through to NetworkTables. Keys the
    // Elastic layout reads are marked at startup (see addDashboardKeysFromElasticLayout). The
    // mirror switch sends everything, for a live AdvantageScope session on the bench.

    /** Keys (relative to the outputs table, e.g. {@code "Hood/Voltage"}) published to NT. */
    private static final java.util.Set<String> dashboardKeys =
            java.util.concurrent.ConcurrentHashMap.newKeySet();

    /**
     * Dashboard key of the switch that mirrors every log entry to NetworkTables. Forced off when
     * the FMS is attached.
     */
    public static final String NT_MIRROR_SWITCH_KEY = "/SmartDashboard/Telemetry/MirrorLogsToNT";

    private static org.littletonrobotics.junction.networktables.LoggedNetworkBoolean mirrorSwitch;

    /** Read by the NT receiver thread; written once per loop on the main thread. */
    private static volatile boolean mirrorAll = false;

    /**
     * Creates the mirror switch. Call once, after the logger has started.
     *
     * @param mirrorByDefault initial position: on in simulation, off on the robot
     */
    public static void startDashboard(boolean mirrorByDefault) {
        mirrorSwitch =
                new org.littletonrobotics.junction.networktables.LoggedNetworkBoolean(
                        NT_MIRROR_SWITCH_KEY, mirrorByDefault);
        mirrorAll = mirrorByDefault;
    }

    /** Updates the mirror switch. Call once per loop. */
    public static void periodic() {
        if (mirrorSwitch != null) {
            mirrorAll = mirrorSwitch.get() && !org.wpilib.driverstation.RobotState.isFMSAttached();
        }
    }

    /** Whether every output is being mirrored to NetworkTables right now. */
    public static boolean isMirroringAll() {
        return mirrorAll;
    }

    /** Whether an output key (relative to the outputs table) is a dashboard key. */
    public static boolean isDashboardKey(String key) {
        return dashboardKeys.contains(key);
    }

    /** Marks a key for NetworkTables, e.g. one a dashboard layout reads. */
    public static void addDashboardKey(String key) {
        dashboardKeys.add(key);
    }

    /**
     * Marks every output the Elastic layout reads as a dashboard key, so what Elastic shows is
     * exactly what goes over NetworkTables and the two cannot drift apart. Topics under {@code
     * /AdvantageKit/RealOutputs/} become output keys.
     *
     * @param layoutJson the layout file's contents
     * @return how many keys were added
     */
    public static int addDashboardKeysFromElasticLayout(String layoutJson) {
        var m =
                java.util.regex.Pattern.compile(
                                "\"topic\"\\s*:\\s*\"/AdvantageKit/RealOutputs/([^\"]+)\"")
                        .matcher(layoutJson);
        int n = 0;
        while (m.find()) {
            if (dashboardKeys.add(m.group(1))) {
                n++;
            }
        }
        return n;
    }

    public static void logDash(String key, double value) {
        dashboardKeys.add(key);
        log(key, value);
    }

    public static void logDash(String key, double value, String unit) {
        dashboardKeys.add(key);
        log(key, value, unit);
    }

    public static void logDash(String key, boolean value) {
        dashboardKeys.add(key);
        log(key, value);
    }

    public static void logDash(String key, String value) {
        dashboardKeys.add(key);
        log(key, value);
    }

    public static void logDash(String key, long value) {
        dashboardKeys.add(key);
        log(key, value);
    }

    public static void logDashAlways(String key, double value) {
        logDash(key, value);
    }

    public static void logDashAlways(String key, double value, String unit) {
        logDash(key, value, unit);
    }

    public static void logDashAlways(String key, boolean value) {
        logDash(key, value);
    }

    public static void logDashAlways(String key, String value) {
        logDash(key, value);
    }

    public static void logDashAlways(String key, long value) {
        logDash(key, value);
    }

    // ── Loop timers ──────────────────────────────────────────────────────────

    /** Start times of open {@link #time} spans, in nanoseconds. */
    private static final Map<String, Long> epochStartNanos = new HashMap<>();

    /**
     * Starts a timed span. Pair with {@link #timeEnd(String)} on the same key.
     *
     * <p>Uses the real clock, not {@link Logger#getTimestamp()}, because the point is to measure
     * how long the code took. In replay these values describe the replay machine, not the robot.
     *
     * @param key the log key the elapsed time will be written to
     */
    public static void time(String key) {
        epochStartNanos.put(key, System.nanoTime());
    }

    /**
     * Ends a timed span and logs its length in seconds, so existing analysis of the {@code
     * Scheduler/*} keys keeps working.
     *
     * @param key the key passed to {@link #time(String)}
     */
    public static void timeEnd(String key) {
        Long start = epochStartNanos.remove(key);
        if (start == null) {
            return;
        }
        log(key, (System.nanoTime() - start) / 1e9, "seconds");
    }

    // ── Events ───────────────────────────────────────────────────────────────

    /** Loop whose events {@link #events} holds. */
    private static long eventLoop = -1;

    /** Events logged this loop, by key. */
    private static final Map<String, java.util.List<String>> events = new HashMap<>();

    /**
     * Logs a discrete event. AdvantageKit keeps one value per key per cycle, so logging two
     * messages to the same key in one loop kept only the second -- two commands ending on the same
     * loop, or two prints, lost one. Events are accumulated per loop and recorded as a string array
     * holding every one of them.
     *
     * @param key the log key
     * @param message the event
     */
    public static void logEvent(String key, String message) {
        long loop = RobotLoop.count();
        if (loop != eventLoop) {
            eventLoop = loop;
            events.clear();
        }
        var list = events.computeIfAbsent(key, k -> new java.util.ArrayList<>(2));
        list.add(message);
        Logger.recordOutput(key, list.toArray(new String[0]));
    }

    // ── Commands, prints ─────────────────────────────────────────────────────

    /**
     * Wraps a command so that its initialization and end are logged to the "Commands" key.
     *
     * @param cmd The command to wrap
     * @return a decorated command that logs lifecycle events and preserves the original name
     */
    public static Command log(Command cmd) {
        return cmd.deadlineFor(
                        Commands.startEnd(
                                () -> logEvent("Commands", "Init: " + cmd.getName()),
                                () -> logEvent("Commands", "End: " + cmd.getName())))
                .ignoringDisable(cmd.runsWhenDisabled())
                .withName(cmd.getName());
    }

    /**
     * Print a statement if the priority allows it. AdvantageKit captures the console into the log,
     * so the text is also recorded; it is additionally logged under "Prints" for search.
     */
    public static void print(String output, PrintPriority priority) {
        String out = "TIME: " + String.format("%.3f", Timer.getTimestamp()) + " || " + output;
        if (priority == PrintPriority.HIGH || Telemetry.priority == PrintPriority.NORMAL) {
            System.out.println(out);
        }
        logEvent("Prints", out);
    }

    /**
     * Prints a message at {@link PrintPriority#NORMAL} priority.
     *
     * @param output The string to print
     */
    public static void print(String output) {
        print(output, PrintPriority.NORMAL);
    }
}
