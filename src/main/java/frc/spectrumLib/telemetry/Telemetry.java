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

    /** Loops between slow-tier publishes: every fifth 20 ms loop is 10 Hz. */
    public static final int SLOW_LOG_EVERY_LOOPS = 5;

    private Telemetry() {}

    /**
     * Returns {@code true} on the loops the slow telemetry tier publishes on. Every caller in the
     * same loop agrees, so related values land on the same records.
     *
     * @return whether slow-tier values should be logged this loop
     */
    public static boolean slowLogThisLoop() {
        return RobotLoop.every(SLOW_LOG_EVERY_LOOPS);
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

    // ── Dashboard tier (identical to log on AdvantageKit) ────────────────────

    public static void logDash(String key, double value) {
        log(key, value);
    }

    public static void logDash(String key, double value, String unit) {
        log(key, value, unit);
    }

    public static void logDash(String key, boolean value) {
        log(key, value);
    }

    public static void logDash(String key, String value) {
        log(key, value);
    }

    public static void logDash(String key, long value) {
        log(key, value);
    }

    public static void logDashAlways(String key, double value) {
        log(key, value);
    }

    public static void logDashAlways(String key, double value, String unit) {
        log(key, value, unit);
    }

    public static void logDashAlways(String key, boolean value) {
        log(key, value);
    }

    public static void logDashAlways(String key, String value) {
        log(key, value);
    }

    public static void logDashAlways(String key, long value) {
        log(key, value);
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
                                () -> log("Commands", "Init: " + cmd.getName()),
                                () -> log("Commands", "End: " + cmd.getName())))
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
        log("Prints", out);
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
