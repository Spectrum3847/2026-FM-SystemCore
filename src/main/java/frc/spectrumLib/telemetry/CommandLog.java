package frc.spectrumLib.telemetry;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;

/**
 * Logs what the command scheduler did: {@code Commands/Running}, every loop, and {@code
 * Commands/Events}, the starts, ends and interrupts of the loop they happened in ("start Name",
 * "end Name", "interrupt Name by Other"). The scheduler's callbacks run inside {@code run()} on the
 * main thread, so logging from them is safe.
 */
public final class CommandLog {
    private static final Map<Command, String> running = new LinkedHashMap<>();
    private static final List<String> events = new ArrayList<>();

    private CommandLog() {}

    /** Hooks the scheduler. Call once, before the first loop. */
    public static void start() {
        CommandScheduler s = CommandScheduler.getInstance();
        s.onCommandInitialize(
                c -> {
                    running.put(c, c.getName());
                    events.add("start " + c.getName());
                });
        s.onCommandFinish(
                c -> {
                    running.remove(c);
                    events.add("end " + c.getName());
                });
        s.onCommandInterrupt(
                (c, by) -> {
                    running.remove(c);
                    events.add(
                            "interrupt "
                                    + c.getName()
                                    + by.map(b -> " by " + b.getName()).orElse(""));
                });
    }

    /** Logs this loop's commands. Call once per loop, after the scheduler runs. */
    public static void periodic() {
        // Names of the commands running now.
        Telemetry.log("Commands/Running", running.values().toArray(new String[0]));
        // Empty on quiet loops so each event appears once, in the loop it happened.
        Telemetry.log("Commands/Events", events.toArray(new String[0]));
        events.clear();
    }
}
