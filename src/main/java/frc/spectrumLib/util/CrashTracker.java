package frc.spectrumLib.util;

import frc.spectrumLib.telemetry.Telemetry;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.nio.charset.StandardCharsets;
import java.time.Instant;
import java.util.UUID;
import org.wpilib.framework.RobotBase;

/** Tracks start-up and caught crash events, logging them to a file which doesn't roll over */
public class CrashTracker {

    private static final UUID RUN_INSTANCE_UUID = UUID.randomUUID();
    private static String filePath = "/home/systemcore/crash_tracking.txt";

    /**
     * Appends the run UUID, an "Exception" marker, the current time, and the stack trace to
     * '/home/systemcore/crash_tracking.txt'. Entries are appended, never overwritten.
     *
     * @param throwable The Throwable exception to log. Can be any subclass of Throwable, capturing
     *     errors and exceptions that occur during the application's execution.
     */
    public static void logThrowableCrash(Throwable throwable) {
        logMarker("Exception", throwable);
    }

    /** Logs a marker with no exception attached. */
    @SuppressWarnings("unused")
    private static void logMarker(String mark) {
        logMarker(mark, null);
    }

    /** Logs the marker. */
    private static void logMarker(String mark, Throwable nullableException) {
        try (PrintWriter writer =
                new PrintWriter(new FileWriter(filePath, StandardCharsets.UTF_8, true))) {
            writer.print(RUN_INSTANCE_UUID.toString());
            writer.print(", ");
            writer.print(mark);
            writer.print(", ");
            writer.print(Instant.now().toString());

            if (nullableException != null) {
                writer.print(", ");
                nullableException.printStackTrace(writer);
            }

            writer.println();
        } catch (IOException e) {
            if (e instanceof FileNotFoundException) {
                if (RobotBase.isSimulation()) {
                    Telemetry.print(
                            "CrashTracker failed to save crash file to robot: running in simulation mode");
                } else {
                    Telemetry.print(
                            "CrashTracker failed to save crash file to robot: path `/home/systemcore/crash_tracking.txt` not found");
                }
            } else {
                e.printStackTrace();
            }
        }
    }
}
