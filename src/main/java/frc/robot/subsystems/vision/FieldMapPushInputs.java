package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * One Limelight's field-map push, as an AdvantageKit input ({@code Vision/FieldMapPush/<camera>}).
 * The push runs on a background thread; these are its results, fed back to the main thread.
 */
public class FieldMapPushInputs implements LoggableInputs {
    /** Whether pushing is switched on ({@code VisionConfig.pushFieldMapToLimelights}). */
    public boolean enabled = false;

    /** Pushes started this boot. */
    public long attempts = 0;

    /** Whether a push is running now. */
    public boolean inFlight = false;

    /** HTTP status of the last finished push: -1 before any, 0 when nothing answered. */
    public int httpStatus = -1;

    /** Canonical hash of the map the last push sent; empty before any. */
    public String attemptedHash = "";

    /** Canonical hash of the map a push delivered (HTTP 2xx) this boot; empty until one does. */
    public String confirmedHash = "";

    /** Why the last push failed, or empty. */
    public String error = "";

    @Override
    public void toLog(LogTable table) {
        table.put("Enabled", enabled);
        table.put("Attempts", attempts);
        table.put("InFlight", inFlight);
        table.put("HttpStatus", httpStatus);
        table.put("AttemptedHash", attemptedHash);
        table.put("ConfirmedHash", confirmedHash);
        table.put("Error", error);
    }

    @Override
    public void fromLog(LogTable table) {
        // Logs from before this was recorded have none: switched off, never pushed.
        enabled = table.get("Enabled", false);
        attempts = table.get("Attempts", 0L);
        inFlight = table.get("InFlight", false);
        httpStatus = table.get("HttpStatus", -1);
        attemptedHash = table.get("AttemptedHash", "");
        confirmedHash = table.get("ConfirmedHash", "");
        error = table.get("Error", "");
    }
}
