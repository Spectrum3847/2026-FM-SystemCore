package frc.robot.subsystems.vision;

import frc.robot.Constants;
import frc.spectrumLib.telemetry.Alert;
import frc.spectrumLib.vision.FieldMapHash;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.wpilib.fields.Field;
import org.wpilib.fields.Fields;
import org.wpilib.util.Alert.Level;

/**
 * Which AprilTag field the robot solves with, as a logged input ({@code Vision/FieldMap/Name} and
 * {@code Hash}). The Orin poses are solved on the robot from logged detections, so a replay
 * re-solves them with whatever layout it loads: it must load the match's field, not whatever {@code
 * Event} says today.
 */
public final class TagField {
    private TagField() {}

    /** What the field map input holds when the log has none (logs from before it was recorded). */
    static final String ABSENT = "";

    /**
     * The field vision solves with.
     *
     * @param field the WPILib field
     * @param layout its layout
     * @param hash its canonical hash ({@link FieldMapHash})
     * @param warning why a replay may not solve as the match did, or null
     */
    public record Choice(Fields field, Field layout, String hash, String warning) {}

    /** The input: written on the robot and in the sim, read back in replay. */
    static final class Inputs implements LoggableInputs {
        String name = ABSENT;
        String hash = ABSENT;

        @Override
        public void toLog(LogTable table) {
            // The AprilTag field map the robot solves with: the WPILib field's name, or custom.
            table.put("Name", name);
            // Its canonical hash, to compare with the Jetson's Vision/Jetson/FieldMap/Hash.
            table.put("Hash", hash);
        }

        @Override
        public void fromLog(LogTable table) {
            name = table.get("Name", ABSENT);
            hash = table.get("Hash", ABSENT);
        }
    }

    private static final Alert replayAlert = new Alert("", Level.MEDIUM);

    /**
     * Chooses the field and records it. Call once, during init (the input lands in the first cycle,
     * where a replay reads it back before the first loop).
     *
     * @param eventField this event's field ({@code Event.APRILTAG_FIELD})
     * @return the field to solve with
     */
    public static Choice load(Fields eventField) {
        Inputs inputs = new Inputs();
        boolean replay = Constants.currentMode == Constants.Mode.REPLAY;
        Choice choice;
        if (replay) {
            Logger.processInputs("Vision/FieldMap", inputs);
            choice = forReplay(eventField, inputs.name, inputs.hash);
        } else {
            choice = of(eventField, null);
            inputs.name = choice.field().name();
            inputs.hash = choice.hash();
            Logger.processInputs("Vision/FieldMap", inputs);
        }
        // An output, so a replay's log shows what the replay solved with.
        Logger.recordOutput("Vision/FieldMap/Solving", choice.field().name());
        if (choice.warning() != null) {
            replayAlert.setText(choice.warning());
            replayAlert.set(true);
        }
        return choice;
    }

    /**
     * The field a replay solves with: the logged one if this code knows it, else the event's.
     *
     * @param eventField {@code Event.APRILTAG_FIELD} as this code has it
     * @param loggedName the logged field name, {@link #ABSENT} in older logs
     * @param loggedHash the logged canonical hash, {@link #ABSENT} in older logs
     * @return the field, with a warning when the replay may solve differently from the match
     */
    static Choice forReplay(Fields eventField, String loggedName, String loggedHash) {
        if (loggedName.equals(ABSENT)) {
            return of(
                    eventField,
                    "Replay: the log has no field map (recorded before it was logged), so the Orin"
                            + " re-solves with Event's "
                            + eventField.name()
                            + "; replay may differ from the match");
        }
        Fields logged = parse(loggedName);
        if (logged == null) {
            return of(
                    eventField,
                    "Replay: the log's field map "
                            + loggedName
                            + " is not a WPILib field, so the Orin re-solves with Event's "
                            + eventField.name()
                            + "; replay may differ from the match");
        }
        Choice c = of(logged, null);
        if (!loggedHash.equals(ABSENT) && !loggedHash.equals(c.hash())) {
            return of(
                    logged,
                    "Replay: "
                            + logged.name()
                            + " here hashes "
                            + c.hash()
                            + ", the match's hashed "
                            + loggedHash
                            + " (WPILib's layout changed); replay may differ from the match");
        }
        return c;
    }

    private static Choice of(Fields field, String warning) {
        Field layout = Field.loadField(field);
        return new Choice(field, layout, FieldMapHash.of(layout), warning);
    }

    /** The 2026 fields under their alpha-6 enum names, so logs from then still replay. */
    private static final java.util.Map<String, Fields> ALPHA6_NAMES =
            java.util.Map.of(
                    "k2026RebuiltAndymark", Fields.FRC_2026_REBUILT_ANDY_MARK,
                    "k2026RebuiltWelded", Fields.FRC_2026_REBUILT_WELDED);

    private static Fields parse(String name) {
        for (Fields f : Fields.values()) {
            if (f.name().equals(name)) {
                return f;
            }
        }
        return ALPHA6_NAMES.get(name);
    }
}
