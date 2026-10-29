package frc.spectrumLib.telemetry;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.concurrent.CopyOnWriteArrayList;
import org.littletonrobotics.junction.Logger;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.StringArrayPublisher;
import org.wpilib.networktables.StringPublisher;
import org.wpilib.system.Timer;

/**
 * A WPILib {@link org.wpilib.driverstation.Alert} that is also logged and shown on Elastic.
 *
 * <p>WPILib 2027 alpha-6 sends alerts to the Driver Station through the HAL only. They are not
 * published to {@code /SmartDashboard/Alerts}, so Elastic's Alerts widget stays empty. There is no
 * API to list them, so AdvantageKit's alert logger never sees them either. This subclass keeps its
 * own registry: {@link #periodic()} records the active alerts as outputs ({@code Alerts/Errors},
 * {@code Alerts/Warnings}, {@code Alerts/Infos}, newest first, as the 2026 dashboard showed them)
 * and republishes them in the {@code /SmartDashboard/Alerts} group Elastic reads.
 *
 * <p>A drop-in replacement: same constructors and methods, and {@code Alert.Level} still refers to
 * WPILib's levels.
 */
public class Alert extends org.wpilib.driverstation.Alert {
    private static final List<Alert> ALL = new CopyOnWriteArrayList<>();

    private final Level level;
    private boolean active;
    private String text;
    private double activeSinceSeconds;

    private static NetworkTable table;
    private static StringArrayPublisher errors;
    private static StringArrayPublisher warnings;
    private static StringArrayPublisher infos;
    private static StringPublisher type;

    /**
     * Set when an alert turns on or off or changes text. {@link #periodic()} rebuilds and publishes
     * only then, since most loops nothing changes.
     */
    private static volatile boolean dirty = true;

    /**
     * Told when an alert turns on, for {@link MatchSummary}. Called from {@link #periodic()}, so an
     * alert that code clears and sets again within one loop is not a new one.
     */
    private static java.util.function.BiConsumer<Level, String> raisedListener;

    /** Whether this alert was on at the last {@link #periodic()}. */
    private boolean reported;

    /** Sets the one listener told when an alert turns on, with its level and text. */
    static void setRaisedListener(java.util.function.BiConsumer<Level, String> listener) {
        raisedListener = listener;
    }

    /**
     * Creates an alert in the default group.
     *
     * @param text the text shown while active
     * @param level its severity
     */
    public Alert(String text, Level level) {
        super(text, level);
        this.level = level;
        this.text = text;
        ALL.add(this);
    }

    /**
     * Creates an alert in a named group.
     *
     * @param group the group (kept for WPILib compatibility; Elastic shows one list)
     * @param text the text shown while active
     * @param level its severity
     */
    public Alert(String group, String text, Level level) {
        super(group, text, level);
        this.level = level;
        this.text = text;
        ALL.add(this);
    }

    @Override
    public void set(boolean on) {
        if (on && !active) {
            activeSinceSeconds = Timer.getTimestamp();
        }
        if (on != active) {
            dirty = true;
        }
        active = on;
        super.set(on);
    }

    @Override
    public void setText(String newText) {
        if (active && !newText.equals(text)) {
            dirty = true;
        }
        text = newText;
        super.setText(newText);
    }

    @Override
    public void close() {
        dirty = true;
        ALL.remove(this);
        super.close();
    }

    private static String[] active(Level level) {
        List<Alert> list = new ArrayList<>();
        for (Alert a : ALL) {
            if (a.active && a.level == level) {
                list.add(a);
            }
        }
        list.sort(Comparator.comparingDouble((Alert a) -> a.activeSinceSeconds).reversed());
        String[] out = new String[list.size()];
        for (int i = 0; i < out.length; i++) {
            out[i] = list.get(i).text;
        }
        return out;
    }

    /**
     * Logs the active alerts and republishes them for Elastic when they change. Call every loop.
     */
    public static void periodic() {
        if (!dirty) {
            return; // The log and NetworkTables keep the last published values.
        }
        dirty = false;
        var listener = raisedListener;
        for (Alert a : ALL) {
            if (a.active && !a.reported) {
                Telemetry.logEvent("Events", "alert " + a.level + ": " + a.text);
                if (listener != null) {
                    listener.accept(a.level, a.text);
                }
            } else if (!a.active && a.reported) {
                Telemetry.logEvent("Events", "alert cleared: " + a.text);
            }
            a.reported = a.active;
        }
        String[] e = active(Level.HIGH);
        String[] w = active(Level.MEDIUM);
        String[] i = active(Level.LOW);
        Logger.recordOutput("Alerts/Errors", e);
        Logger.recordOutput("Alerts/Warnings", w);
        Logger.recordOutput("Alerts/Infos", i);
        if (table == null) {
            table = NetworkTableInstance.getDefault().getTable("SmartDashboard/Alerts");
            type = table.getStringTopic(".type").publish();
            type.set("Alerts");
            errors = table.getStringArrayTopic("errors").publish();
            warnings = table.getStringArrayTopic("warnings").publish();
            infos = table.getStringArrayTopic("infos").publish();
        }
        errors.set(e);
        warnings.set(w);
        infos.set(i);
    }
}
