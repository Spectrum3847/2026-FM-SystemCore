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
import org.wpilib.util.Alert.Level;

/**
 * A WPILib {@link org.wpilib.util.Alert} that is also logged and shown on Elastic.
 *
 * <p>WPILib 2027 alpha-6 sends alerts to the Driver Station through the HAL only. They are not
 * published to {@code /SmartDashboard/Alerts}, so Elastic's Alerts widget stays empty. There is no
 * API to list them, so AdvantageKit's alert logger never sees them either. This subclass keeps its
 * own registry: {@link #periodic()} records the active alerts as outputs ({@code Alerts/Errors},
 * {@code Alerts/Warnings}, {@code Alerts/Infos}, newest first, as the 2026 dashboard showed them)
 * and republishes them in the {@code /SmartDashboard/Alerts} group Elastic reads.
 *
 * <p>The WPILib alert behind it is created the first time this one turns on. AdvantageKit copies
 * every registered alert out of native code every loop, active or not, and most of a robot's alerts
 * never turn on, so registering them all at startup made that copy the largest allocation in the
 * logger. An alert is registered in the same call that turns it on, so it reaches the Driver
 * Station and the log on the same loop as before.
 *
 * <p>Same constructors and methods as WPILib's; the levels are WPILib's {@code Alert.Level}.
 */
public class Alert {
    private static final List<Alert> ALL = new CopyOnWriteArrayList<>();

    // WPILib needs an id unique within the group; many alerts share text (often empty until set).
    private static final java.util.concurrent.atomic.AtomicInteger IDS =
            new java.util.concurrent.atomic.AtomicInteger();

    private static String nextId() {
        return "alert" + IDS.incrementAndGet();
    }

    private final String group;
    private final String id = nextId();
    private final Level level;
    private org.wpilib.util.Alert registered;
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
        this.group = null;
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
        this.group = group;
        this.level = level;
        this.text = text;
        ALL.add(this);
    }

    /**
     * Turns the alert on or off.
     *
     * @param on whether the alert is active
     */
    public synchronized void set(boolean on) {
        if (on && !active) {
            activeSinceSeconds = Timer.getTimestamp();
        }
        if (on != active) {
            dirty = true;
        }
        active = on;
        if (on && registered == null) {
            registered =
                    group == null
                            ? new org.wpilib.util.Alert(id, text, level)
                            : new org.wpilib.util.Alert(group, id, text, level);
        }
        if (registered != null) {
            registered.set(on);
        }
    }

    /**
     * Whether the alert is active.
     *
     * @return true while on
     */
    public boolean get() {
        return active;
    }

    /**
     * Changes the text shown while active.
     *
     * @param newText the new text
     */
    public synchronized void setText(String newText) {
        if (active && !newText.equals(text)) {
            dirty = true;
        }
        text = newText;
        if (registered != null) {
            registered.setText(newText);
        }
    }

    /**
     * The text shown while active.
     *
     * @return the text
     */
    public String getText() {
        return text;
    }

    /**
     * The alert's severity.
     *
     * @return its level
     */
    public Level getLevel() {
        return level;
    }

    /** Removes the alert. */
    public synchronized void close() {
        dirty = true;
        ALL.remove(this);
        if (registered != null) {
            registered.close();
        }
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
                // Events: one line per thing that happened, in order: modes, auto, state changes,
                // alerts, brownouts, trims, param changes.
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
        // Active error alerts, newest first.
        Logger.recordOutput("Alerts/Errors", e);
        // Active warning alerts, newest first.
        Logger.recordOutput("Alerts/Warnings", w);
        // Active info alerts, newest first.
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
