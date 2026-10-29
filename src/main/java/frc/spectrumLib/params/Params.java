package frc.spectrumLib.params;

import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import frc.spectrumLib.telemetry.Alert;
import frc.spectrumLib.telemetry.Telemetry;
import frc.spectrumLib.telemetry.Telemetry.PrintPriority;
import java.io.IOException;
import java.io.UncheckedIOException;
import java.nio.file.Path;
import java.time.Instant;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.CompletionException;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.ThreadPoolExecutor;
import java.util.concurrent.TimeUnit;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.wpilib.driverstation.Alert.Level;
import org.wpilib.driverstation.RobotState;

/**
 * Robot params: values tuned on the robot that survive the next deploy, without editing code.
 *
 * <h2>Layers</h2>
 *
 * Each param is declared in code ({@link ParamGroup}) and resolved from these layers, later ones
 * winning:
 *
 * <ol>
 *   <li>the default in the declaration;
 *   <li>{@code <deploy>/params/<group>.json}, in git;
 *   <li>{@code <deploy>/params/robots/<robot>/<group>.json}, for one robot, in git;
 *   <li>{@code <robot>/<group>.json} in the robot-local folder (e.g. {@code
 *       /home/systemcore/params/FM2026/}), which a deploy does not wipe: what was saved in the pit.
 *       A hard-stop group's file is used only on the robot it was saved on ({@link HardwareStamp});
 *   <li>live changes from the dashboard, only with a transport (tuning mode) and only while
 *       disabled with no field attached ({@link #liveChangesAllowed}).
 * </ol>
 *
 * A file holds a JSON object of {@code key: value}; a value may also be an object with the value
 * under {@code "value"} (the local file records {@code replaced} and {@code savedAt} that way).
 *
 * <h2>Never stops the robot booting</h2>
 *
 * A missing file is skipped. An unreadable file (bad JSON, or a key given twice), or a value of the
 * wrong type or out of bounds, is skipped too, and raises an alert; the value comes from the layer
 * below. A key no code declares raises an info alert, since it is usually a typo. A bad default in
 * code is a bug and throws.
 *
 * <h2>Moving a controller between robots</h2>
 *
 * The local folder is per robot config, and a hard-stop group's file (swerve offsets, soft limits)
 * carries a {@link HardwareStamp}: a file saved on another robot, or one whose robot cannot be
 * checked, is refused with an error alert and the group's values come from git. A file saved before
 * the folder was per robot ({@code <local>/<group>.json}) is read from there until the next save
 * moves it; a hard-stop group's such file has no stamp, so it is refused the same way.
 *
 * <h2>Replay</h2>
 *
 * Every value and the layer it came from is a logged input ({@link ParamsInputs}), processed when a
 * param is declared and whenever a value changes, so replay runs on the robot's values. Live
 * changes are applied in {@link #periodic()} on the main thread, never in replay.
 *
 * <h2>Saving</h2>
 *
 * The transport's {@code SaveParams} button, or {@link #requestSave()}, writes every value that
 * differs from the git layers to the robot-local folder. Disabled only; the file write runs on a
 * background thread and reaches the disk before it replaces the old file.
 *
 * <p>Main thread only.
 */
public final class Params {
    private Params() {}

    /** NT folder the {@link NetworkTablesTransport} publishes params under. */
    public static final String NT_PREFIX = "/Tuning";

    private static final ParamStore STORE = new ParamStore();
    private static final ParamsInputs INPUTS = new ParamsInputs(STORE);

    private static boolean started = false;

    /** Each param's value at the last log, to log a change as an event. */
    private static final Map<String, Object> lastLogged = new HashMap<>();

    private static ParamTransport.LiveValue saveButton;
    private static boolean saveRequested = false;
    private static ThreadPoolExecutor writer;
    private static CompletableFuture<Void> pendingSave;
    private static ParamStore.SaveJob pendingJob;

    private static int loggedValuesVersion = -1;
    private static int shownProblemsVersion = -1;
    private static int manifestSize = -1;
    private static Set<String> loggedPending = null;
    private static final Map<String, Alert> ALERTS = new HashMap<>();

    /**
     * Starts declaring params in a group. Hold the params in {@code static final} fields so each is
     * declared once.
     */
    public static ParamGroup group(String name) {
        return new ParamGroup(name, Params::register);
    }

    private static void register(Param<?> p) {
        STORE.register(p);
        if (started) {
            Logger.processInputs("Params", INPUTS);
            loggedValuesVersion = STORE.valuesVersion();
        }
    }

    /** {@link #start(Path, String, Path, ParamTransport, Map)} with no hard-stop groups. */
    public static void start(
            Path deployParamsDir, String robot, Path localRoot, ParamTransport transport) {
        start(deployParamsDir, robot, localRoot, transport, Map.of());
    }

    /**
     * Reads the params files and re-resolves anything declared so far. Call once at boot, after
     * {@code Logger.start()} and before the subsystems are built.
     *
     * @param deployParamsDir the deploy folder's {@code params} directory
     * @param robot this robot's name, for {@code params/robots/<robot>/} and {@code
     *     <localRoot>/<robot>/}
     * @param localRoot the robot-local folder that survives deploys, or null for none (replay)
     * @param transport where live tuning comes from; used only in {@link TuningMode}
     * @param stamps the hard-stop groups, whose local file must have been saved on this robot
     */
    public static void start(
            Path deployParamsDir,
            String robot,
            Path localRoot,
            ParamTransport transport,
            Map<String, HardwareStamp> stamps) {
        new HashSet<>(stamps.values()).forEach(Params::logStamp);
        ParamStore.Settings settings =
                new ParamStore.Settings(deployParamsDir, robot, localRoot, Map.copyOf(stamps));
        STORE.configure(settings);
        boolean live = transport != null && TuningMode.enabled();
        if (live) {
            STORE.enableLive(transport);
            saveButton = transport.button("SaveParams");
            STORE.problem(
                    "tuning",
                    ParamStore.Level.WARNING,
                    "Tuning mode is on: dashboard changes to params apply while disabled, off"
                            + " the field");
        }
        if (writer == null) {
            // Started now, during init, so it does not inherit the main thread's real-time
            // priority, which is raised at the end of init.
            writer =
                    new ThreadPoolExecutor(
                            1,
                            1,
                            0,
                            TimeUnit.SECONDS,
                            new LinkedBlockingQueue<>(),
                            r -> {
                                Thread t = new Thread(r, "ParamsWriter");
                                t.setDaemon(true);
                                return t;
                            });
            writer.prestartAllCoreThreads();
        }
        started = true;
        Telemetry.log("Params/Robot", robot == null ? "" : robot);
        Path localDir = settings.localDir();
        Telemetry.log("Params/LocalFolder", localDir == null ? "" : localDir.toString());
        Telemetry.log("Params/LiveTuning", live);
        Logger.processInputs("Params", INPUTS);
        loggedValuesVersion = STORE.valuesVersion();
        logChanges();
        report();
    }

    /** The stamp's readings as an input, so replay sees the ones the robot read. */
    private static void logStamp(HardwareStamp stamp) {
        Logger.processInputs(
                "Params/Hardware/" + stamp.name(),
                new LoggableInputs() {
                    @Override
                    public void toLog(LogTable table) {
                        double[] readings = stamp.atBoot();
                        table.put("Read", readings != null);
                        table.put("Readings", readings == null ? new double[0] : readings);
                    }

                    @Override
                    public void fromLog(LogTable table) {
                        stamp.restore(
                                table.get("Read", false)
                                        ? table.get("Readings", new double[0])
                                        : null);
                    }
                });
    }

    /**
     * Applies live changes ({@link #liveChangesAllowed}), logs the values, handles a save. Call
     * once per loop, before anything reads a param.
     */
    public static void periodic() {
        boolean replay = Logger.hasReplaySource();
        if (!replay) {
            STORE.applyLive(
                    liveChangesAllowed(RobotState.isDisabled(), RobotState.isFMSAttached()));
        }
        if (replay || STORE.valuesVersion() != loggedValuesVersion) {
            Logger.processInputs("Params", INPUTS);
            loggedValuesVersion = STORE.valuesVersion();
            logChanges();
        }
        if (!replay) {
            handleSave();
        }
        pollSave();
        report();
    }

    private static void logChanges() {
        changes(STORE, lastLogged).forEach(line -> Telemetry.logEvent("Events", line));
    }

    /**
     * One line per param whose value changed since {@code last}, with where the new value came
     * from, and updates {@code last}. Nothing for a param's first value.
     */
    static List<String> changes(ParamStore store, Map<String, Object> last) {
        List<String> lines = new ArrayList<>();
        for (Param<?> p : store.params()) {
            Object now = p.get();
            Object before = last.put(p.path(), now);
            if (before != null && !before.equals(now)) {
                lines.add(
                        "param "
                                + p.path()
                                + ": "
                                + before
                                + " -> "
                                + now
                                + " ("
                                + p.source()
                                + ")");
            }
        }
        return lines;
    }

    /**
     * Whether a dashboard change may apply now: disabled, and no field attached. On the field the
     * robot is disabled between auto and teleop, and a change applied there would run in teleop
     * with nobody having checked it; it waits until the robot is back in the pit.
     *
     * @param disabled the robot is disabled
     * @param fmsAttached a field management system is attached
     */
    static boolean liveChangesAllowed(boolean disabled, boolean fmsAttached) {
        return disabled && !fmsAttached;
    }

    /** Asks for a save at the next {@link #periodic()}; refused unless disabled. */
    public static void requestSave() {
        saveRequested = true;
    }

    private static void handleSave() {
        boolean pressed =
                saveRequested || (saveButton != null && Boolean.TRUE.equals(saveButton.get()));
        if (!pressed) {
            return;
        }
        saveRequested = false;
        if (saveButton != null) {
            saveButton.set(false);
        }
        if (!RobotState.isDisabled()) {
            STORE.problem(
                    "save",
                    ParamStore.Level.WARNING,
                    "Params: save refused while enabled; disable and save again");
            return;
        }
        if (pendingSave != null) {
            return;
        }
        ParamStore.SaveJob job = STORE.prepareSave(Instant.now().toString());
        if (job == null) {
            STORE.problem(
                    "save", ParamStore.Level.WARNING, "Params: no robot-local folder to save to");
            return;
        }
        pendingJob = job;
        pendingSave =
                CompletableFuture.runAsync(
                        () -> {
                            try {
                                ParamStore.write(job);
                            } catch (IOException e) {
                                throw new UncheckedIOException(e);
                            }
                        },
                        writer);
    }

    private static void pollSave() {
        if (pendingSave == null || !pendingSave.isDone()) {
            return;
        }
        String result;
        try {
            pendingSave.join();
            STORE.clearProblem("save");
            STORE.markSaved(pendingJob);
            result =
                    "saved "
                            + pendingJob.values().size()
                            + " changed values to "
                            + pendingJob.dir();
        } catch (CompletionException e) {
            Throwable cause = e.getCause() == null ? e : e.getCause();
            result = "save failed: " + cause.getMessage();
            STORE.problem("save", ParamStore.Level.ERROR, "Params: " + result);
        }
        Telemetry.print("Params: " + result, PrintPriority.HIGH);
        Telemetry.log("Params/LastSave", result);
        pendingSave = null;
        pendingJob = null;
    }

    /** Alerts and log outputs, when something changed. */
    private static void report() {
        if (STORE.problemsVersion() != shownProblemsVersion) {
            shownProblemsVersion = STORE.problemsVersion();
            Set<String> active = new HashSet<>();
            List<String> texts = new ArrayList<>();
            for (var e : STORE.problems().entrySet()) {
                ParamStore.Problem p = e.getValue();
                String id = e.getKey() + "|" + p.level();
                active.add(id);
                texts.add(p.text());
                Alert alert = ALERTS.get(id);
                if (alert == null) {
                    alert = new Alert("", level(p.level()));
                    ALERTS.put(id, alert);
                }
                if (!alert.get() || !p.text().equals(alert.getText())) {
                    Telemetry.print(p.text(), PrintPriority.HIGH);
                }
                alert.setText(p.text());
                alert.set(true);
            }
            ALERTS.forEach((id, alert) -> alert.set(active.contains(id)));
            Telemetry.log("Params/Problems", texts.toArray(new String[0]));
            List<String> files = new ArrayList<>();
            for (ParamStore.Layer l : STORE.loadedLayers()) {
                files.add(l.label() + ": " + l.status());
            }
            Telemetry.log("Params/Files", files.toArray(new String[0]));
        }
        if (!STORE.pendingLive().equals(loggedPending)) {
            loggedPending = new HashSet<>(STORE.pendingLive());
            Telemetry.log("Params/PendingLive", loggedPending.toArray(new String[0]));
        }
        if (STORE.params().size() != manifestSize) {
            manifestSize = STORE.params().size();
            Telemetry.logDash("Params/Manifest", manifest());
        }
    }

    private static Level level(ParamStore.Level l) {
        return switch (l) {
            case ERROR -> Level.HIGH;
            case WARNING -> Level.MEDIUM;
            case INFO -> Level.LOW;
        };
    }

    /** Every declared param as JSON: path, type, default, units and bounds, for dashboards. */
    private static String manifest() {
        ArrayNode out = ParamStore.JSON.createArrayNode();
        for (Param<?> p : STORE.params()) {
            ObjectNode o = out.addObject();
            o.put("path", p.path());
            o.put("type", p.type().name().toLowerCase(Locale.ROOT));
            o.set("default", defaultJson(p));
            o.put("units", p.units());
            if (!Double.isNaN(p.min()) && Double.isFinite(p.min())) {
                o.put("min", p.min());
            }
            if (!Double.isNaN(p.max()) && Double.isFinite(p.max())) {
                o.put("max", p.max());
            }
        }
        return out.toString();
    }

    private static <T> com.fasterxml.jackson.databind.JsonNode defaultJson(Param<T> p) {
        return p.toJson(p.getDefault());
    }

    /** Whether live tuning is running (tuning mode on and a transport given). */
    public static boolean liveTuning() {
        return STORE.liveEnabled();
    }

    /** The declared param at {@code group/key}, or null. */
    public static Param<?> find(String path) {
        return STORE.find(path);
    }

    /**
     * Whether a params file refused this param's value (wrong type or out of bounds), or its
     * group's file is unreadable or was saved on another robot, so the value came from a lower
     * layer. Read-only, for checks. Judged on the files this run read: in replay, the replaying
     * PC's.
     */
    public static boolean refusedInFile(Param<?> p) {
        return STORE.refusedInFile(p);
    }

    /**
     * Problems with a params file against the params declared so far: unreadable, a key nobody
     * declares, or a value of the wrong type or out of bounds. For tests and checks; it applies
     * nothing.
     */
    public static List<String> checkFile(Path file) {
        return STORE.checkFile(file);
    }
}
