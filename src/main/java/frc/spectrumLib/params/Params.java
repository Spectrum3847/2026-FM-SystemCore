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
import org.littletonrobotics.junction.Logger;
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
 *   <li>{@code <group>.json} in the robot-local folder (e.g. {@code /home/systemcore/params/}),
 *       which a deploy does not wipe: what was saved in the pit;
 *   <li>live changes from the dashboard, only with a transport (tuning mode) and only while
 *       disabled.
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

    /**
     * Reads the params files and re-resolves anything declared so far. Call once at boot, after
     * {@code Logger.start()} and before the subsystems are built.
     *
     * @param deployParamsDir the deploy folder's {@code params} directory
     * @param robot this robot's name, for {@code params/robots/<robot>/}
     * @param localDir the robot-local folder that survives deploys, or null for none (replay)
     * @param transport where live tuning comes from; used only in {@link TuningMode}
     */
    public static void start(
            Path deployParamsDir, String robot, Path localDir, ParamTransport transport) {
        STORE.configure(new ParamStore.Settings(deployParamsDir, robot, localDir));
        boolean live = transport != null && TuningMode.enabled();
        if (live) {
            STORE.enableLive(transport);
            saveButton = transport.button("SaveParams");
            STORE.problem(
                    "tuning",
                    ParamStore.Level.WARNING,
                    "Tuning mode is on: dashboard changes to params apply while disabled");
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
        Telemetry.log("Params/LocalFolder", localDir == null ? "" : localDir.toString());
        Telemetry.log("Params/LiveTuning", live);
        Logger.processInputs("Params", INPUTS);
        loggedValuesVersion = STORE.valuesVersion();
        report();
    }

    /**
     * Applies live changes (disabled only), logs the values, handles a save. Call once per loop,
     * before anything reads a param.
     */
    public static void periodic() {
        boolean replay = Logger.hasReplaySource();
        if (!replay) {
            STORE.applyLive(RobotState.isDisabled());
        }
        if (replay || STORE.valuesVersion() != loggedValuesVersion) {
            Logger.processInputs("Params", INPUTS);
            loggedValuesVersion = STORE.valuesVersion();
        }
        if (!replay) {
            handleSave();
        }
        pollSave();
        report();
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
     * Problems with a params file against the params declared so far: unreadable, a key nobody
     * declares, or a value of the wrong type or out of bounds. For tests and checks; it applies
     * nothing.
     */
    public static List<String> checkFile(Path file) {
        return STORE.checkFile(file);
    }
}
