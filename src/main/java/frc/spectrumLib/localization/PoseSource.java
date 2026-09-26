package frc.spectrumLib.localization;

import frc.spectrumLib.localization.PoseObservation.Kind;
import java.util.ArrayList;
import java.util.List;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.wpilib.math.geometry.Pose2d;

/**
 * One robot-pose source: a device, its own gate chain, and its own trust model.
 *
 * <p>A source always runs. {@link #update()} reads the device into logged inputs and {@link
 * #process(GateContext)} gates every observation and logs the verdict, whether or not the source is
 * {@link #isEnabled() enabled}. Enabled only decides whether its accepted measurements move the
 * robot's <em>fused</em> pose. A disabled source still feeds its own shadow estimator in {@link
 * PoseFusion}, so every source can be compared, live or in replay, on equal terms.
 *
 * <p>The enable switch is a dashboard input ({@code /Localization/Sources/<name>/Enabled} on
 * NetworkTables), recorded by AdvantageKit, so a replay reproduces exactly which sources were
 * fused.
 */
public class PoseSource {

    /** Gate outcome for one observation. */
    public record Result(
            PoseObservation observation, String rejection, StdDevModel.StdDevs stdDevs) {
        public boolean accepted() {
            return rejection == null;
        }
    }

    @Getter private final String name;
    private final PoseSourceIO io;
    private final PoseSourceInputs inputs = new PoseSourceInputs();
    private final List<Gate> gates;
    private final StdDevModel stdDevModel;
    private final LoggedNetworkBoolean enabled;
    private final String logPrefix;

    // Log keys, built once: 14 sources times a dozen keys was a lot of string building per loop.
    private final String acceptedPosesKey;
    private final String rejectedPosesKey;
    private final String verdictsKey;
    private final String xyStdKey;
    private final String acceptedCountKey;
    private final String rejectedCountKey;
    private final String lastVerdictKey;
    private final String enabledSwitchKey;
    private final String policyKey;
    private final String rejectionCountPrefix;
    private final java.util.Map<String, String> rejectionCountKeys = new java.util.HashMap<>();

    /** Observations last loop, so a quiet source does not rewrite its empty arrays every loop. */
    private int lastLoggedCount = -1;

    /** Results of the most recent {@link #process} call. */
    @Getter private final List<Result> results = new ArrayList<>();

    @Getter private long acceptedCount = 0;
    @Getter private long rejectedCount = 0;
    @Getter private String lastRejection = "";
    @Getter private double lastAcceptedTimestamp = Double.NaN;

    /**
     * Running count of rejections per reason, logged as {@code .../RejectionCounts/<reason>}. The
     * verdict arrays are only written when they change, so these counters are the reliable way to
     * tally a match.
     */
    private final java.util.Map<String, Long> rejectionCounts = new java.util.LinkedHashMap<>();

    /**
     * Creates a source.
     *
     * @param name unique name; becomes the log path
     * @param io the device layer ({@link PoseSourceIO#NONE} in replay)
     * @param gates rejection rules, checked in order; the first to fire names the rejection
     * @param stdDevModel trust model for observations that pass
     * @param enabledByDefault whether this source moves the fused pose until the dashboard says
     *     otherwise
     */
    public PoseSource(
            String name,
            PoseSourceIO io,
            List<Gate> gates,
            StdDevModel stdDevModel,
            boolean enabledByDefault) {
        this.name = name;
        this.io = io;
        this.gates = List.copyOf(gates);
        this.stdDevModel = stdDevModel;
        this.logPrefix = "Localization/Sources/" + name;
        // Leading slash: rooted at /Localization/... on NetworkTables (without it the topic has
        // no leading slash at all, which dashboards cannot bind to).
        this.enabled = new LoggedNetworkBoolean("/" + logPrefix + "/Enabled", enabledByDefault);
        acceptedPosesKey = logPrefix + "/AcceptedPoses";
        rejectedPosesKey = logPrefix + "/RejectedPoses";
        verdictsKey = logPrefix + "/Verdicts";
        xyStdKey = logPrefix + "/XYStdDevs";
        acceptedCountKey = logPrefix + "/AcceptedCount";
        rejectedCountKey = logPrefix + "/RejectedCount";
        lastVerdictKey = logPrefix + "/LastVerdict";
        enabledSwitchKey = logPrefix + "/EnabledSwitch";
        policyKey = logPrefix + "/PolicyAllowsFusion";
        rejectionCountPrefix = logPrefix + "/RejectionCounts/";
        // What the Localization tab shows for every source.
        frc.spectrumLib.telemetry.Telemetry.addDashboardKey(acceptedCountKey);
        frc.spectrumLib.telemetry.Telemetry.addDashboardKey(rejectedCountKey);
        frc.spectrumLib.telemetry.Telemetry.addDashboardKey(lastVerdictKey);
        frc.spectrumLib.telemetry.Telemetry.addDashboardKey(policyKey);
    }

    /** Whether accepted measurements from this source move the fused pose. */
    public boolean isEnabled() {
        return enabled.get();
    }

    /** Sets the enable switch (also reflected on the dashboard). */
    public void setEnabled(boolean value) {
        enabled.set(value);
    }

    /** Whether the device is reporting. */
    public boolean isConnected() {
        return inputs.connected;
    }

    /** This loop's raw inputs. */
    public PoseSourceInputs getInputs() {
        return inputs;
    }

    /** Reads the device and records (or, in replay, restores) the raw inputs. */
    public void update() {
        io.updateInputs(inputs);
        Logger.processInputs(logPrefix, inputs);
    }

    /**
     * Runs every observation through the gate chain and the trust model, and logs the outcome.
     *
     * @param context robot state for this loop
     * @return this loop's results, one per observation
     */
    public List<Result> process(GateContext context) {
        results.clear();
        int n = inputs.count();
        for (int i = 0; i < n; i++) {
            PoseObservation obs = inputs.get(name, i);
            String rejection = null;
            for (Gate gate : gates) {
                rejection = gate.check(obs, context);
                if (rejection != null) {
                    break;
                }
            }
            StdDevModel.StdDevs std = null;
            if (rejection == null) {
                std = stdDevModel.choose(obs, context);
                if (std == null) {
                    rejection = StdDevModel.NO_TIER;
                }
            }
            results.add(new Result(obs, rejection, std));
            if (rejection == null) {
                acceptedCount++;
                lastAcceptedTimestamp = obs.timestampSeconds();
            } else {
                rejectedCount++;
                lastRejection = rejection;
                long count = rejectionCounts.merge(rejection, 1L, Long::sum);
                Logger.recordOutput(
                        rejectionCountKeys.computeIfAbsent(
                                rejection, r -> rejectionCountPrefix + r),
                        count);
            }
        }
        log();
        return results;
    }

    /**
     * Logs whether the fusion policy allowed this source to move the fused pose this loop.
     *
     * @param allowed the policy's answer
     */
    public void logPolicy(boolean allowed) {
        Logger.recordOutput(policyKey, allowed);
    }

    private void log() {
        int n = results.size();
        Logger.recordOutput(enabledSwitchKey, isEnabled());
        if (n == 0 && lastLoggedCount == 0) {
            return; // nothing new: the empty arrays and the counts are already in the log
        }
        lastLoggedCount = n;
        List<Pose2d> accepted = new ArrayList<>();
        List<Pose2d> rejected = new ArrayList<>();
        String[] verdicts = new String[n];
        double[] xyStd = new double[n];
        for (int i = 0; i < n; i++) {
            Result r = results.get(i);
            if (r.accepted()) {
                accepted.add(r.observation().pose2d());
                verdicts[i] = r.stdDevs().tier();
                xyStd[i] = r.stdDevs().xyMeters();
            } else {
                rejected.add(r.observation().pose2d());
                verdicts[i] = r.rejection();
                xyStd[i] = Double.NaN;
            }
        }
        Logger.recordOutput(acceptedPosesKey, accepted.toArray(new Pose2d[0]));
        Logger.recordOutput(rejectedPosesKey, rejected.toArray(new Pose2d[0]));
        // One entry per observation, in input order: the tier if accepted, the reason if not.
        Logger.recordOutput(verdictsKey, verdicts);
        Logger.recordOutput(xyStdKey, xyStd);
        Logger.recordOutput(acceptedCountKey, acceptedCount);
        Logger.recordOutput(rejectedCountKey, rejectedCount);
        if (n > 0) {
            Logger.recordOutput(lastVerdictKey, verdicts[n - 1]);
        }
    }

    /** Whether any observation of the given kind was accepted this loop. */
    public boolean acceptedThisLoop(Kind kind) {
        for (Result r : results) {
            if (r.accepted() && r.observation().kind() == kind) {
                return true;
            }
        }
        return false;
    }
}
