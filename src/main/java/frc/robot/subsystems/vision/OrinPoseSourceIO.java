package frc.robot.subsystems.vision;

import frc.spectrumLib.localization.PoseObservation;
import frc.spectrumLib.localization.PoseSourceIO;
import frc.spectrumLib.localization.PoseSourceInputs;
import java.util.List;
import java.util.Set;
import java.util.function.DoubleFunction;
import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.system.Timer;

/**
 * The Orin camera's pose source: reads the camera into logged {@link OrinCameraInputs}, then solves
 * poses from those inputs with {@link OrinSolver}. It is {@link #derived() derived}: in replay the
 * camera IO is {@link OrinCameraIO#NONE}, the raw inputs come back from the log, and the solve runs
 * again, so mounts, the solver and its filters can all be changed and re-run against a match.
 */
public class OrinPoseSourceIO implements PoseSourceIO {
    private final String logPrefix;
    private final OrinCameraIO io;
    @Getter private final OrinCameraInputs cameraInputs = new OrinCameraInputs();
    private final OrinSolver solver;
    private final Supplier<Set<Integer>> excludedTags;
    private final DoubleFunction<Rotation2d> headingAt;

    private int resultsThisSecond = 0;
    private double secondStart = Double.NaN;

    /** Results per second over the last full second (section 7a); NaN until the first second. */
    @Getter private double resultsPerSecond = Double.NaN;

    /** When the camera last sent a result; NaN before the first. Robot clock, replayed. */
    @Getter private double lastResultSeconds = Double.NaN;

    /**
     * @param cameraName PhotonVision camera name; also the log path
     * @param io the camera ({@link OrinCameraIO#NONE} in replay)
     * @param solver this camera's solver
     * @param excludedTags tags to leave out of single-tag solves (section 12)
     * @param headingAt robot heading at a capture time, or null if not known yet
     */
    public OrinPoseSourceIO(
            String cameraName,
            OrinCameraIO io,
            OrinSolver solver,
            Supplier<Set<Integer>> excludedTags,
            DoubleFunction<Rotation2d> headingAt) {
        this.logPrefix = "Vision/Orin/" + cameraName;
        this.io = io;
        this.solver = solver;
        this.excludedTags = excludedTags;
        this.headingAt = headingAt;
    }

    /** Capture time of the newest result, PhotonLib's timebase; NaN before the first. */
    public double getLastResultTimestampSeconds() {
        return solver.lastTimestampSeconds;
    }

    @Override
    public boolean derived() {
        return true;
    }

    /** Asks the camera to run a pipeline. Ignored in replay. */
    public void setPipelineIndex(int index) {
        io.setPipelineIndex(index);
    }

    @Override
    public void updateInputs(PoseSourceInputs inputs) {
        io.updateInputs(cameraInputs);
        Logger.processInputs(logPrefix, cameraInputs);

        double now = Timer.getTimestamp();
        int n = cameraInputs.results.length;
        if (n > 0) {
            lastResultSeconds = now;
        }
        if (Double.isNaN(secondStart)) {
            secondStart = now;
        }
        resultsThisSecond += n;
        if (now - secondStart >= 1.0) {
            resultsPerSecond = resultsThisSecond / (now - secondStart);
            resultsThisSecond = 0;
            secondStart = now;
        }

        List<PoseObservation> observations =
                solver.solve(cameraInputs, excludedTags.get(), headingAt);
        inputs.connected = cameraInputs.connected;
        inputs.resize(observations.size());
        for (int i = 0; i < observations.size(); i++) {
            inputs.set(i, observations.get(i));
        }
        inputs.kind = PoseObservation.Kind.PHOTON.ordinal();
        inputs.tagIds = solver.seenTags.stream().toArray();

        Logger.recordOutput(logPrefix + "/ResultsPerSecond", resultsPerSecond);
        Logger.recordOutput(logPrefix + "/LatencyMs", solver.lastLatencyMs);
        Logger.recordOutput(logPrefix + "/ResultAgeSeconds", now - solver.lastTimestampSeconds);
        if (solver.decodeFailures > 0 || cameraInputs.readFailures > 0) {
            Logger.recordOutput(
                    logPrefix + "/DecodeFailures",
                    solver.decodeFailures + cameraInputs.readFailures);
        }
        if (solver.excludedTargets > 0) {
            Logger.recordOutput(logPrefix + "/ExcludedTargets", solver.excludedTargets);
        }
    }
}
