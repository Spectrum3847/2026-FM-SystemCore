package frc.spectrumLib.localization;

/**
 * One rejection rule in a source's gate chain.
 *
 * <p>Gates are per source <em>type</em>, not universal: AprilTag sources declare the 2026 tag-flip,
 * tilt, height and spin gates; QuestNav declares tracking-loss, jump and time-since-reset gates.
 * Every gate has a name, and a rejection is recorded under that name, which is the single most
 * valuable thing the 2026 {@code Vision.java} did: a log says <em>why</em> a frame was thrown out,
 * not just that it was.
 */
@FunctionalInterface
public interface Gate {

    /**
     * Checks one observation.
     *
     * @param observation the measurement under test
     * @param context robot state at gating time
     * @return {@code null} to pass, or the rejection reason (a short, stable string: it becomes a
     *     log value that analysis groups by)
     */
    String check(PoseObservation observation, GateContext context);

    /**
     * A named gate: returns {@code reason} whenever {@code rejectIf} is true.
     *
     * @param reason the rejection reason to log
     * @param rejectIf predicate that is true for observations to reject
     * @return the gate
     */
    static Gate rejectIf(
            String reason, java.util.function.BiPredicate<PoseObservation, GateContext> rejectIf) {
        return (obs, ctx) -> rejectIf.test(obs, ctx) ? reason : null;
    }
}
