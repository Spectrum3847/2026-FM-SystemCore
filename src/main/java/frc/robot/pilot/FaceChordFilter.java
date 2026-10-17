package frc.robot.pilot;

/**
 * Makes bare A and X wait {@link #SETTLE_SECONDS} before they count, so a button pressed a cycle
 * before LB, or let go a cycle after it, does not blip unjam or tracking. Chords act at once.
 *
 * <p>Uses the robot clock and logged inputs, so it replays exactly. Several triggers poll it each
 * loop, so only the first call at a new timestamp advances it.
 */
public final class FaceChordFilter {
    /** How long a bare face button must be held, without LB, before it counts. */
    public static final double SETTLE_SECONDS = 0.12;

    private double lastTime = Double.NaN;
    private FaceChord output = FaceChord.NONE;

    /** The raw answer being waited on, and since when; NONE when not waiting. */
    private FaceChord pending = FaceChord.NONE;

    private double pendingSince = Double.NaN;

    private static boolean bare(FaceChord c) {
        return c == FaceChord.UNJAM || c == FaceChord.TRACK_TARGET;
    }

    /**
     * @param nowSeconds robot time
     * @param raw {@link FaceChord#of} for the buttons held now
     * @return what to act on
     */
    public FaceChord update(double nowSeconds, FaceChord raw) {
        if (nowSeconds == lastTime) {
            return output;
        }
        lastTime = nowSeconds;
        if (!bare(raw)) {
            pending = FaceChord.NONE;
            output = raw;
            return output;
        }
        // UNJAM to TRACK_TARGET only happens by letting go of A, so it does not wait.
        if (raw == output || (output == FaceChord.UNJAM && raw == FaceChord.TRACK_TARGET)) {
            pending = FaceChord.NONE;
            output = raw;
            return output;
        }
        if (raw != pending) {
            pending = raw;
            pendingSince = nowSeconds;
        }
        if (nowSeconds - pendingSince >= SETTLE_SECONDS) {
            pending = FaceChord.NONE;
            output = raw;
        } else if (!bare(output)) {
            output = FaceChord.NONE;
        }
        return output;
    }
}
