package frc.robot.pilot;

/**
 * Makes a bare face button (A = unjam, X = track target) wait {@link #SETTLE_SECONDS} before it
 * counts, so sloppy chord timing doesn't fire it for a loop or two. The LB chords (set shots) still
 * act at once.
 *
 * <p>The cases this is for:
 *
 * <ul>
 *   <li>Letting go of LB + A with A a cycle late: without this, "A alone" asked for UNJAM for 10-20
 *       ms, jerking the indexer backwards as the shot ended. Now it goes straight to nothing
 *       (IDLE), and only an A still held after the settle time unjams.
 *   <li>Pressing A a cycle before LB: no unjam blip before the tower shot.
 *   <li>The same for X (track target) with the left-trench chord.
 * </ul>
 *
 * <p>While a bare button settles, the output stays on the previous bare answer if there was one (X
 * held, then A pressed: tracking continues until unjam takes over), and is {@link FaceChord#NONE}
 * otherwise. So letting go of LB with X still held is IDLE for the settle time, then tracking.
 *
 * <p>Time is the robot clock and the buttons are logged inputs, so this replays exactly. {@link
 * #update} may be called many times a loop (one trigger per answer polls it); only the first call
 * at a new timestamp advances it.
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
     * The filtered answer for the buttons held now.
     *
     * @param nowSeconds robot time (the same value all loop)
     * @param raw {@link FaceChord#of} for the buttons held now
     * @return what to act on
     */
    public FaceChord update(double nowSeconds, FaceChord raw) {
        if (nowSeconds == lastTime) {
            return output;
        }
        lastTime = nowSeconds;
        if (!bare(raw)) {
            // Chords and "nothing" act at once.
            pending = FaceChord.NONE;
            output = raw;
            return output;
        }
        if (raw == output) {
            pending = FaceChord.NONE; // already acting on it
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
