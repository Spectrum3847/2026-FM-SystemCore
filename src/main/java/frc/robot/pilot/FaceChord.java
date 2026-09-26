package frc.robot.pilot;

/**
 * What the pilot's LB + face-button cluster asks for right now, as one pure function of the buttons
 * held, so press and release order cannot matter.
 *
 * <p>Before, each chord and each bare button had its own onTrue/onFalse bindings, all requesting a
 * state through instant commands with no requirements, so whichever fired last in the loop won. The
 * bare X and A triggers carry "not LB" (so LB + X does not also track), which makes pressing LB
 * while X is held a <em>falling</em> edge of {@code trackTarget_X}: the left-trench chord asked for
 * SET_SHOT and, later the same loop, X's onFalse asked for IDLE. The chords had been bound first on
 * purpose so that releasing LB first went back to tracking, which is exactly what made the opposite
 * order land in IDLE. Now {@link Pilot} makes one trigger per value here, binds only their rising
 * edges, and requests IDLE only when the value becomes {@link #NONE}; at most one of them changes
 * to true per loop, so there is nothing left to race.
 *
 * <p>Held together: with LB, A beats X beats B (tower, left trench, right trench); without LB, A
 * (unjam) beats X (track), as the old bindings resolved a same-loop press of both. Set shots are
 * teleop only (LB + a face button outside teleop asks for nothing, as before); bare X and A are not
 * gated.
 */
public enum FaceChord {
    NONE,
    TRACK_TARGET,
    UNJAM,
    SET_SHOT_TOWER,
    SET_SHOT_LEFT_TRENCH,
    SET_SHOT_RIGHT_TRENCH;

    /**
     * The request for the buttons held now.
     *
     * @param lb left bumper
     * @param a A
     * @param x X
     * @param b B
     * @param teleop whether the robot is in teleop (set shots only fire then)
     * @return what those buttons ask for
     */
    public static FaceChord of(boolean lb, boolean a, boolean x, boolean b, boolean teleop) {
        if (lb) {
            if (!teleop) {
                return NONE;
            }
            if (a) {
                return SET_SHOT_TOWER;
            }
            if (x) {
                return SET_SHOT_LEFT_TRENCH;
            }
            if (b) {
                return SET_SHOT_RIGHT_TRENCH;
            }
            return NONE;
        }
        if (a) {
            return UNJAM;
        }
        if (x) {
            return TRACK_TARGET;
        }
        return NONE;
    }
}
