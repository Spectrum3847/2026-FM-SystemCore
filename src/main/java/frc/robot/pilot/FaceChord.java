package frc.robot.pilot;

/**
 * What LB and the face buttons ask for, as one function of the buttons held, so press and release
 * order cannot matter. Only one value holds at a time, so separate edge bindings cannot race.
 *
 * <p>With LB, A beats X beats B. Without LB, A beats X. Set shots are teleop only.
 */
public enum FaceChord {
    NONE,
    TRACK_TARGET,
    UNJAM,
    SET_SHOT_TOWER,
    SET_SHOT_LEFT_TRENCH,
    SET_SHOT_RIGHT_TRENCH;

    /**
     * @param lb left bumper
     * @param a A
     * @param x X
     * @param b B
     * @param teleop whether set shots are allowed
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
