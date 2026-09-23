package frc.robot;

import org.wpilib.framework.RobotBase;

/**
 * AdvantageKit runtime mode. Always {@link Mode#REAL} on SystemCore. On the desktop it is {@link
 * #simMode}: {@link Mode#SIM} for the physics sim, or {@link Mode#REPLAY} to replay a log (see
 * README, "Replaying a match").
 */
public final class Constants {
    private Constants() {}

    /**
     * Desktop mode. Also settable without editing code: {@code -Dfm.replay=true} or the {@code
     * FM_REPLAY=1} environment variable selects replay.
     */
    public static final Mode simMode =
            Boolean.getBoolean("fm.replay") || "1".equals(System.getenv("FM_REPLAY"))
                    ? Mode.REPLAY
                    : Mode.SIM;

    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    public enum Mode {
        /** Running on a real robot. */
        REAL,
        /** Running a physics simulator. */
        SIM,
        /** Replaying from a log file. */
        REPLAY
    }

    /** Whether hardware (real or simulated) exists, i.e. not replaying. */
    public static boolean hasHardware() {
        return currentMode != Mode.REPLAY;
    }
}
