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

    /**
     * Robot loop period. The 2026 roboRIO ran 0.020 (50 Hz) and was often over budget; SystemCore
     * runs the whole cycle in a few milliseconds, so FM runs at 100 Hz. Everything that used to
     * assume 20 ms reads {@code RobotLoop.periodSeconds()} instead.
     */
    public static final double LOOP_PERIOD_SECONDS = 0.01;

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
