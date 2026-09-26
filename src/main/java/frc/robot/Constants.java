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

    /**
     * Real-time priority for the robot's main thread on the real robot, 1-99; 0 leaves it as a
     * normal thread. SystemCore shares its four cores with its own camera servers and services
     * (~85% busy on the bench unit), and a normal-priority main thread waits its turn behind them,
     * which shows up as late loops.
     *
     * <p>1 is the lowest real-time priority: above every normal process, but below Phoenix's CAN
     * threads (real-time 2-3 on SystemCore, 2026-09-23) and the HAL notifier (40) that wakes the
     * loop. Do not raise it above Phoenix: the 2026 offseason bot ran its main thread at 99 on the
     * roboRIO and starved Phoenix's frame dispatch ("CAN message is stale", WaitForAll -1003,
     * reverted in 2026-Spectrum 704030d).
     *
     * <p>Threads the main thread starts after this is set inherit it, so it is applied at the end
     * of robot init; {@code System/TopThreads} marks real-time threads with {@code [rt N]}.
     */
    public static final int MAIN_THREAD_RT_PRIORITY = 1;

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
