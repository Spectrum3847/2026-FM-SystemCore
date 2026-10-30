package frc.robot;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.File;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import org.littletonrobotics.junction.Logger;
import org.wpilib.framework.IterativeRobotBase;
import org.wpilib.hardware.hal.AllianceStationID;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.simulation.SimHooks;

/**
 * The whole robot booted headless in simulation, with its loop stepped by hand on a paused sim
 * clock ({@link SimHooks#stepTiming}): the same calls {@code LoggedRobot.startCompetition} makes
 * each cycle, so timing is the same on every machine. For tests tagged {@code sim}, which each get
 * their own JVM ({@code ./gradlew simTest}).
 */
final class SimRobot {
    private SimRobot() {}

    static Robot robot;
    static long bootMillis;
    private static Method beforeUser, afterUser, loopFunc;
    private static final Set<File> logsBefore = new HashSet<>();

    /** Boots the robot disabled in teleop, blue 1, and runs 50 loops. */
    static void boot() throws Exception {
        File[] logs = new File("logs").listFiles();
        if (logs != null) {
            logsBefore.addAll(List.of(logs));
        }
        assertTrue(HAL.initialize(), "HAL failed to initialize");
        SimHooks.pauseTiming();
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setAllianceStationId(AllianceStationID.BLUE_1);
        DriverStationSim.setRobotMode(RobotMode.TELEOPERATED);
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();

        long start = System.nanoTime();
        robot = new Robot();
        bootMillis = (System.nanoTime() - start) / 1_000_000;

        beforeUser = Logger.class.getDeclaredMethod("periodicBeforeUser");
        beforeUser.setAccessible(true);
        afterUser = Logger.class.getDeclaredMethod("periodicAfterUser", long.class, long.class);
        afterUser.setAccessible(true);
        loopFunc = IterativeRobotBase.class.getDeclaredMethod("loopFunc");
        loopFunc.setAccessible(true);
        robot.simulationInit();
        step(50);
    }

    /** Runs {@code loops} robot loops. */
    static void step(int loops) throws Exception {
        for (int i = 0; i < loops; i++) {
            SimHooks.stepTiming(Constants.LOOP_PERIOD_SECONDS);
            beforeUser.invoke(null);
            try {
                loopFunc.invoke(robot);
            } catch (InvocationTargetException e) {
                throw (Exception) (e.getCause() instanceof Exception ? e.getCause() : e);
            }
            afterUser.invoke(null, 0L, 0L);
        }
    }

    /** Sets the Driver Station's mode and enable, then runs 20 loops. */
    static void setMode(RobotMode mode, boolean enabled) throws Exception {
        DriverStationSim.setRobotMode(mode);
        DriverStationSim.setEnabled(enabled);
        DriverStationSim.notifyNewData();
        step(20);
    }

    /** Ends the log and deletes this run's sim log: the test's results are in the test report. */
    static void cleanUp() {
        Logger.end();
        File[] logs = new File("logs").listFiles();
        if (logs != null) {
            for (File f : logs) {
                if (!logsBefore.contains(f) && f.getName().endsWith(".wpilog")) {
                    f.delete();
                }
            }
        }
    }
}
