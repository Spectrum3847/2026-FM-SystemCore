package frc.spectrumLib.framework;

import java.lang.reflect.Field;
import org.littletonrobotics.junction.LoggedRobot;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.framework.IterativeRobotBase;
import org.wpilib.system.Watchdog;

/**
 * Base robot class for Spectrum robots on SystemCore: AdvantageKit's {@link LoggedRobot}, which
 * runs the loop, records every input, and drives replay.
 */
public class SpectrumRobot extends LoggedRobot {

    /**
     * Loop length above which WPILib prints "Loop time of Xs overrun" plus a per-section epoch
     * breakdown to the Driver Station.
     *
     * <p>0.20 s keeps the console quiet in matches. It also hides everything below it: in the
     * 2026-09-05 roboRIO logs 60 to 90 percent of enabled loops ran over 25 ms and none of that
     * reached the console. AdvantageKit's {@code /RealOutputs/LoggedRobot/*} timings and the {@code
     * Scheduler/*} timers are the record of loop time.
     */
    public static final double LOOP_OVERRUN_WARNING_SECONDS = 0.20;

    /** Sets the loop overrun watchdogs to {@link #LOOP_OVERRUN_WARNING_SECONDS}. */
    public SpectrumRobot() {
        super();
        try {
            Field watchdogField = IterativeRobotBase.class.getDeclaredField("m_watchdog");
            watchdogField.setAccessible(true);
            Watchdog watchdog = (Watchdog) watchdogField.get(this);
            watchdog.setTimeout(LOOP_OVERRUN_WARNING_SECONDS);
        } catch (Exception e) {
            DriverStationErrors.reportWarning("Failed to adjust loop overrun warnings.", false);
        }
        CommandScheduler.getInstance().setPeriod(LOOP_OVERRUN_WARNING_SECONDS);
    }
}
