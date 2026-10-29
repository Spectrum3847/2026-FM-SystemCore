package frc.robot;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * Facts about where the code is running that logic depends on, as an AdvantageKit input.
 *
 * <p>Replay always runs in {@link Constants.Mode#REPLAY}, so code that branches on "is this the
 * simulator?" would take the simulator's branch when replaying a real match. Anything that must
 * behave the way the recorded run did reads this instead: on the robot and in the sim it records
 * the truth; in replay it restores what the recorded run was.
 */
public final class RuntimeInputs implements LoggableInputs {
    private static final RuntimeInputs INSTANCE = new RuntimeInputs();

    /** Whether the recorded run was the desktop simulator (false on a robot). */
    private boolean simulation = Constants.currentMode == Constants.Mode.SIM;

    /** Battery voltage, as the controller measured it. */
    private double batteryVoltage = 12.0;

    /** The controller is browned out (outputs disabled). False where the API is unavailable. */
    private boolean brownedOut = false;

    /** Whether the HAL's brownout API works on this controller; see {@code Robot}. */
    private static boolean brownoutApiAvailable = false;

    private RuntimeInputs() {}

    /** Records (or in replay, restores) the inputs. Call once per loop, first thing. */
    public static void update() {
        if (Constants.currentMode != Constants.Mode.REPLAY) {
            INSTANCE.simulation = Constants.currentMode == Constants.Mode.SIM;
            INSTANCE.batteryVoltage = org.wpilib.system.RobotController.getBatteryVoltage();
            INSTANCE.brownedOut =
                    brownoutApiAvailable && org.wpilib.system.RobotController.isBrownedOut();
        }
        Logger.processInputs("Runtime", INSTANCE);
    }

    /** Whether the run being executed (or replayed) is the desktop simulator. */
    public static boolean isSimulation() {
        return INSTANCE.simulation;
    }

    /** Battery voltage for the run being executed (or replayed). */
    public static double batteryVoltage() {
        return INSTANCE.batteryVoltage;
    }

    /** Whether the controller is browned out, for the run being executed (or replayed). */
    public static boolean brownedOut() {
        return INSTANCE.brownedOut;
    }

    /** Starts reading the brownout state, once the robot has found the API works. */
    public static void setBrownoutApiAvailable(boolean available) {
        brownoutApiAvailable = available;
    }

    @Override
    public void toLog(LogTable table) {
        table.put("Simulation", simulation);
        table.put("BatteryVoltage", batteryVoltage, "volts");
        table.put("BrownedOut", brownedOut);
    }

    @Override
    public void fromLog(LogTable table) {
        simulation = table.get("Simulation", simulation);
        batteryVoltage = table.get("BatteryVoltage", batteryVoltage);
        // Logs from before this was recorded have none: not browned out.
        brownedOut = table.get("BrownedOut", false);
    }
}
