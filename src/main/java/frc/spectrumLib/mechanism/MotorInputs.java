package frc.spectrumLib.mechanism;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * Everything a {@link Mechanism} reads from its motors in one loop, as an AdvantageKit input.
 *
 * <p>This is what makes every Spectrum mechanism replayable without a per-subsystem IO layer: the
 * signals are refreshed once per loop, copied into this object, and passed through {@code
 * Logger.processInputs}. On the robot that records them; in replay it overwrites them with the
 * recorded values, and every getter on {@link Mechanism} reads from here, so the mechanism's logic
 * sees exactly what it saw on the field.
 *
 * <p>Logged under {@code /Mechanisms/<name>/...}.
 */
public class MotorInputs implements LoggableInputs, Cloneable {
    /** The leader answered this loop's signal refresh. Readings are stale or zero while false. */
    public boolean connected = false;

    /**
     * The motor rebooted since last loop, so its position restarted and its signal rates went back
     * to defaults. Not set for a device's first report after the program starts.
     */
    public boolean resetOccurred = false;

    /** Leader position in mechanism rotations (after the gear ratio), as of its last frame. */
    public double positionRotations = 0;

    /** Position projected from the frame's own timestamp to the refresh, along the velocity. */
    public double latencyCompensatedPositionRotations = 0;

    /** Leader velocity in mechanism rotations per second. */
    public double velocityRps = 0;

    /** Voltage the leader applies to its motor (its output, not the battery). */
    public double voltage = 0;

    /** Current through the leader's motor windings: what makes torque and heat. */
    public double statorCurrentAmps = 0;

    /** Current the leader draws from the battery. */
    public double supplyCurrentAmps = 0;

    /** Leader's internal temperature. */
    public double tempCelsius = 0;

    /**
     * Phoenix's fault bitfields ({@code getFaultField}, {@code getStickyFaultField}): active now,
     * and latched since boot (sticky faults are cleared when the motor is created). The bit order
     * is undocumented: Tuner X decodes it. {@link #troubleFaults} names the ones that matter.
     */
    public long faultField = 0;

    /** Faults latched since boot, as a bitfield (see faultField). */
    public long stickyFaultField = 0;

    /** Each follower's latched-fault bitfield, in config order. */
    public long[] followerStickyFaultField = new long[0];

    /**
     * The leader's sticky faults that mean something went wrong (hardware, temperature, voltage,
     * boot during enable), by name, e.g. {@code Undervoltage}. Never current or position limits.
     */
    public String[] troubleFaults = new String[0];

    /** Each follower's battery current, in config order. */
    public double[] followerSupplyCurrentAmps = new double[0];

    /** Whether each follower answered this loop, in config order. */
    public boolean[] followerConnected = new boolean[0];

    /** Whether each follower rebooted since last loop, in config order. */
    public boolean[] followerResetOccurred = new boolean[0];

    @Override
    public void toLog(LogTable table) {
        table.put("Connected", connected);
        table.put("ResetOccurred", resetOccurred);
        table.put("PositionRotations", positionRotations, "rotations");
        table.put(
                "LatencyCompensatedPositionRotations",
                latencyCompensatedPositionRotations,
                "rotations");
        table.put("VelocityRPS", velocityRps, "rotations per second");
        table.put("Voltage", voltage, "volts");
        table.put("StatorCurrentAmps", statorCurrentAmps, "amps");
        table.put("SupplyCurrentAmps", supplyCurrentAmps, "amps");
        table.put("TempCelsius", tempCelsius, "celsius");
        table.put("FaultField", faultField);
        table.put("StickyFaultField", stickyFaultField);
        table.put("FollowerStickyFaultField", followerStickyFaultField);
        table.put("TroubleFaults", troubleFaults);
        table.put("FollowerSupplyCurrentAmps", followerSupplyCurrentAmps);
        table.put("FollowerConnected", followerConnected);
        table.put("FollowerResetOccurred", followerResetOccurred);
    }

    @Override
    public void fromLog(LogTable table) {
        connected = table.get("Connected", connected);
        // Logs from before resets were recorded have none: false, never the previous loop's.
        resetOccurred = table.get("ResetOccurred", false);
        positionRotations = table.get("PositionRotations", positionRotations);
        latencyCompensatedPositionRotations =
                table.get(
                        "LatencyCompensatedPositionRotations", latencyCompensatedPositionRotations);
        velocityRps = table.get("VelocityRPS", velocityRps);
        voltage = table.get("Voltage", voltage);
        statorCurrentAmps = table.get("StatorCurrentAmps", statorCurrentAmps);
        supplyCurrentAmps = table.get("SupplyCurrentAmps", supplyCurrentAmps);
        tempCelsius = table.get("TempCelsius", tempCelsius);
        // Logs from before faults were recorded have none: no faults.
        faultField = table.get("FaultField", 0L);
        stickyFaultField = table.get("StickyFaultField", 0L);
        followerStickyFaultField = table.get("FollowerStickyFaultField", new long[0]);
        troubleFaults = table.get("TroubleFaults", new String[0]);
        followerSupplyCurrentAmps =
                table.get("FollowerSupplyCurrentAmps", followerSupplyCurrentAmps);
        followerConnected = table.get("FollowerConnected", followerConnected);
        followerResetOccurred =
                table.get("FollowerResetOccurred", new boolean[followerConnected.length]);
    }
}
