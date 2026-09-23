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
    public boolean connected = false;
    public double positionRotations = 0;

    /** Position projected from the frame's own timestamp to the refresh, along the velocity. */
    public double latencyCompensatedPositionRotations = 0;

    public double velocityRps = 0;
    public double voltage = 0;
    public double statorCurrentAmps = 0;
    public double supplyCurrentAmps = 0;
    public double tempCelsius = 0;
    public double[] followerSupplyCurrentAmps = new double[0];
    public boolean[] followerConnected = new boolean[0];

    @Override
    public void toLog(LogTable table) {
        table.put("Connected", connected);
        table.put("PositionRotations", positionRotations);
        table.put("LatencyCompensatedPositionRotations", latencyCompensatedPositionRotations);
        table.put("VelocityRPS", velocityRps);
        table.put("Voltage", voltage);
        table.put("StatorCurrentAmps", statorCurrentAmps);
        table.put("SupplyCurrentAmps", supplyCurrentAmps);
        table.put("TempCelsius", tempCelsius);
        table.put("FollowerSupplyCurrentAmps", followerSupplyCurrentAmps);
        table.put("FollowerConnected", followerConnected);
    }

    @Override
    public void fromLog(LogTable table) {
        connected = table.get("Connected", connected);
        positionRotations = table.get("PositionRotations", positionRotations);
        latencyCompensatedPositionRotations =
                table.get(
                        "LatencyCompensatedPositionRotations", latencyCompensatedPositionRotations);
        velocityRps = table.get("VelocityRPS", velocityRps);
        voltage = table.get("Voltage", voltage);
        statorCurrentAmps = table.get("StatorCurrentAmps", statorCurrentAmps);
        supplyCurrentAmps = table.get("SupplyCurrentAmps", supplyCurrentAmps);
        tempCelsius = table.get("TempCelsius", tempCelsius);
        followerSupplyCurrentAmps =
                table.get("FollowerSupplyCurrentAmps", followerSupplyCurrentAmps);
        followerConnected = table.get("FollowerConnected", followerConnected);
    }
}
