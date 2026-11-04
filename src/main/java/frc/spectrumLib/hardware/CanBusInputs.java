package frc.spectrumLib.hardware;

import com.ctre.phoenix6.CANBus.CANBusStatus;
import java.util.LinkedHashMap;
import java.util.Map;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * Every CAN bus's health as an AdvantageKit input, under {@code /CAN/<bus>/}, so replay and alerts
 * can use it. Read once a second off the main thread; between reads it repeats the last one. Every
 * counter reads 0 on a bus whose status read failed, so check {@code statusOk} first.
 */
public class CanBusInputs implements LoggableInputs {
    /** One bus's reading. */
    public record Bus(
            double utilizationPct,
            long busOffCount,
            long txFullCount,
            long receiveErrorCount,
            long transmitErrorCount,
            String status,
            boolean statusOk) {}

    /** By bus label, in the order the buses were found. */
    public Map<String, Bus> buses = new LinkedHashMap<>();

    /** Replaces the readings with a new sample. */
    public void set(Map<String, CANBusStatus> sample) {
        var next = new LinkedHashMap<String, Bus>();
        sample.forEach(
                (label, s) ->
                        next.put(
                                label,
                                new Bus(
                                        s.BusUtilization * 100,
                                        s.BusOffCount,
                                        s.TxFullCount,
                                        s.REC,
                                        s.TEC,
                                        s.Status.getName(),
                                        s.Status.isOK())));
        buses = next;
    }

    @Override
    public void toLog(LogTable table) {
        // The buses found, by the label each one is logged under.
        table.put("Labels", buses.keySet().toArray(new String[0]));
        buses.forEach(
                (label, b) -> {
                    LogTable t = table.getSubtable(label);
                    // Share of the bus's bandwidth in use.
                    t.put("BusUtilization", b.utilizationPct(), "percent");
                    // Times the bus went bus-off (too many errors) since the controller booted.
                    t.put("BusOffCount", b.busOffCount());
                    // Frames dropped because the transmit buffer was full since boot.
                    t.put("TxFullCount", b.txFullCount());
                    // CAN receive error counter: rises with noise or a bad wire, falls as frames
                    // arrive clean.
                    t.put("ReceiveErrorCounter", b.receiveErrorCount());
                    // CAN transmit error counter (0-255): over 255 the bus goes bus-off.
                    t.put("TransmitErrorCounter", b.transmitErrorCount());
                    // Phoenix's status for the read; anything but OK means the counters are 0.
                    t.put("Status", b.status());
                    // The status read worked: only then are the other values real.
                    t.put("StatusOK", b.statusOk());
                });
    }

    @Override
    public void fromLog(LogTable table) {
        var next = new LinkedHashMap<String, Bus>();
        for (String label : table.get("Labels", new String[0])) {
            LogTable t = table.getSubtable(label);
            next.put(
                    label,
                    new Bus(
                            t.get("BusUtilization", 0.0),
                            t.get("BusOffCount", 0L),
                            t.get("TxFullCount", 0L),
                            t.get("ReceiveErrorCounter", 0L),
                            t.get("TransmitErrorCounter", 0L),
                            t.get("Status", ""),
                            t.get("StatusOK", false)));
        }
        buses = next;
    }
}
