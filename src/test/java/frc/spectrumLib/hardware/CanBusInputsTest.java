package frc.spectrumLib.hardware;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.LinkedHashMap;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.LogTable;

class CanBusInputsTest {
    @Test
    void replayReadsBackEveryBus() {
        var written = new CanBusInputs();
        var buses = new LinkedHashMap<String, CanBusInputs.Bus>();
        buses.put("CANivore", new CanBusInputs.Bus(42.5, 1, 2, 3, 4, "OK", true));
        buses.put("SystemCoreCAN0", new CanBusInputs.Bus(0, 0, 0, 0, 0, "CAN bus down", false));
        written.buses = buses;
        LogTable table = new LogTable(0);
        written.toLog(table);

        var read = new CanBusInputs();
        read.fromLog(table);
        assertEquals(written.buses, read.buses);

        // A log from before these were inputs has no buses, not an error.
        var old = new CanBusInputs();
        old.fromLog(new LogTable(0));
        assertTrue(old.buses.isEmpty());
    }
}
