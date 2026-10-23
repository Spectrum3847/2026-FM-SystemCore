package frc.spectrumLib.mechanism;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.LogTable;

class MotorResetMonitorTest {
    private final MotorResetMonitor monitor = new MotorResetMonitor();

    private void connectedLoops(int n) {
        for (int i = 0; i < n; i++) {
            assertFalse(monitor.filter(true, false));
        }
    }

    @Test
    void firstContactIsNotAReset() {
        assertFalse(monitor.filter(true, true), "Phoenix's first report");
        connectedLoops(MotorResetMonitor.ARM_LOOPS - 2);
        assertFalse(monitor.filter(true, true));
        assertTrue(monitor.filter(true, true), "armed after a second of contact");
    }

    @Test
    void aDeviceThatAppearsLateArmsFromItsFirstContact() {
        for (int i = 0; i < 500; i++) {
            assertFalse(monitor.filter(false, false));
        }
        assertFalse(monitor.filter(true, true));
        connectedLoops(MotorResetMonitor.ARM_LOOPS - 1);
        assertTrue(monitor.filter(true, true));
    }

    @Test
    void staysArmedThroughTheRebootsDisconnect() {
        connectedLoops(MotorResetMonitor.ARM_LOOPS);
        for (int i = 0; i < 25; i++) {
            assertFalse(monitor.filter(false, false)); // booting
        }
        assertTrue(monitor.filter(true, true));
    }

    @Test
    void aResetLosesThePositionOnlyWhereItIsTracked() {
        assertFalse(monitor.update(false, true));
        assertFalse(monitor.isPositionLost());

        assertTrue(monitor.update(true, false), "a roller: counted, nothing lost");
        assertFalse(monitor.isPositionLost());

        assertTrue(monitor.update(true, true));
        assertTrue(monitor.isPositionLost());
        assertFalse(monitor.update(false, true));
        assertTrue(monitor.isPositionLost(), "latched until homed");

        monitor.positionRestored();
        assertFalse(monitor.isPositionLost());
        assertEquals(2, monitor.getResets());
    }

    @Test
    void replayingAnOldLogWithNoResetInputChangesNothing() {
        MotorInputs inputs = new MotorInputs();
        LogTable table = new LogTable(0);
        table.put("Connected", true);
        table.put("FollowerConnected", new boolean[] {true, true});
        inputs.resetOccurred = true;
        inputs.fromLog(table);
        assertFalse(inputs.resetOccurred);
        assertEquals(2, inputs.followerResetOccurred.length);
        assertFalse(inputs.followerResetOccurred[0] || inputs.followerResetOccurred[1]);
    }
}
