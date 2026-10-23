package frc.spectrumLib.checks;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.spectrumLib.checks.CheckResult.Status;
import org.junit.jupiter.api.Test;

class DeviceHealthCheckTest {

    /** A device's logged inputs, set by the test. */
    private static final class Fake {
        boolean connected = true;
        boolean reset = false;
        String problem = null;
    }

    private final DeviceHealthCheck check = new DeviceHealthCheck("CAN devices");
    private double now = 0;

    private void run(double seconds) {
        for (double t = 0; t < seconds; t += 0.02) {
            check.observe(now);
            now += 0.02;
        }
    }

    private Fake add(String bus, String name) {
        Fake f = new Fake();
        check.add(bus, name, () -> f.connected).resets(() -> f.reset).problem(() -> f.problem);
        return f;
    }

    @Test
    void everyDeviceAnsweringPasses() {
        add("CANivore", "Hood (15)");
        add("CANivore", "Pigeon");
        add("SystemCoreCAN0", "FuelIntake (5)");
        run(2);
        CheckResult r = check.evaluate(now);
        assertEquals(Status.PASS, r.status());
        assertEquals("3 CAN devices answering", r.reason());
    }

    @Test
    void aDeviceSilentForHalfASecondFailsAndNamesItsBus() {
        add("CANivore", "Pigeon");
        Fake hood = add("CANivore", "Hood (15)");
        run(2);
        hood.connected = false;
        run(0.3);
        assertEquals(Status.PASS, check.evaluate(now).status(), "a short dropout is not missing");
        run(0.3);
        CheckResult r = check.evaluate(now);
        assertEquals(Status.FAIL, r.status());
        assertEquals("CANivore: Hood (15) not answering", r.reason());
    }

    @Test
    void aDeviceThatNeverAnsweredFailsAtOnce() {
        Fake f = add("CANivore", "FrontLeft module");
        f.connected = false;
        check.observe(now);
        assertEquals(Status.FAIL, check.evaluate(now).status());
    }

    @Test
    void theFirstContactResetIsIgnoredAndALaterOneWarns() {
        Fake f = add("CANivore", "Hood (15)");
        f.reset = true;
        run(0.5);
        f.reset = false;
        run(1);
        assertEquals(Status.PASS, check.evaluate(now).status(), "Phoenix's first-contact report");
        f.reset = true;
        run(0.02);
        f.reset = false;
        run(30);
        CheckResult r = check.evaluate(now);
        assertEquals(Status.WARN, r.status());
        assertEquals("Hood (15) rebooted in the last minute - check power wiring", r.reason());
        run(100);
        assertTrue(check.evaluate(now).reason().startsWith("Hood (15) rebooted 2 min ago"));
    }

    @Test
    void aResetWhileDisconnectedIsNotTrusted() {
        Fake f = add("CANivore", "Hood (15)");
        run(2);
        f.connected = false;
        f.reset = true;
        run(0.1);
        f.connected = true;
        f.reset = false;
        run(0.1);
        assertEquals(Status.PASS, check.evaluate(now).status());
    }

    @Test
    void anOldRebootIsForgotten() {
        Fake f = add("CANivore", "Hood (15)");
        run(2);
        f.reset = true;
        run(0.02);
        f.reset = false;
        run(DeviceHealthCheck.RECENT_REBOOT_SECONDS + 1);
        assertEquals(Status.PASS, check.evaluate(now).status());
    }

    @Test
    void severityOfAMissingDeviceCanBeLoweredOrSkipped() {
        DeviceHealthCheck co = new DeviceHealthCheck("coprocessors");
        co.add("Jetson", "orin-left", () -> false).missingIs(() -> Status.WARN);
        co.add("Quest", "Quest", () -> false).missingIs(() -> Status.SKIP);
        co.add("Limelight", "LL-Back", () -> true);
        co.observe(0);
        CheckResult r = co.evaluate(0);
        assertEquals(Status.WARN, r.status());
        assertEquals("Jetson: orin-left not answering", r.reason());
    }

    @Test
    void aProblemWhileConnectedWarns() {
        Fake f = add("Jetson", "orin-left");
        run(1);
        f.problem = "timestamps far from now (clock sync?)";
        CheckResult r = check.evaluate(now);
        assertEquals(Status.WARN, r.status());
        assertEquals("orin-left timestamps far from now (clock sync?)", r.reason());
    }

    @Test
    void noDevicesIsSkipped() {
        assertEquals(Status.SKIP, check.evaluate(0).status());
    }
}
