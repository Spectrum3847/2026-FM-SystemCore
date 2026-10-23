package frc.spectrumLib.checks;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.spectrumLib.checks.CheckResult.Status;
import frc.spectrumLib.checks.HomingCheck.State;
import java.util.OptionalDouble;
import org.junit.jupiter.api.Test;

class BatteryHomingSettingsCheckTest {

    // -- Battery ------------------------------------------------------------------------------

    private double volts = 12.7;
    private BatteryCheck.Identity identity = BatteryCheck.Identity.NONE;
    private boolean disabled = true;
    private final BatteryCheck battery =
            new BatteryCheck(() -> volts, () -> disabled, () -> identity);

    @Test
    void sagWhileEnabledIsLoadNotCharge() {
        // TXMAN1 Q24 replay: 8.8 V at the end of the match, 12.0 V a second after.
        disabled = false;
        volts = 8.8;
        batteryAfter(5);
        disabled = true;
        volts = 12.6;
        assertEquals(Status.PASS, batteryAfter(0.1).status());
    }

    private double now = 0;

    private CheckResult batteryAfter(double seconds) {
        for (double t = 0; t < seconds; t += 0.02) {
            battery.observe(now);
            now += 0.02;
        }
        return battery.evaluate(now);
    }

    @Test
    void aChargedBatteryPasses() {
        CheckResult r = batteryAfter(1);
        assertEquals(Status.PASS, r.status());
        assertEquals("12.70 V at rest (no battery ID reader)", r.reason());
    }

    @Test
    void aUsedBatteryWarnsOnlyOnceItStaysLow() {
        volts = 12.1;
        assertEquals(Status.PASS, batteryAfter(2).status(), "a dip is not a used battery");
        assertEquals(Status.WARN, batteryAfter(1.5).status());
    }

    @Test
    void aFlatBatteryFails() {
        volts = 11.5;
        assertEquals(Status.FAIL, batteryAfter(4).status());
    }

    @Test
    void noReadingWarns() {
        volts = 0;
        assertEquals(Status.WARN, batteryAfter(4).status());
    }

    @Test
    void theSameBatteryAsLastMatchFails() {
        identity = new BatteryCheck.Identity("D7A89B9E", "D7A89B9E");
        CheckResult r = batteryAfter(1);
        assertEquals(Status.FAIL, r.status());
        assertEquals(
                "battery D7A89B9E was in the robot last match: swap it (12.70 V at rest)",
                r.reason());
        identity = new BatteryCheck.Identity("1234ABCD", "D7A89B9E");
        assertEquals(Status.PASS, batteryAfter(0.1).status());
    }

    private CheckResult after(BatteryCheck check, double seconds) {
        for (double t = 0; t < seconds; t += 0.02) {
            check.observe(now);
            now += 0.02;
        }
        return check.evaluate(now);
    }

    @Test
    void theCalibrationOffsetCorrectsTheReading() {
        // This controller reads 0.4 V low: 11.6 V on it is a used pack, not a flat one.
        BatteryCheck offset =
                new BatteryCheck(
                        () -> volts, OptionalDouble.of(0.4), () -> disabled, () -> identity);
        volts = 11.6;
        CheckResult r = after(offset, 4);
        assertEquals(Status.WARN, r.status());
        assertEquals("12.00 V at rest: not fully charged", r.reason());
    }

    @Test
    void anUncalibratedRobotWarnsAtMost() {
        BatteryCheck uncalibrated =
                new BatteryCheck(
                        () -> volts, OptionalDouble.empty(), () -> disabled, () -> identity);
        volts = 11.5;
        CheckResult r = after(uncalibrated, 4);
        assertEquals(Status.WARN, r.status());
        assertEquals(
                "11.50 V at rest: swap the battery? (voltage reading not calibrated)", r.reason());
        volts = 12.7;
        assertEquals(
                "12.70 V at rest (no battery ID reader, voltage reading not calibrated)",
                after(uncalibrated, 1).reason());
        // The same battery as last match is not a voltage question: still a FAIL.
        identity = new BatteryCheck.Identity("D7A89B9E", "D7A89B9E");
        assertEquals(Status.FAIL, after(uncalibrated, 1).status());
    }

    // -- Homing -------------------------------------------------------------------------------

    @Test
    void homingVerdicts() {
        State[] hood = {State.HOMES_WHEN_ENABLED};
        State[] intake = {State.HOMED};
        HomingCheck homing = new HomingCheck();
        homing.add("Hood", () -> hood[0]);
        homing.add("IntakeExtension", () -> intake[0]);

        CheckResult r = homing.evaluate(0);
        assertEquals(Status.PASS, r.status());
        assertEquals("1 homed, Hood homes when enabled", r.reason());

        intake[0] = State.LOST;
        assertEquals(Status.WARN, homing.evaluate(0).status());
        intake[0] = State.DISCONNECTED;
        assertEquals(Status.WARN, homing.evaluate(0).status());
        hood[0] = State.FAILED;
        r = homing.evaluate(0);
        assertEquals(Status.FAIL, r.status());
        assertEquals("Hood failed to home - check it", r.reason());
        assertEquals(Status.SKIP, new HomingCheck().evaluate(0).status());
    }

    // -- Settings -----------------------------------------------------------------------------

    @Test
    void settingsAtDefaultsPassAndChangesAreListed() {
        boolean[] practice = {false};
        boolean[] mt2 = {true};
        boolean[] fms = {false};
        SettingsCheck settings =
                new SettingsCheck()
                        .info(() -> "field k2026RebuiltAndymark")
                        .add(
                                () -> practice[0] && !fms[0] ? "practice profile ON" : null,
                                Status.WARN)
                        .add(
                                () ->
                                        practice[0] && fms[0]
                                                ? "practice profile ON in a match"
                                                : null,
                                Status.FAIL)
                        .add(() -> mt2[0] ? null : "MegaTag2 fusion OFF", Status.WARN);

        CheckResult r = settings.evaluate(0);
        assertEquals(Status.PASS, r.status());
        assertEquals("field k2026RebuiltAndymark; rest at defaults", r.reason());

        practice[0] = true;
        mt2[0] = false;
        r = settings.evaluate(0);
        assertEquals(Status.WARN, r.status());
        assertEquals("not default: practice profile ON; MegaTag2 fusion OFF", r.reason());

        fms[0] = true;
        assertEquals(Status.FAIL, settings.evaluate(0).status());
    }
}
