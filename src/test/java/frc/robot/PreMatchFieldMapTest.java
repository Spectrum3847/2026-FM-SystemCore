package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.spectrumLib.checks.CheckResult;
import frc.spectrumLib.checks.CheckResult.Status;
import org.junit.jupiter.api.Test;

class PreMatchFieldMapTest {
    private static final String ROBOT = "2792cd4e868967b8";

    @Test
    void theSameMapPasses() {
        assertEquals(Status.PASS, PreMatch.jetsonFieldMap(true, ROBOT, ROBOT).status());
    }

    @Test
    void aDifferentMapFails() {
        CheckResult r = PreMatch.jetsonFieldMap(true, ROBOT, "ca28a7c1139ed099");
        assertEquals(Status.FAIL, r.status());
        assertEquals(
                "Jetson field map differs (Jetson ca28a7c1139ed099, robot 2792cd4e868967b8)",
                r.reason());
    }

    @Test
    void noReportWarns() {
        CheckResult r = PreMatch.jetsonFieldMap(true, ROBOT, "");
        assertEquals(Status.WARN, r.status());
        assertEquals("Jetson not reporting its map", r.reason());
    }

    @Test
    void noJetsonExpectedSkips() {
        assertEquals(Status.SKIP, PreMatch.jetsonFieldMap(false, ROBOT, "").status());
    }
}
