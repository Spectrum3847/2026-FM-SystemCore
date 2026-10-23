package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.robot.subsystems.vision.FieldMapPushInputs;
import frc.spectrumLib.checks.CheckResult;
import frc.spectrumLib.checks.CheckResult.Status;
import java.util.List;
import org.junit.jupiter.api.Test;

class PreMatchLimelightMapTest {
    private static final String ROBOT = "2792cd4e868967b8";
    private static final List<String> CAMERAS = List.of("LL-Back", "LL-Left");

    private static FieldMapPushInputs push(
            boolean enabled, int status, String confirmed, String error) {
        FieldMapPushInputs p = new FieldMapPushInputs();
        p.enabled = enabled;
        p.httpStatus = status;
        p.confirmedHash = confirmed;
        p.error = error;
        return p;
    }

    @Test
    void everyCameraPushedThisMapPasses() {
        CheckResult r =
                PreMatch.limelightFieldMaps(
                        CAMERAS,
                        List.of(push(true, 200, ROBOT, ""), push(true, 200, ROBOT, "")),
                        ROBOT);
        assertEquals(Status.PASS, r.status());
        assertEquals("Limelight maps pushed", r.reason());
    }

    @Test
    void pushOffWarns() {
        CheckResult r =
                PreMatch.limelightFieldMaps(
                        CAMERAS, List.of(push(false, -1, "", ""), push(false, -1, "", "")), ROBOT);
        assertEquals(Status.WARN, r.status());
        assertEquals("Limelight map push off: LL-Back, LL-Left", r.reason());
    }

    @Test
    void anUnconfirmedPushWarnsWithWhy() {
        CheckResult r =
                PreMatch.limelightFieldMaps(
                        CAMERAS,
                        List.of(
                                push(true, 200, ROBOT, ""),
                                push(true, 0, "", "HttpConnectTimeoutException: timed out")),
                        ROBOT);
        assertEquals(Status.WARN, r.status());
        assertEquals(
                "Limelight map push unconfirmed: LL-Left (HttpConnectTimeoutException: timed out)",
                r.reason());
    }

    @Test
    void notYetAnsweredReadsAsPushing() {
        CheckResult r =
                PreMatch.limelightFieldMaps(
                        List.of("LL-Back"), List.of(push(true, -1, "", "")), ROBOT);
        assertEquals("Limelight map push unconfirmed: LL-Back (pushing)", r.reason());
    }

    @Test
    void anOldLogReadsAsPushOff() {
        CheckResult r =
                PreMatch.limelightFieldMaps(
                        List.of("LL-Back"), List.of(new FieldMapPushInputs()), ROBOT);
        assertEquals(Status.WARN, r.status());
    }
}
