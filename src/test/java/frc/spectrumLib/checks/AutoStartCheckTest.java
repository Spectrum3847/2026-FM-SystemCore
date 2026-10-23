package frc.spectrumLib.checks;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.spectrumLib.checks.CheckResult.Status;
import java.util.Optional;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

class AutoStartCheckTest {

    private static final Pose2d START = new Pose2d(3.5, 2.0, Rotation2d.fromDegrees(180));

    private String name = "TBTB Left";
    private boolean noAuto = false;
    private boolean fileFound = true;
    private Optional<Pose2d> start = Optional.of(START);
    private Pose2d pose = START;
    private boolean seeded = true;
    private boolean alliance = true;
    private boolean enabledOnce = false;

    private final AutoStartCheck check =
            new AutoStartCheck(
                    () ->
                            new AutoStartCheck.State(
                                    name,
                                    noAuto,
                                    fileFound,
                                    start,
                                    pose,
                                    seeded,
                                    alliance,
                                    enabledOnce));

    private double now = 0;

    private CheckResult after(double seconds) {
        for (double t = 0; t < seconds; t += 0.02) {
            check.observe(now);
            now += 0.02;
        }
        return check.evaluate(now);
    }

    @Test
    void onTheStartPasses() {
        pose = new Pose2d(3.55, 2.02, Rotation2d.fromDegrees(178));
        CheckResult r = after(2);
        assertEquals(Status.PASS, r.status());
        assertEquals("TBTB Left selected, robot on its start (0.05 m and -2 deg)", r.reason());
    }

    @Test
    void offTheStartFailsOnceItHolds() {
        pose = new Pose2d(3.5, 5.0, Rotation2d.fromDegrees(180)); // the other side
        assertEquals(Status.WARN, after(0.5).status(), "may still be being placed");
        CheckResult r = after(1);
        assertEquals(Status.FAIL, r.status());
        assertEquals(
                "robot is 3.00 m and 0 deg from the start of TBTB Left: wrong spot, side or"
                        + " alliance, or a bad vision seed",
                r.reason());
    }

    @Test
    void facingTheWrongWayFails() {
        pose = new Pose2d(3.5, 2.0, Rotation2d.fromDegrees(8)); // Q30: seeded backwards
        assertEquals(Status.FAIL, after(2).status());
    }

    @Test
    void noAutoWarns() {
        noAuto = true;
        name = "Do Nothing";
        assertEquals(Status.WARN, after(1).status());
    }

    @Test
    void aMissingAutoFileFails() {
        fileFound = false;
        assertEquals(Status.FAIL, after(1).status());
    }

    @Test
    void anAutoWithNoPathsFails() {
        start = Optional.empty();
        assertEquals(Status.FAIL, after(1).status());
    }

    @Test
    void anUnseededPoseIsUnverified() {
        seeded = false;
        pose = new Pose2d(); // wherever the code put it
        CheckResult r = after(2);
        assertEquals(Status.WARN, r.status());
        assertEquals(
                "start pose unverified: no camera has seeded the pose yet (TBTB Left)", r.reason());
    }

    @Test
    void noAllianceWarns() {
        alliance = false;
        assertEquals(Status.WARN, after(1).status());
    }

    @Test
    void afterTheFirstEnableThePoseIsNotCompared() {
        enabledOnce = true;
        pose = new Pose2d(8, 4, Rotation2d.kZero);
        assertEquals(Status.PASS, after(2).status());
    }
}
