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
    private long placements = 0;
    private long seeds = 0;
    private boolean alliance = true;
    private boolean enabled = false;
    private boolean fms = false;
    private int match = 0;

    private final AutoStartCheck check =
            new AutoStartCheck(
                    () ->
                            new AutoStartCheck.State(
                                    name,
                                    noAuto,
                                    fileFound,
                                    start,
                                    pose,
                                    placements,
                                    seeds,
                                    alliance,
                                    enabled,
                                    fms,
                                    match));

    private double now = 0;

    private CheckResult after(double seconds) {
        for (double t = 0; t < seconds; t += 0.02) {
            check.observe(now);
            now += 0.02;
        }
        return check.evaluate(now);
    }

    /** The code puts the pose on the start, then a loop later a camera seeds it. */
    private void placeThenSeed() {
        placements++;
        after(0.02);
        seeds++;
    }

    @Test
    void onTheStartPasses() {
        placeThenSeed();
        pose = new Pose2d(3.55, 2.02, Rotation2d.fromDegrees(178));
        CheckResult r = after(2);
        assertEquals(Status.PASS, r.status());
        assertEquals("TBTB Left selected, robot on its start (0.05 m and -2 deg)", r.reason());
    }

    @Test
    void offTheStartFailsOnceItHolds() {
        placeThenSeed();
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
        placeThenSeed();
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
        pose = new Pose2d(); // wherever the code put it
        CheckResult r = after(2);
        assertEquals(Status.WARN, r.status());
        assertEquals(
                "start pose unverified: no fresh multi-tag vision seed (TBTB Left)", r.reason());
    }

    @Test
    void aPlacedPoseIsNotComparedWithItself() {
        seeds = 5; // seeded earlier, then the auto changed and the code placed the pose
        after(1);
        placements++;
        pose = START;
        CheckResult r = after(2);
        assertEquals(Status.WARN, r.status(), "the pose is the start only because the code put it");
        seeds++;
        assertEquals(Status.PASS, after(0.1).status());
    }

    @Test
    void noAllianceWarns() {
        alliance = false;
        assertEquals(Status.WARN, after(1).status());
    }

    @Test
    void afterAnEnableThePoseIsNotCompared() {
        placeThenSeed();
        enabled = true;
        after(0.1);
        enabled = false;
        pose = new Pose2d(8, 4, Rotation2d.ZERO);
        assertEquals(Status.PASS, after(2).status());
    }

    @Test
    void reArmsWhenTheFmsAttachesAndNeedsAFreshSeed() {
        placeThenSeed();
        enabled = true; // practice on the field, then back to the queue
        after(1);
        enabled = false;
        pose = new Pose2d(8, 4, Rotation2d.ZERO);
        assertEquals(Status.PASS, after(1).status());

        fms = true;
        assertEquals(Status.WARN, after(1).status(), "the old seed is from before this match");
        seeds++;
        assertEquals(Status.FAIL, after(2).status());
        pose = START;
        seeds++;
        assertEquals(Status.PASS, after(1).status());
    }

    @Test
    void reArmsOnANewMatchNumber() {
        fms = true;
        match = 12;
        placeThenSeed();
        enabled = true;
        after(1);
        enabled = false;
        pose = new Pose2d(8, 4, Rotation2d.ZERO);
        assertEquals(Status.PASS, after(1).status(), "match 12 is over");

        match = 13; // FMS stayed attached between matches
        seeds++;
        after(0.02);
        seeds++;
        assertEquals(Status.FAIL, after(2).status());
    }
}
