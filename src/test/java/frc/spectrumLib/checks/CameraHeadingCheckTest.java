package frc.spectrumLib.checks;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.spectrumLib.checks.CheckResult.Status;
import org.junit.jupiter.api.Test;

class CameraHeadingCheckTest {

    private final CameraHeadingCheck check = new CameraHeadingCheck();
    private double now = 0;

    /**
     * Two seconds of frames: each camera reads the true heading plus its own mount error, while the
     * gyro reads the true heading minus a boot offset that the cameras' offsets must cancel.
     */
    private void frames(double trueHeading, double gyroBoot, double... cameraErrors) {
        for (int i = 0; i < 100; i++) {
            double noise = (i % 5 - 2) * 0.5;
            for (int c = 0; c < cameraErrors.length; c++) {
                check.addFrame(
                        "cam" + c,
                        trueHeading + cameraErrors[c] + noise,
                        trueHeading - gyroBoot,
                        3,
                        0,
                        now);
            }
            now += 0.02;
        }
    }

    /** Judges, then judges again once a worse verdict has had time to hold. */
    private CheckResult settled() {
        check.evaluate(now);
        now += CameraHeadingCheck.HOLD_SECONDS;
        return check.evaluate(now);
    }

    @Test
    void camerasThatAgreePass() {
        frames(30, 40, 0, 1, -1.5);
        CheckResult r = check.evaluate(now);
        assertEquals(Status.PASS, r.status());
        assertTrue(r.reason().startsWith("cam0, cam1, cam2 agree within 3 deg"), r.reason());
    }

    @Test
    void theQ30YawSignErrorFails() {
        // TXMAN1 Q30: LL-Left's MegaTag1 heading read 172 deg from LL-Back's.
        frames(-145, 10, 0, 172);
        CheckResult r = settled();
        assertEquals(Status.FAIL, r.status());
        assertEquals(
                "cam0 and cam1 disagree on heading by 172 deg - a camera's mount yaw is wrong",
                r.reason());
    }

    @Test
    void aFewDegreesOfMountErrorWarns() {
        frames(0, 0, 0, 8);
        assertEquals(Status.WARN, check.evaluate(now).status());
    }

    @Test
    void aVerdictHoldsUntilTheErrorIsADegreeInside() {
        double[] errors = {6.5, 5.5, 4.5, 5.5};
        Status[] expected = {Status.WARN, Status.WARN, Status.PASS, Status.PASS};
        for (int i = 0; i < errors.length; i++) {
            now += CameraHeadingCheck.WINDOW_SECONDS + 1; // only this step's frames count
            frames(0, 0, 0, errors[i]);
            assertEquals(expected[i], check.evaluate(now).status(), "error " + errors[i]);
        }
    }

    @Test
    void aSingleCameraIsComparedWithTheSeededHeading() {
        frames(90, 25, 0);
        check.setRobotHeading(90 + 172, 90 - 25);
        CheckResult r = settled();
        assertEquals(Status.FAIL, r.status());
        assertEquals(
                "cam0 is 172 deg from the robot heading - a camera's mount yaw is wrong",
                r.reason());

        check.setRobotHeading(91, 90 - 25);
        assertEquals(Status.PASS, check.evaluate(now).status());
    }

    @Test
    void aBriefDisagreementIsNotReported() {
        frames(0, 0, 0, 0);
        assertEquals(Status.PASS, check.evaluate(now).status());
        // The robot is turned around without its gyro turning; cam0's median flips first.
        for (int i = 0; i < 150; i++) {
            check.addFrame("cam0", 180, 0, 3, 0, now);
            now += 0.01;
        }
        assertEquals(Status.PASS, check.evaluate(now).status(), "not held yet");
        for (int i = 0; i < 150; i++) {
            check.addFrame("cam1", 180, 0, 3, 0, now);
            now += 0.01;
        }
        assertEquals(Status.PASS, check.evaluate(now).status(), "cam1 caught up");
    }

    @Test
    void aGyroResetStartsOver() {
        frames(0, 0, 0, 0);
        check.setRobotHeading(Double.NaN, 0);
        check.setRobotHeading(Double.NaN, 90); // the gyro restarted
        CheckResult r = check.evaluate(now);
        assertTrue(r.reason().startsWith("no camera has seen 2+ tags"), r.reason());
    }

    @Test
    void offsetsEitherSideOfOneEightyAgree() {
        frames(179, 0, 0, 2); // cam0 reads 179, cam1 reads -179
        assertEquals(Status.PASS, check.evaluate(now).status());
    }

    @Test
    void singleTagAndSpinningFramesDoNotCount() {
        for (int i = 0; i < 100; i++) {
            check.addFrame("cam0", 0, 0, 1, 0, now);
            check.addFrame("cam1", 170, 0, 3, 1.0, now);
            now += 0.02;
        }
        CheckResult r = check.evaluate(now);
        assertEquals(Status.WARN, r.status());
        assertTrue(r.reason().startsWith("no camera has seen 2+ tags"), r.reason());
    }

    @Test
    void oldFramesAgeOut() {
        frames(0, 0, 0, 172);
        now += CameraHeadingCheck.WINDOW_SECONDS + 1;
        assertEquals(Status.WARN, check.evaluate(now).status(), "nothing recent: unchecked");
    }

    @Test
    void wrapStaysInRange() {
        assertEquals(-180, CameraHeadingCheck.wrap(180), 1e-9);
        assertEquals(-170, CameraHeadingCheck.wrap(190), 1e-9);
        assertEquals(170, CameraHeadingCheck.wrap(-190), 1e-9);
    }
}
