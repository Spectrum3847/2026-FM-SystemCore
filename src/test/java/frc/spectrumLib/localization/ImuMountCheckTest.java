package frc.spectrumLib.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.spectrumLib.localization.ImuMountCheck.Verdict;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class ImuMountCheckTest {
    private static final double DT = 0.02;
    private static final double G = 9.81;

    private ImuMountCheck check;
    private double time;
    private double heading;

    // How the gyros are really mounted, edited per test.
    private double pigeonTurnDeg;
    private double pigeonYawSign;
    private double imuYawSign;
    private double pigeonRollDeg;
    private double[] imuGravity;
    private boolean wheelsOk;
    private double vx;
    private double vy;

    @BeforeEach
    void setUp() {
        check = new ImuMountCheck(true);
        time = 10;
        heading = 0;
        pigeonTurnDeg = 0;
        pigeonYawSign = 1;
        imuYawSign = 1;
        pigeonRollDeg = 0;
        imuGravity = new double[] {0, 0, G};
        wheelsOk = true;
        vx = 0;
        vy = 0;
    }

    /** Runs the robot at a constant acceleration (robot frame) and turn rate, from its velocity. */
    private void run(double seconds, double ax, double ay, double omega) {
        double turn = Math.toRadians(pigeonTurnDeg);
        for (int i = 0; i < Math.round(seconds / DT); i++) {
            time += DT;
            vx += ax * DT;
            vy += ay * DT;
            heading += omega * DT;
            // The robot's acceleration seen in a Pigeon turned by pigeonTurnDeg.
            // NaN stands for a Pigeon that reads no acceleration, like the sim's.
            double px = Double.isNaN(turn) ? 0 : (ax * Math.cos(turn) + ay * Math.sin(turn)) / G;
            double py = Double.isNaN(turn) ? 0 : (-ax * Math.sin(turn) + ay * Math.cos(turn)) / G;
            check.update(
                    new ImuMountCheck.Inputs(
                            time,
                            wheelsOk,
                            vx,
                            vy,
                            omega,
                            true,
                            pigeonYawSign * heading + 1.0,
                            0,
                            pigeonRollDeg,
                            px,
                            py,
                            true,
                            imuYawSign * heading - 2.0,
                            imuGravity[0],
                            imuGravity[1],
                            imuGravity[2]));
        }
    }

    private void rest(double seconds) {
        run(seconds, 0, 0, 0);
    }

    private void accelerate() {
        for (int i = 0; i < 4; i++) {
            run(0.5, 6, 0, 0);
            run(0.5, -6, 0, 0);
            run(0.5, 0, 4, 0);
            run(0.5, 0, -4, 0);
            rest(0.2);
        }
    }

    private void spin() {
        run(2, 0, 0, 4);
    }

    @Test
    void correctMountsPass() {
        rest(3);
        accelerate();
        spin();
        assertEquals(Verdict.OK, check.pigeonLevel());
        assertEquals(Verdict.OK, check.pigeonForward());
        assertEquals(0, check.pigeonForwardErrorDeg(), 3);
        assertEquals(Verdict.OK, check.pigeonYawSense().verdict());
        assertEquals(1, check.pigeonYawSense().gain(), 1e-6);
        assertEquals(Verdict.OK, check.imuYawSense().verdict());
        assertEquals(Verdict.OK, check.imuOrientation());
    }

    @Test
    void nothingIsJudgedWithoutEvidence() {
        rest(1);
        run(0.3, 6, 0, 0);
        assertEquals(Verdict.UNKNOWN, check.pigeonLevel());
        assertEquals(Verdict.UNKNOWN, check.pigeonForward());
        assertEquals(Verdict.UNKNOWN, check.pigeonYawSense().verdict());
        assertEquals(Verdict.UNKNOWN, check.imuOrientation());
    }

    @Test
    void pigeonFacingBackwardIsCaught() {
        pigeonTurnDeg = 180;
        accelerate();
        assertEquals(Verdict.WRONG, check.pigeonForward());
        assertEquals(180, Math.abs(check.pigeonForwardErrorDeg()), 3);
    }

    @Test
    void pigeonTurnedLeftReadsPlusNinety() {
        pigeonTurnDeg = 90;
        accelerate();
        assertEquals(Verdict.WRONG, check.pigeonForward());
        assertEquals(90, check.pigeonForwardErrorDeg(), 3);
    }

    @Test
    void aPigeonReadingNoAccelerationIsNotJudged() {
        pigeonTurnDeg = Double.NaN;
        accelerate();
        assertEquals(Verdict.UNKNOWN, check.pigeonForward());
    }

    @Test
    void reversedYawIsCaught() {
        pigeonYawSign = -1;
        imuYawSign = -1;
        spin();
        assertEquals(Verdict.WRONG, check.pigeonYawSense().verdict());
        assertEquals(-1, check.pigeonYawSense().gain(), 1e-6);
        assertEquals(Verdict.WRONG, check.imuYawSense().verdict());
    }

    @Test
    void upsideDownPigeonIsNotLevel() {
        pigeonRollDeg = 180;
        rest(3);
        assertEquals(Verdict.WRONG, check.pigeonLevel());
    }

    @Test
    void imuStandingUpButSetFlatIsCaught() {
        imuGravity = new double[] {G, 0, 0.3};
        rest(3);
        assertEquals(Verdict.WRONG, check.imuOrientation());

        check = new ImuMountCheck(false);
        rest(3);
        assertEquals(Verdict.OK, check.imuOrientation());
    }

    @Test
    void dropoutsGiveNoEvidence() {
        wheelsOk = false;
        rest(3);
        accelerate();
        spin();
        assertEquals(Verdict.UNKNOWN, check.pigeonLevel());
        assertEquals(Verdict.UNKNOWN, check.pigeonForward());
        assertEquals(Verdict.UNKNOWN, check.pigeonYawSense().verdict());
    }
}
