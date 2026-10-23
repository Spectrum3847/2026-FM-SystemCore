package frc.spectrumLib.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.spectrumLib.checks.CheckResult;
import frc.spectrumLib.checks.CheckResult.Status;
import frc.spectrumLib.localization.ImuMountCheck.Verdict;
import java.util.ArrayDeque;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class ImuMountCheckTest {
    private static final double DT = 0.02;
    private static final double G = 9.81;

    private ImuMountCheck check;
    private double time;
    private double heading;

    // How the gyros are really mounted, and where the robot is, edited per test.
    private double pigeonTurnDeg;
    private double pigeonYawSign;
    private double imuYawSign;
    private double pigeonRollDeg;
    private double[] imuGravity;
    private boolean wheelsOk;
    private boolean onCart;
    private boolean pigeonReadsNoAccel;
    private boolean jostled;
    private double vx;
    private double vy;

    // The Pigeon feels the robot's acceleration 0.1 s after the wheels show it, as on FM.
    private final ArrayDeque<double[]> felt = new ArrayDeque<>();

    @BeforeEach
    void setUp() {
        check = new ImuMountCheck(true, 0);
        time = 10;
        heading = 0;
        pigeonTurnDeg = 0;
        pigeonYawSign = 1;
        imuYawSign = 1;
        pigeonRollDeg = 0;
        imuGravity = new double[] {0, 0, G};
        wheelsOk = true;
        onCart = false;
        pigeonReadsNoAccel = false;
        jostled = false;
        vx = 0;
        vy = 0;
        felt.clear();
        for (int i = 0; i < 5; i++) {
            felt.add(new double[2]);
        }
    }

    /**
     * Runs the wheels at a constant acceleration (robot frame) and turn rate, from their velocity.
     * On a cart the wheels spin but the robot, and so the gyros, do not move.
     */
    private void run(double seconds, double ax, double ay, double omega) {
        for (int i = 0; i < Math.round(seconds / DT); i++) {
            time += DT;
            vx += ax * DT;
            vy += ay * DT;
            felt.add(new double[] {onCart ? 0 : ax, onCart ? 0 : ay});
            double[] lagged = felt.remove();
            double feltX = lagged[0];
            double feltY = lagged[1];
            if (!onCart) {
                heading += omega * DT;
            }
            double turn = Math.toRadians(pigeonTurnDeg);
            if (jostled) {
                // Knocks from every direction, unrelated to the wheels.
                turn = time * 7.3;
            }
            double px = (feltX * Math.cos(turn) + feltY * Math.sin(turn)) / G;
            double py = (-feltX * Math.sin(turn) + feltY * Math.cos(turn)) / G;
            if (pigeonReadsNoAccel) {
                px = 0;
                py = 0;
            }
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

    private void driveThenRest() {
        accelerate();
        spin();
        rest(3);
    }

    @Test
    void correctMountsPass() {
        driveThenRest();
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
    void wheelsSpinningOnACartGiveNoVerdict() {
        onCart = true;
        // Even a badly mounted robot: nothing moves with the wheels, so nothing is judged.
        pigeonRollDeg = 90;
        imuGravity = new double[] {G, 0, 0};
        rest(3);
        driveThenRest();
        assertEquals(Verdict.UNKNOWN, check.pigeonLevel());
        assertEquals(Verdict.UNKNOWN, check.pigeonForward());
        assertEquals(Verdict.UNKNOWN, check.pigeonYawSense().verdict());
        assertEquals(Verdict.UNKNOWN, check.imuYawSense().verdict());
        assertEquals(Verdict.UNKNOWN, check.imuOrientation());
    }

    @Test
    void restsCountOnlyAfterDrivingOnTheFloor() {
        pigeonRollDeg = 90;
        rest(5);
        assertEquals(Verdict.UNKNOWN, check.pigeonLevel());
        spin();
        rest(3);
        assertEquals(Verdict.WRONG, check.pigeonLevel());
    }

    @Test
    void aParkedTiltIsNotAMountError() {
        pigeonRollDeg = 15;
        driveThenRest();
        assertEquals(Verdict.OK, check.pigeonLevel());
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
        pigeonReadsNoAccel = true;
        accelerate();
        assertEquals(Verdict.UNKNOWN, check.pigeonForward());
    }

    @Test
    void accelerationUnrelatedToTheWheelsIsNotJudged() {
        jostled = true;
        accelerate();
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
        driveThenRest();
        assertEquals(Verdict.WRONG, check.pigeonLevel());
    }

    @Test
    void imuStandingUpButSetFlatIsCaught() {
        imuGravity = new double[] {G, 0, 0.3};
        driveThenRest();
        assertEquals(Verdict.WRONG, check.imuOrientation());

        check = new ImuMountCheck(false, 0);
        driveThenRest();
        assertEquals(Verdict.OK, check.imuOrientation());
    }

    @Test
    void aLongTippedRestAfterLevelRestsIsNotAMountError() {
        driveThenRest();
        accelerate();
        rest(3);
        spin();
        // Tipped onto a bumper, or lifted, with the wheels still: much longer than the level rests.
        pigeonRollDeg = 70;
        imuGravity = new double[] {G * 0.94, 0, G * 0.34};
        rest(40);
        assertEquals(Verdict.OK, check.pigeonLevel());
        assertEquals(0, check.pigeonRestRollDeg(), 1e-9);
        assertEquals(Verdict.OK, check.imuOrientation());
        assertEquals(1, check.imuGravityZShare(), 1e-9);
    }

    @Test
    void aMountErrorReadsWrongInEveryRest() {
        pigeonRollDeg = 70;
        imuGravity = new double[] {G, 0, 0.3};
        driveThenRest();
        accelerate();
        rest(10);
        assertEquals(Verdict.WRONG, check.pigeonLevel());
        assertEquals(Verdict.WRONG, check.imuOrientation());
    }

    @Test
    void anImuReadingNoGravityIsNotJudged() {
        imuGravity = new double[] {0, 0, 0};
        driveThenRest();
        assertEquals(Verdict.UNKNOWN, check.imuOrientation());
        assertEquals(Double.NaN, check.imuGravityZShare());

        check = new ImuMountCheck(true, 0);
        imuGravity = new double[] {Double.NaN, Double.NaN, Double.NaN};
        driveThenRest();
        assertEquals(Verdict.UNKNOWN, check.imuOrientation());
        // A rest that reads gravity again is judged.
        imuGravity = new double[] {0, 0, G};
        spin();
        rest(3);
        assertEquals(Verdict.OK, check.imuOrientation());
    }

    @Test
    void preMatchSkipsWithoutDriving() {
        rest(5);
        assertEquals(Status.SKIP, check.evaluate(time).status());
        assertEquals("not enough driving since boot", check.evaluate(time).reason());
    }

    @Test
    void preMatchPassesCorrectMounts() {
        driveThenRest();
        CheckResult r = check.evaluate(time);
        assertEquals(Status.PASS, r.status(), r.reason());
    }

    @Test
    void preMatchWarnsOnAWrongForward() {
        check = new ImuMountCheck(true, 0);
        pigeonTurnDeg = 180;
        driveThenRest();
        CheckResult r = check.evaluate(time);
        assertEquals(Status.WARN, r.status(), r.reason());
        assertTrue(r.reason().contains("180 deg from the robot's"), r.reason());
    }

    @Test
    void preMatchFailsOnAReversedYaw() {
        pigeonYawSign = -1;
        driveThenRest();
        CheckResult r = check.evaluate(time);
        assertEquals(Status.FAIL, r.status(), r.reason());
        assertTrue(r.reason().startsWith("Pigeon yaw turns against the wheels"), r.reason());
    }

    @Test
    void dropoutsGiveNoEvidence() {
        wheelsOk = false;
        driveThenRest();
        assertEquals(Verdict.UNKNOWN, check.pigeonLevel());
        assertEquals(Verdict.UNKNOWN, check.pigeonForward());
        assertEquals(Verdict.UNKNOWN, check.pigeonYawSense().verdict());
    }
}
