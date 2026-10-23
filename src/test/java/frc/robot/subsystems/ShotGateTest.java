package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.subsystems.ShotGate.Blocker;
import frc.robot.subsystems.ShotGate.Decision;
import frc.robot.subsystems.ShotGate.FeedReason;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class ShotGateTest {
    private static final double LOOP = 0.01;
    private static final double TARGET_RPM = 2500;
    private static final double TARGET_HOOD = 20;
    private static final double AIM_TOL = Math.toRadians(5);

    private ShotGate gate;
    private double now;

    // One loop's inputs, edited by each test before stepping.
    private boolean launching;
    private boolean override;
    private double hood;
    private double hoodSpeed;
    private boolean hoodZeroed;
    private boolean checkAim;
    private double headingError;
    private boolean checkRange;
    private boolean valid;

    @BeforeEach
    void setUp() {
        gate = new ShotGate(ShotGate.Tolerances.fm());
        now = 100;
        launching = true;
        override = false;
        hood = TARGET_HOOD;
        hoodSpeed = 0;
        hoodZeroed = true;
        checkAim = true;
        headingError = 0;
        checkRange = true;
        valid = true;
    }

    private Decision step() {
        Decision d =
                gate.update(
                        new ShotGate.Inputs(
                                launching,
                                override,
                                now,
                                TARGET_RPM,
                                hood,
                                hoodSpeed,
                                TARGET_HOOD,
                                hoodZeroed,
                                checkAim,
                                headingError,
                                AIM_TOL,
                                checkRange,
                                valid));
        now += LOOP;
        return d;
    }

    private Decision steps(int n) {
        Decision d = null;
        for (int i = 0; i < n; i++) {
            d = step();
        }
        return d;
    }

    @Test
    void readyShotFeedsOnceSettled() {
        Decision first = step();
        assertTrue(first.ready());
        assertFalse(first.feed(), "not on the first loop");
        assertEquals(Blocker.SETTLING, first.blocker());
        // 0.04 s of settling at 10 ms loops: the fifth loop feeds.
        Decision d = steps(4);
        assertTrue(d.feed());
        assertEquals(FeedReason.READY, d.reason());
        assertEquals(Blocker.NONE, d.blocker());
    }

    @Test
    void notLaunchingNeverFeedsButStillSaysWhetherItWouldBeReady() {
        launching = false;
        Decision d = steps(20);
        assertFalse(d.feed());
        assertTrue(d.ready());
        assertEquals(FeedReason.NOT_LAUNCHING, d.reason());

        hood = TARGET_HOOD + 5;
        d = step();
        assertFalse(d.ready());
        assertEquals(Blocker.HOOD, d.blocker());
    }

    @Test
    void eachCheckNamesItself() {
        hood = TARGET_HOOD + 1.5;
        assertEquals(Blocker.HOOD, step().blocker());
        hood = TARGET_HOOD - 0.9;
        headingError = -Math.toRadians(6);
        assertEquals(Blocker.AIM, step().blocker());
        headingError = Math.toRadians(4.9);
        valid = false;
        assertEquals(Blocker.RANGE, step().blocker());
    }

    @Test
    void aHoodSwingingThroughItsTargetIsNotReady() {
        hoodSpeed = 35; // on target this instant, on its way through an overshoot
        assertEquals(Blocker.HOOD, step().blocker());
        hoodSpeed = -19;
        assertTrue(step().ready());
        // Once feeding, the hood moving is fine: the keep checks only look at where it is.
        steps(4);
        hoodSpeed = 60;
        assertTrue(step().feed());
    }

    @Test
    void anUnhomedHoodBlocksAheadOfItsAngle() {
        hoodZeroed = false;
        hood = TARGET_HOOD + 5;
        Decision d = step();
        assertFalse(d.ready());
        assertEquals(Blocker.HOOD_NOT_ZEROED, d.blocker());
        assertEquals("HoodNotZeroed", d.blocker().label);
        hood = TARGET_HOOD;
        assertFalse(steps(5).feed(), "even on target: the angle cannot be trusted");
        hoodZeroed = true;
        assertTrue(steps(5).feed());
    }

    @Test
    void anUnhomedHoodStillTimesOut() {
        hoodZeroed = false;
        assertFalse(steps(100).feed());
        Decision d = step();
        assertTrue(d.feed());
        assertEquals(FeedReason.TIMEOUT, d.reason());
        assertEquals(Blocker.HOOD_NOT_ZEROED, d.blocker());
    }

    @Test
    void aimAndRangeOnlyVoteWhenAsked() {
        checkAim = false; // the set shot: the driver aims
        headingError = Math.toRadians(90);
        checkRange = false; // pose not trusted, or the set shot
        valid = false;
        assertTrue(steps(5).feed());
    }

    @Test
    void aMissingTargetBlocksRatherThanPasses() {
        hood = Double.NaN;
        Decision d = step();
        assertFalse(d.ready());
        assertEquals(Blocker.HOOD, d.blocker());
    }

    @Test
    void volleyIsNotChoppedByARangeDropout() {
        assertTrue(steps(5).feed());
        // One bad pose frame mid-volley does not chop it.
        valid = false;
        assertTrue(step().feed());
    }

    @Test
    void headingKeepToleranceIsTwiceTheStartOne() {
        assertTrue(steps(5).feed());
        headingError = Math.toRadians(9); // outside 5, inside 10
        assertTrue(step().feed());
        headingError = Math.toRadians(11);
        assertFalse(step().feed());
    }

    @Test
    void timeoutFeedsAfterOneSecondAndStaysOpenForTheLaunch() {
        headingError = Math.toRadians(30); // never aimed
        // Held from the first loop: 100 loops later is 0.99 s.
        Decision d = steps(100);
        assertFalse(d.feed());
        assertEquals(Blocker.AIM, d.blocker());
        assertEquals(0.99, d.secondsHeld(), 1e-9);
        d = step();
        assertTrue(d.feed(), "held for a full second");
        assertEquals(FeedReason.TIMEOUT, d.reason());
        assertEquals(Blocker.AIM, d.blocker(), "and still says what it bypassed");
        assertTrue(steps(50).feed()); // the timeout holds for the rest of the launch
        // Leaving the launch state resets it.
        launching = false;
        step();
        launching = true;
        assertFalse(step().feed());
    }

    @Test
    void timeoutCountsFromWhenTheVolleyStopped() {
        assertTrue(steps(5).feed());
        steps(200); // a long volley
        headingError = Math.toRadians(30);
        assertFalse(steps(100).feed());
        assertTrue(step().feed());
    }

    @Test
    void overrideFeedsImmediatelyAndOnlyWhileHeld() {
        hood = TARGET_HOOD + 5;
        override = true;
        Decision d = step();
        assertTrue(d.feed());
        assertEquals(FeedReason.OVERRIDE, d.reason());
        assertEquals(Blocker.HOOD, d.blocker());
        override = false;
        assertFalse(step().feed());
    }

    @Test
    void noShotIsItsOwnReason() {
        launching = false;
        Decision d =
                gate.update(
                        new ShotGate.Inputs(
                                false,
                                false,
                                now,
                                0,
                                9,
                                0,
                                Double.NaN,
                                false,
                                false,
                                Double.NaN,
                                AIM_TOL,
                                false,
                                false));
        assertEquals(Blocker.NO_SHOT, d.blocker());
        assertFalse(d.ready());
    }

    @Test
    void aimToleranceIsTheAngleAtWhichTheBallReachesTheRim() {
        double rim = ShotGate.HUB_OPENING_RADIUS_METERS - ShotGate.FUEL_DIAMETER_METERS;
        assertEquals(0.3796, rim, 1e-3);
        assertEquals(Math.atan(rim / 3.0), ShotGate.aimToleranceRad(3.0, false), 1e-12);
        assertEquals(7.2, Math.toDegrees(ShotGate.aimToleranceRad(3.0, false)), 0.05);
        assertEquals(4.3, Math.toDegrees(ShotGate.aimToleranceRad(5.0, false)), 0.05);
        // Clamped: close in it would be 14 deg, far out under 2.
        assertEquals(10.0, Math.toDegrees(ShotGate.aimToleranceRad(1.5, false)), 1e-9);
        assertEquals(2.0, Math.toDegrees(ShotGate.aimToleranceRad(20, false)), 1e-9);
        // Feed shots land on a patch of floor: 10 deg wherever.
        assertEquals(10.0, Math.toDegrees(ShotGate.aimToleranceRad(8, true)), 1e-9);
        assertEquals(10.0, Math.toDegrees(ShotGate.aimToleranceRad(Double.NaN, false)), 1e-9);
    }
}
