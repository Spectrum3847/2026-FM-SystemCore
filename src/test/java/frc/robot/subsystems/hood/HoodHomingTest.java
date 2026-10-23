package frc.robot.subsystems.hood;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.subsystems.hood.HoodHoming.Action;
import frc.robot.subsystems.hood.HoodHoming.Output;
import frc.robot.subsystems.hood.HoodHoming.State;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class HoodHomingTest {
    private static final double LOOP = 0.01;
    private static final double STOP = HoodHoming.Params.fm().stopDeg();

    private HoodHoming homing;
    private double now;

    // One loop's inputs, edited by each test before stepping.
    private boolean enabled;
    private boolean auto;
    private boolean teleop;
    private boolean connected;
    private boolean motorReset;
    private double position;
    private double speed;
    private boolean wantsHome;

    @BeforeEach
    void setUp() {
        homing = new HoodHoming(HoodHoming.Params.fm());
        now = 100;
        enabled = false;
        auto = false;
        teleop = true;
        connected = true;
        position = STOP;
        speed = 0;
        wantsHome = false;
    }

    private Output step() {
        Output out =
                homing.update(
                        new HoodHoming.Inputs(
                                now,
                                enabled,
                                auto,
                                teleop,
                                connected,
                                motorReset,
                                position,
                                speed,
                                wantsHome));
        now += LOOP;
        motorReset = false;
        return out;
    }

    private Output steps(int n) {
        Output out = null;
        for (int i = 0; i < n; i++) {
            out = step();
        }
        return out;
    }

    /** Steps until the hood zeroes, failing after {@code maxLoops}; returns that loop's output. */
    private Output stepUntilReset(int maxLoops) {
        for (int i = 0; i < maxLoops; i++) {
            Output out = step();
            if (out.resetPosition()) {
                return out;
            }
        }
        throw new AssertionError("never zeroed");
    }

    @Test
    void bootsNotZeroedAndHoldsInBrake() {
        Output out = steps(50);
        assertFalse(out.zeroed());
        assertEquals(State.NOT_ZEROED, out.state());
        assertEquals(Action.HOLD, out.action());
        assertFalse(homing.isZeroed());
    }

    @Test
    void teleopStartHomesAHoodAlreadyOnTheStop() {
        enabled = true;
        Output out = step();
        assertEquals(State.HOMING, out.state());
        assertEquals(Action.DRIVE_TO_STOP, out.action());
        assertFalse(out.zeroed());
        // Stalled from the first loop: 0.2 s at 10 ms loops is the 21st loop.
        assertFalse(steps(19).resetPosition());
        out = step();
        assertTrue(out.resetPosition());
        assertTrue(out.zeroed());
        assertEquals(State.ZEROED, out.state());
        assertEquals(Action.CLOSED_LOOP, out.action());
        assertFalse(step().resetPosition(), "zeroes once");
    }

    @Test
    void aHoodStartingHighIsNotTakenAsStalledBeforeItMoves() {
        enabled = true;
        position = 30;
        steps(5); // breaking away from rest: still reads 0 deg/s
        speed = -50;
        steps(40);
        speed = 0;
        Output out = stepUntilReset(100);
        assertTrue(out.zeroed());
        assertEquals(0.05 + 0.40 + 0.20, now - 100, 0.011);
    }

    @Test
    void aSlowWobbleDoesNotCountAsStalled() {
        enabled = true;
        speed = -50;
        for (int i = 0; i < 100; i++) {
            speed = i % 10 == 0 ? -50 : 0; // never stalled for 0.2 s
            assertFalse(step().resetPosition());
        }
    }

    @Test
    void timeoutRaisesTheAlertAndKeepsRefusingSetpoints() {
        enabled = true;
        speed = -50; // never stalls
        Output out = steps(300); // started on the first loop: 3 s later is the 301st
        assertEquals(State.HOMING, out.state());
        out = step();
        assertEquals(State.TIMED_OUT, out.state());
        assertTrue(out.timedOut());
        assertFalse(out.zeroed());
        assertEquals(Action.HOLD, out.action());
        // No retry until the next mode start.
        speed = 0;
        out = steps(100);
        assertEquals(Action.HOLD, out.action());
        assertEquals(State.TIMED_OUT, out.state());

        enabled = false;
        step();
        enabled = true;
        out = stepUntilReset(30);
        assertTrue(out.zeroed());
        assertFalse(out.timedOut(), "a success clears the alert");
    }

    @Test
    void neverHomesWhileDisabledOrOutsideAutoAndTeleop() {
        assertEquals(Action.HOLD, steps(100).action());
        teleop = false; // utility mode
        enabled = true;
        Output out = steps(100);
        assertEquals(State.NOT_ZEROED, out.state());
        assertEquals(Action.HOLD, out.action(), "refuses setpoints, does not home");
    }

    @Test
    void disablingMidHomingCancelsIt() {
        enabled = true;
        speed = -50;
        steps(10);
        enabled = false;
        Output out = step();
        assertEquals(State.NOT_ZEROED, out.state());
        assertEquals(Action.HOLD, out.action());
        speed = 0;
        assertFalse(steps(100).zeroed());
    }

    @Test
    void autoStartAlwaysHomesTeleopStartOnlyWhenNotZeroed() {
        enabled = true;
        stepUntilReset(30);
        enabled = false;
        step();

        teleop = true;
        enabled = true;
        Output out = step();
        assertEquals(State.ZEROED, out.state(), "teleop trusts an existing zero");
        enabled = false;
        step();

        teleop = false;
        auto = true;
        enabled = true;
        out = step();
        assertEquals(State.HOMING, out.state());
        assertFalse(out.zeroed(), "auto start re-homes and distrusts the old zero meanwhile");
        wantsHome = false; // a shot is wanted: an auto-start homing is not abortable
        assertEquals(State.HOMING, step().state());
        assertTrue(stepUntilReset(30).zeroed());
    }

    @Test
    void reHomesOnceEachTimeItIsSentHome() {
        enabled = true;
        stepUntilReset(30);
        position = 30;
        assertEquals(Action.CLOSED_LOOP, steps(10).action());

        wantsHome = true; // on its way down
        speed = -100;
        assertEquals(Action.CLOSED_LOOP, steps(10).action(), "not near home yet");
        position = STOP + 1.5;
        Output out = step();
        assertEquals(Action.DRIVE_TO_STOP, out.action());
        assertTrue(out.zeroed(), "an existing zero stands while re-homing");
        position = STOP + 0.4; // the old zero was 0.4 deg off
        speed = 0;
        out = stepUntilReset(30);
        assertEquals(Action.HOLD, out.action(), "rests on the stop");
        out = steps(200);
        assertEquals(Action.HOLD, out.action());
        assertEquals(State.ZEROED, out.state(), "and does not re-home again on this visit");

        wantsHome = false;
        position = 20;
        assertEquals(Action.CLOSED_LOOP, step().action());
        wantsHome = true;
        position = STOP;
        assertEquals(Action.DRIVE_TO_STOP, step().action(), "the next visit re-homes");
    }

    @Test
    void aReHomeStopsWhenAShotIsWanted() {
        enabled = true;
        stepUntilReset(30);
        wantsHome = true;
        assertEquals(Action.DRIVE_TO_STOP, step().action());
        wantsHome = false;
        Output out = step();
        assertEquals(Action.CLOSED_LOOP, out.action());
        assertTrue(out.zeroed());
        assertFalse(out.resetPosition());
    }

    @Test
    void aFailedReHomeKeepsTheZeroItStartedWith() {
        enabled = true;
        stepUntilReset(30);
        wantsHome = true;
        speed = -50;
        Output out = steps(301);
        assertEquals(State.ZEROED, out.state());
        assertTrue(out.timedOut());
        assertEquals(Action.HOLD, out.action(), "no retry loop against the stop");
    }

    @Test
    void aDisconnectedMotorNeverZeroes() {
        connected = false; // its speed reads 0, which would look like a stall
        enabled = true;
        Output out = steps(100);
        assertFalse(out.zeroed());
        assertEquals(Action.HOLD, out.action());
        connected = true; // the request waits for it
        assertTrue(stepUntilReset(30).zeroed());

        enabled = false;
        step();
        teleop = false;
        auto = true;
        enabled = true;
        step();
        connected = false;
        out = step();
        assertEquals(Action.HOLD, out.action(), "drops out mid-homing: stop pushing");
        assertFalse(steps(100).zeroed());
    }

    @Test
    void aMotorRebootWhileEnabledUnZeroesAndReHomesAtOnce() {
        enabled = true;
        assertTrue(stepUntilReset(30).zeroed());
        position = 30; // aiming
        assertEquals(Action.CLOSED_LOOP, steps(50).action());

        motorReset = true;
        position = 0; // restarted from the boot value
        Output out = step();
        assertFalse(out.zeroed());
        assertFalse(homing.isZeroed());
        assertEquals(Action.DRIVE_TO_STOP, out.action());
        speed = -50;
        steps(40);
        speed = 0;
        out = stepUntilReset(30);
        assertTrue(out.zeroed());
        assertEquals(Action.CLOSED_LOOP, out.action());
    }

    @Test
    void aMotorRebootWhileDisabledReHomesAtTheNextModeStart() {
        enabled = true;
        assertTrue(stepUntilReset(30).zeroed());
        enabled = false;
        steps(10);

        motorReset = true;
        Output out = step();
        assertFalse(out.zeroed());
        assertEquals(Action.HOLD, out.action(), "never homes while disabled");
        assertEquals(Action.HOLD, steps(50).action());

        enabled = true; // teleop start: not zeroed, so it homes
        assertEquals(Action.DRIVE_TO_STOP, step().action());
        assertTrue(stepUntilReset(30).zeroed());
    }

    @Test
    void aMotorRebootMidHomingRestartsTheAttempt() {
        enabled = true;
        speed = -50;
        steps(10);
        speed = 0; // the reboot's standstill, not the stop
        steps(10);
        motorReset = true;
        Output out = step();
        assertEquals(Action.DRIVE_TO_STOP, out.action());
        assertFalse(steps(18).resetPosition(), "the stall clock restarted with the attempt");
        assertTrue(stepUntilReset(5).zeroed());
    }

    @Test
    void aMotorRebootAbortsAReHomeOnStowAndRunsAFullOne() {
        enabled = true;
        assertTrue(stepUntilReset(30).zeroed());
        position = 30;
        steps(5);
        motorReset = true;
        wantsHome = true;
        position = STOP; // reads near the stop only because it restarted there
        Output out = step();
        assertEquals(Action.DRIVE_TO_STOP, out.action());
        wantsHome = false; // a shot is wanted: an un-zeroed hood keeps homing
        assertEquals(Action.DRIVE_TO_STOP, steps(5).action());
    }
}
