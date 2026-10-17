package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class ShotLogDipCounterTest {
    private static final double TARGET = 2500;

    private static int count(double... rpms) {
        ShotLog.DipCounter c = new ShotLog.DipCounter();
        for (double rpm : rpms) {
            c.update(rpm, TARGET);
        }
        return c.count();
    }

    @Test
    void eachBallIsOneDipAndRecovery() {
        // Three balls: on speed, dip, recover; noise inside the window is not a ball.
        assertEquals(3, count(2500, 2320, 2480, 2300, 2460, 2490, 2200, 2400, 2470, 2480, 2510));
    }

    @Test
    void aSlowRecoveryIsNotCountedTwice() {
        // Down past the threshold, part way back (not within half of it), down again: one ball.
        assertEquals(1, count(2500, 2320, 2400, 2320, 2410, 2460));
    }

    @Test
    void aVolleyThatStartsLowCountsNothingUntilItHasBeenOnSpeed() {
        // Fed by the timeout with the flywheel still spinning up: no phantom balls.
        assertEquals(0, count(1800, 1900, 2100, 2200));
        assertEquals(1, count(1800, 2460, 2300));
    }

    @Test
    void noTargetNoCount() {
        ShotLog.DipCounter c = new ShotLog.DipCounter();
        c.update(2500, 0);
        c.update(0, 0);
        assertEquals(0, c.count());
    }
}
