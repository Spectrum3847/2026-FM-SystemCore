package frc.spectrumLib.mechanism;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class PositionWriteTest {
    private static final double LOOP = 0.01;
    private final PositionWrite write = new PositionWrite();
    private double now = 50;

    private double step(boolean connected, double position, double velocity) {
        now += LOOP;
        return write.update(now, connected, position, velocity);
    }

    @Test
    void pendingUntilTheReadingsShowTheWrite() {
        write.sent(now, 0.025, 0.031);
        assertTrue(write.isPending());
        assertTrue(Double.isNaN(step(true, 0.031, 0)), "old frame: still pending, no resend yet");
        assertTrue(write.isPending());
        step(true, 0.025, 0);
        assertFalse(write.isPending());
    }

    @Test
    void aWriteToWhereItAlreadyReadsIsNotPending() {
        write.sent(now, 0.025, 0.025 + 0.5 * PositionWrite.SAME_POSITION_ROTATIONS);
        assertFalse(write.isPending());
    }

    @Test
    void motionMeanwhileMovesBothFrames() {
        // Written at 0 while reading 1, then moving at 2 rot/s: after two loops the old frame
        // reads 1.04, the new one 0.04.
        write.sent(now, 0, 1);
        step(true, 1.02, 2);
        step(true, 1.04, 2);
        assertTrue(write.isPending());
        step(true, 0.06, 2);
        assertFalse(write.isPending());
    }

    @Test
    void aDisconnectedMotorConfirmsNothing() {
        write.sent(now, 0.025, 0.031);
        // A dropped device reads 0, nearer the new frame than the old: not proof of anything.
        for (int i = 0; i < 50; i++) {
            assertTrue(Double.isNaN(step(false, 0, 0)));
        }
        assertTrue(write.isPending());
    }

    @Test
    void aWriteTheReadingsNeverShowIsSentAgain() {
        write.sent(now, 0.025, 0.031);
        double resend = Double.NaN;
        int loops = 0;
        while (Double.isNaN(resend) && loops < 100) {
            resend = step(true, 0.031, 0);
            loops++;
        }
        assertEquals(0.025, resend, 1e-12);
        assertEquals(Math.round(PositionWrite.RESEND_SECONDS / LOOP), loops);
        assertEquals(1, write.getResends());
        assertTrue(write.isPending(), "still holding until the readings show it");
        step(true, 0.025, 0);
        assertFalse(write.isPending());
    }

    @Test
    void aNewerWriteReplacesAPendingOne() {
        write.sent(now, 0.025, 0.031);
        write.sent(now, 0.040, 0.031);
        step(true, 0.025, 0);
        assertTrue(write.isPending(), "the first write's position is not the one wanted now");
        step(true, 0.040, 0);
        assertFalse(write.isPending());
    }
}
