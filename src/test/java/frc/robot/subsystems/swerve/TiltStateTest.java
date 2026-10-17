package frc.robot.subsystems.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class TiltStateTest {
    @Test
    void tiltCombinesPitchAndRoll() {
        assertEquals(0, TiltState.tiltDegrees(0, 0), 1e-9);
        assertEquals(10, TiltState.tiltDegrees(10, 0), 1e-9);
        assertEquals(10, TiltState.tiltDegrees(0, -10), 1e-9);
        // 10 deg pitch and 10 deg roll: a bit over 14 deg from vertical.
        assertEquals(14.106, TiltState.tiltDegrees(10, 10), 1e-3);
    }

    @Test
    void odometryScaleFallsFromDeadbandToZero() {
        assertEquals(1.0, TiltState.odometryScale(0), 1e-9);
        assertEquals(1.0, TiltState.odometryScale(TiltState.TILT_DEADBAND_DEG), 1e-9);
        double mid = (TiltState.TILT_DEADBAND_DEG + TiltState.TILT_ZERO_DEG) / 2;
        assertEquals(0.5, TiltState.odometryScale(mid), 1e-9);
        assertEquals(0.0, TiltState.odometryScale(TiltState.TILT_ZERO_DEG), 1e-9);
        assertEquals(0.0, TiltState.odometryScale(40), 1e-9);
    }

    @Test
    void impactIgnoresGravityInAnyOrientationAndCatchesKnocks() {
        TiltState t = new TiltState();
        // Level and still: 1 g straight down.
        t.update(1.0, 0, 0, 0, 0, 0, 0, 1.0);
        assertEquals(0, t.impactG(), 1e-9);
        assertFalse(t.bumpedRecently(1.0));
        // Tilted 20 deg and still: gravity split across axes, still 1 g in total.
        double s = Math.sin(Math.toRadians(20));
        double c = Math.cos(Math.toRadians(20));
        t.update(2.0, 20, 0, 0, 0, s, 0, c);
        assertEquals(0, t.impactG(), 1e-9);
        assertTrue(t.tilted());
        // A 1.5 g sideways knock.
        t.update(3.0, 0, 0, 0, 0, 1.5, 0, 1.0);
        assertTrue(t.impactG() > TiltState.BUMP_G);
        assertTrue(t.bumpedRecently(3.2));
        assertFalse(t.bumpedRecently(3.0 + TiltState.BUMP_HOLD_SECONDS + 0.01));
    }

    @Test
    void noPigeonDataIsNotFreeFall() {
        TiltState t = new TiltState();
        t.update(1.0, 0, 0, 0, 0, 0, 0, 0);
        assertEquals(0, t.impactG(), 1e-9);
        assertFalse(t.bumpedRecently(1.0));
    }
}
