package frc.spectrumLib.hardware;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class RioTest {
    /** Loads the enum (its static init once threw on UNKNOWN's null serial) off the robot. */
    @Test
    void identifiesAsSimOffTheRobot() {
        assertEquals(Rio.SIM, Rio.id);
    }
}
