package frc.spectrumLib.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class YawRateHistoryTest {
    @Test
    void peakLooksBackOverTheWindowOnly() {
        YawRateHistory h = new YawRateHistory(64);
        h.record(0.00, 3.0); // a spin
        h.record(0.20, 0.1); // stopped
        h.record(0.40, 0.0);
        // A frame gated at 0.25 s was captured during the spin: the gate must still see it.
        assertEquals(3.0, h.peak(0.25, 0.3), 1e-9);
        // By 0.45 s the spin is outside the 0.3 s window.
        assertEquals(0.1, h.peak(0.45, 0.3), 1e-9);
    }

    @Test
    void negativeRatesCountByMagnitude() {
        YawRateHistory h = new YawRateHistory(8);
        h.record(1.0, -2.5);
        assertEquals(2.5, h.peak(1.1, 0.3), 1e-9);
    }
}
