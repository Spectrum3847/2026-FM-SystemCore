package frc.robot.auton;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.robot.auton.AutoEnd.Reason;
import org.junit.jupiter.api.Test;

class AutoEndTest {

    /** A disable with the field attached, at this much match time left. */
    private static Reason fieldDisable(double matchTime) {
        return AutoEnd.classify(true, false, false, false, false, true, matchTime);
    }

    @Test
    void finishingIsFinished() {
        assertEquals(Reason.FINISHED, AutoEnd.classify(false, false, false, true, false, true, 5));
        assertEquals(Reason.FINISHED, AutoEnd.classify(false, false, false, false, false, true, 0));
    }

    @Test
    void anotherCommandInterrupts() {
        assertEquals(Reason.INTERRUPTED, AutoEnd.classify(true, true, false, true, false, true, 9));
    }

    @Test
    void cancelledWhileEnabledIsATimeoutOnlyWhenTheLimitPassed() {
        assertEquals(Reason.TIMED_OUT, AutoEnd.classify(true, false, true, true, false, false, -1));
        assertEquals(
                Reason.CANCELLED, AutoEnd.classify(true, false, false, true, false, false, -1));
    }

    @Test
    void eStopWins() {
        assertEquals(Reason.E_STOP, AutoEnd.classify(true, false, false, false, true, true, 12));
        assertEquals(Reason.E_STOP, AutoEnd.classify(true, false, false, false, true, true, 0));
    }

    @Test
    void theFieldEndingTheAutoIsEndOfPeriod() {
        // Every TXMAN1 auto that ran its 20 s was disabled with the match time at 0.
        assertEquals(Reason.END_OF_PERIOD, fieldDisable(0));
    }

    @Test
    void theFieldDisablingWithTimeLeftIsEarly() {
        // TXMAN1 Q30 (2.5 s in, 18 s left), Q24 (17.9 s in, 3 s left).
        assertEquals(Reason.DISABLED_BY_FMS_EARLY, fieldDisable(18));
        assertEquals(Reason.DISABLED_BY_FMS_EARLY, fieldDisable(3));
        assertEquals(Reason.DISABLED_BY_FMS_EARLY, fieldDisable(1));
    }

    @Test
    void aDriverStationWithoutAFieldDisables() {
        assertEquals(
                Reason.DISABLED_BY_DS,
                AutoEnd.classify(true, false, false, false, false, false, -1));
        assertEquals(
                Reason.DISABLED_BY_DS,
                AutoEnd.classify(true, false, false, false, false, false, 7));
        // A practice match timer running out is still the end of the period.
        assertEquals(
                Reason.END_OF_PERIOD, AutoEnd.classify(true, false, false, false, false, false, 0));
    }
}
