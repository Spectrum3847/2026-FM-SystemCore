package frc.robot.auton;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.robot.auton.AutoEnd.Reason;
import org.junit.jupiter.api.Test;

class AutoEndTest {

    /** A disable with the field attached, at this much match time left. */
    private static Reason fieldDisable(double matchTime) {
        return AutoEnd.classify(true, false, false, false, false, true, true, matchTime);
    }

    @Test
    void finishingIsFinished() {
        assertEquals(
                Reason.FINISHED, AutoEnd.classify(false, false, false, true, false, true, true, 5));
        assertEquals(
                Reason.FINISHED,
                AutoEnd.classify(false, false, false, false, false, true, true, 0));
    }

    @Test
    void anotherCommandInterrupts() {
        assertEquals(
                Reason.INTERRUPTED,
                AutoEnd.classify(true, true, false, true, false, true, true, 9));
    }

    @Test
    void cancelledWhileEnabledIsATimeoutOnlyWhenTheLimitPassed() {
        assertEquals(
                Reason.TIMED_OUT,
                AutoEnd.classify(true, false, true, true, false, false, true, -1));
        assertEquals(
                Reason.CANCELLED,
                AutoEnd.classify(true, false, false, true, false, false, true, -1));
    }

    @Test
    void eStopWins() {
        assertEquals(
                Reason.E_STOP, AutoEnd.classify(true, false, false, false, true, true, true, 12));
        assertEquals(
                Reason.E_STOP, AutoEnd.classify(true, false, false, false, true, true, true, 0));
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
                AutoEnd.classify(true, false, false, false, false, false, true, -1));
        assertEquals(
                Reason.DISABLED_BY_DS,
                AutoEnd.classify(true, false, false, false, false, false, true, 7));
        // A practice match timer running out is still the end of the period.
        assertEquals(
                Reason.END_OF_PERIOD,
                AutoEnd.classify(true, false, false, false, false, false, true, 0));
    }

    @Test
    void aLostDriverStationIsADisconnect() {
        // Field attached with time left, or none: either way the DS stopped answering.
        assertEquals(
                Reason.DS_DISCONNECTED,
                AutoEnd.classify(true, false, false, false, false, true, false, 12));
        assertEquals(
                Reason.DS_DISCONNECTED,
                AutoEnd.classify(true, false, false, false, false, false, false, -1));
        // Even with the match time at 0, a lost DS is not the field ending the period.
        assertEquals(
                Reason.DS_DISCONNECTED,
                AutoEnd.classify(true, false, false, false, false, true, false, 0));
        // An E-stop or another command interrupting still says so.
        assertEquals(
                Reason.E_STOP, AutoEnd.classify(true, false, false, false, true, true, false, 5));
        assertEquals(
                Reason.INTERRUPTED,
                AutoEnd.classify(true, true, false, false, false, true, false, 5));
        // Finishing on its own the loop the DS drops is still finishing.
        assertEquals(
                Reason.FINISHED,
                AutoEnd.classify(false, false, false, false, false, true, false, 5));
    }
}
