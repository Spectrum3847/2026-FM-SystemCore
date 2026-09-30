package frc.robot.pilot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

class FaceChordFilterTest {
    private static final double LOOP = 0.01; // 100 Hz

    /** One loop's buttons: {lb, a, x, b}, held for {@code loops} loops. */
    private record Hold(boolean lb, boolean a, boolean x, boolean b, int loops) {}

    private static Hold hold(String buttons, int loops) {
        return new Hold(
                buttons.contains("LB"),
                buttons.contains("A"),
                buttons.contains("X"),
                buttons.contains("B") && !buttons.contains("LB") || buttons.contains("+B"),
                loops);
    }

    /**
     * Replays button holds through the filter the way Robot binds the result: a new answer's rising
     * edge requests it, and becoming NONE requests IDLE. Returns "REQUEST@loop" in order.
     */
    private static List<String> requests(Hold... holds) {
        FaceChordFilter filter = new FaceChordFilter();
        List<String> out = new ArrayList<>();
        FaceChord last = FaceChord.NONE;
        int loop = 0;
        for (Hold h : holds) {
            for (int i = 0; i < h.loops(); i++, loop++) {
                double now = 10.0 + loop * LOOP;
                FaceChord raw = FaceChord.of(h.lb(), h.a(), h.x(), h.b(), true);
                FaceChord out1 = filter.update(now, raw);
                // Several triggers poll it each loop: the answer must not change within a loop.
                assertEquals(out1, filter.update(now, raw));
                if (out1 != last) {
                    out.add((out1 == FaceChord.NONE ? "IDLE" : out1.name()) + "@" + loop);
                }
                last = out1;
            }
        }
        return out;
    }

    private static List<String> names(List<String> requests) {
        return requests.stream().map(r -> r.substring(0, r.indexOf('@'))).toList();
    }

    @Test
    void aLetGoTwoLoopsAfterLbDoesNotUnjam() {
        var r = requests(hold("LB A", 50), hold("A", 2), hold("", 20));
        assertEquals(List.of("SET_SHOT_TOWER", "IDLE"), names(r));
        assertEquals("IDLE@50", r.get(1)); // stops the moment LB is let go
    }

    @Test
    void xLetGoTwoLoopsAfterLbDoesNotTrack() {
        assertEquals(
                List.of("SET_SHOT_LEFT_TRENCH", "IDLE"),
                names(requests(hold("LB X", 50), hold("X", 2), hold("", 20))));
    }

    @Test
    void aPressedTwoLoopsBeforeLbDoesNotUnjam() {
        assertEquals(
                List.of("SET_SHOT_TOWER", "IDLE"),
                names(requests(hold("A", 2), hold("LB A", 50), hold("", 5))));
    }

    @Test
    void aDeliberateUnjamStartsAfterTheSettleTime() {
        var r = requests(hold("A", 50), hold("", 5));
        assertEquals(List.of("UNJAM", "IDLE"), names(r));
        int start = Integer.parseInt(r.get(0).substring(r.get(0).indexOf('@') + 1));
        double seconds = start * LOOP;
        assertTrue(
                seconds >= FaceChordFilter.SETTLE_SECONDS - 1e-9
                        && seconds <= FaceChordFilter.SETTLE_SECONDS + 2 * LOOP,
                "unjam started at " + seconds + " s");
    }

    @Test
    void aTapShorterThanTheSettleTimeDoesNothing() {
        assertEquals(List.of(), requests(hold("A", 5), hold("", 20)));
    }

    @Test
    void holdingXThenPressingAKeepsTrackingUntilUnjamTakesOver() {
        // No IDLE between tracking and unjam.
        assertEquals(
                List.of("TRACK_TARGET", "UNJAM", "IDLE"),
                names(requests(hold("X", 30), hold("A X", 30), hold("", 5))));
    }

    @Test
    void lettingGoOfLbWithXHeldIsIdleThenTracking() {
        assertEquals(
                List.of("SET_SHOT_LEFT_TRENCH", "IDLE", "TRACK_TARGET", "IDLE"),
                names(requests(hold("LB X", 30), hold("X", 30), hold("", 5))));
    }

    @Test
    void chordsActImmediately() {
        var r = requests(hold("LB +B", 10), hold("", 5));
        assertEquals("SET_SHOT_RIGHT_TRENCH@0", r.get(0));
    }
}
