package frc.spectrumLib.gamepads;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BiConsumer;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.wpilib.command2.button.Trigger;
import org.wpilib.hardware.hal.HAL;

/**
 * FM's pilot face chords (Pilot): LB + A, X, B for the set shots, teleop only, and A and X bare for
 * unjam and track target. Then the same helper with other modifiers.
 */
class ModifierChordsTest {
    private static final double LOOP = 0.01; // 100 Hz
    private static final double SETTLE = ModifierChords.DEFAULT_SETTLE_SECONDS;

    @BeforeAll
    static void initHal() {
        assertTrue(HAL.initialize());
    }

    /** The buttons a test holds, read by the group as the robot reads the controller. */
    private static final class Pad {
        double now = 10.0;
        boolean lb, rb, lt, a, x, b, teleop = true;

        void hold(String buttons) {
            List<String> held = List.of(buttons.split(" "));
            lb = held.contains("LB");
            rb = held.contains("RB");
            lt = held.contains("LT");
            a = held.contains("A");
            x = held.contains("X");
            b = held.contains("B");
        }
    }

    /** A group and its named triggers, in the order they were made. */
    private static final class Group {
        final Pad pad = new Pad();
        final ModifierChords chords;
        final Map<String, Trigger> triggers = new LinkedHashMap<>();
        final Trigger any;

        Group(double settleSeconds, BiConsumer<Group, Pad> bind) {
            chords = new ModifierChords(() -> pad.now, settleSeconds);
            bind.accept(this, pad);
            any = chords.any();
        }

        /** The one trigger that is true, IDLE if none; checks that no other is. */
        String active() {
            String on = "IDLE";
            int count = 0;
            for (var e : triggers.entrySet()) {
                if (e.getValue().getAsBoolean()) {
                    on = e.getKey();
                    count++;
                }
            }
            assertTrue(count <= 1, "more than one trigger true: " + count);
            assertEquals(count == 1, any.getAsBoolean());
            return on;
        }
    }

    private static Group fm(double settleSeconds) {
        return new Group(
                settleSeconds,
                (g, p) -> {
                    var c = g.chords;
                    g.triggers.put(
                            "SET_SHOT_TOWER", c.chord(() -> p.lb, () -> p.a, () -> p.teleop));
                    g.triggers.put(
                            "SET_SHOT_LEFT_TRENCH", c.chord(() -> p.lb, () -> p.x, () -> p.teleop));
                    g.triggers.put(
                            "SET_SHOT_RIGHT_TRENCH",
                            c.chord(() -> p.lb, () -> p.b, () -> p.teleop));
                    g.triggers.put("UNJAM", c.bare(() -> p.a));
                    g.triggers.put("TRACK_TARGET", c.bare(() -> p.x));
                });
    }

    private record Hold(String buttons, int loops) {}

    private static Hold hold(String buttons, int loops) {
        return new Hold(buttons, loops);
    }

    /**
     * Replays holds the way Robot binds the triggers: each change of the active one is a request,
     * IDLE when none. Returns "REQUEST@loop" in order.
     */
    private static List<String> requests(Group g, Hold... holds) {
        List<String> out = new ArrayList<>();
        String last = "IDLE";
        int loop = 0;
        for (Hold h : holds) {
            g.pad.hold(h.buttons());
            for (int i = 0; i < h.loops(); i++, loop++) {
                g.pad.now = 10.0 + loop * LOOP;
                String now = g.active();
                // Several triggers poll it each loop.
                assertEquals(now, g.active());
                if (!now.equals(last)) {
                    out.add(now + "@" + loop);
                }
                last = now;
            }
        }
        return out;
    }

    private static List<String> names(List<String> requests) {
        return requests.stream().map(r -> r.substring(0, r.indexOf('@'))).toList();
    }

    private static int loopOf(String request) {
        return Integer.parseInt(request.substring(request.indexOf('@') + 1));
    }

    /** One loop per state, no settle time: what the buttons ask for. */
    private static List<String> instant(String... states) {
        Group g = fm(0);
        Hold[] holds = new Hold[states.length];
        for (int i = 0; i < states.length; i++) {
            holds[i] = hold(states[i], 1);
        }
        return names(requests(g, holds));
    }

    private static String held(String buttons) {
        Group g = fm(0);
        g.pad.hold(buttons);
        return g.active();
    }

    // What the buttons ask for.

    @Test
    void lbPressedWhileHoldingXIsTheLeftTrenchShot() {
        assertEquals(
                List.of("TRACK_TARGET", "SET_SHOT_LEFT_TRENCH", "TRACK_TARGET", "IDLE"),
                instant("X", "LB X", "X", ""));
    }

    @Test
    void lbPressedWhileHoldingAIsTheTowerShot() {
        assertEquals(
                List.of("UNJAM", "SET_SHOT_TOWER", "UNJAM", "IDLE"), instant("A", "LB A", "A", ""));
    }

    @Test
    void chordReleasedFaceButtonFirst() {
        assertEquals(List.of("SET_SHOT_LEFT_TRENCH", "IDLE"), instant("LB", "LB X", "LB", ""));
        assertEquals(List.of("SET_SHOT_RIGHT_TRENCH", "IDLE"), instant("LB B", "B", ""));
    }

    @Test
    void chordReleasedTogether() {
        assertEquals(List.of("SET_SHOT_TOWER", "IDLE"), instant("LB A", ""));
    }

    @Test
    void priorities() {
        assertEquals("SET_SHOT_TOWER", held("LB A X B"));
        assertEquals("SET_SHOT_LEFT_TRENCH", held("LB X B"));
        assertEquals("SET_SHOT_RIGHT_TRENCH", held("LB B"));
        assertEquals("UNJAM", held("A X"));
        assertEquals("TRACK_TARGET", held("X B"));
        assertEquals("IDLE", held("B")); // bare B does nothing
        assertEquals("IDLE", held("LB")); // bare LB is not a face chord
    }

    @Test
    void setShotsAreTeleopOnly() {
        Group g = fm(0);
        g.pad.teleop = false;
        g.pad.hold("LB X");
        assertEquals("IDLE", g.active()); // LB still keeps X from tracking
        g.pad.now += LOOP;
        g.pad.hold("X");
        assertEquals("TRACK_TARGET", g.active());
        g.pad.now += LOOP;
        g.pad.hold("A");
        assertEquals("UNJAM", g.active());
    }

    // With the default settle time.

    @Test
    void aLetGoTwoLoopsAfterLbDoesNotUnjam() {
        var r = requests(fm(SETTLE), hold("LB A", 50), hold("A", 2), hold("", 20));
        assertEquals(List.of("SET_SHOT_TOWER", "IDLE"), names(r));
        assertEquals("IDLE@50", r.get(1)); // stops the moment LB is let go
    }

    @Test
    void xLetGoTwoLoopsAfterLbDoesNotTrack() {
        assertEquals(
                List.of("SET_SHOT_LEFT_TRENCH", "IDLE"),
                names(requests(fm(SETTLE), hold("LB X", 50), hold("X", 2), hold("", 20))));
    }

    @Test
    void aPressedTwoLoopsBeforeLbDoesNotUnjam() {
        assertEquals(
                List.of("SET_SHOT_TOWER", "IDLE"),
                names(requests(fm(SETTLE), hold("A", 2), hold("LB A", 50), hold("", 5))));
    }

    @Test
    void aDeliberateUnjamStartsAfterTheSettleTime() {
        var r = requests(fm(SETTLE), hold("A", 50), hold("", 5));
        assertEquals(List.of("UNJAM", "IDLE"), names(r));
        double seconds = loopOf(r.get(0)) * LOOP;
        assertTrue(
                seconds >= SETTLE - 1e-9 && seconds <= SETTLE + LOOP,
                "unjam started at " + seconds + " s");
    }

    @Test
    void aTapPlaysLateForAsLongAsItWasHeld() {
        var r = requests(fm(SETTLE), hold("A", 5), hold("", 20));
        int start = (int) Math.round(SETTLE / LOOP);
        assertEquals(List.of("UNJAM@" + start, "IDLE@" + (start + 5)), r);
    }

    @Test
    void aChordBeforeATapPlaysCancelsIt() {
        assertEquals(
                List.of("SET_SHOT_LEFT_TRENCH", "IDLE"),
                names(
                        requests(
                                fm(SETTLE),
                                hold("A", 3),
                                hold("", 2),
                                hold("LB X", 20),
                                hold("", 5))));
    }

    @Test
    void aTapWhileTrackingPlaysThenTrackingResumes() {
        assertEquals(
                List.of("TRACK_TARGET", "UNJAM", "TRACK_TARGET", "IDLE"),
                names(
                        requests(
                                fm(SETTLE),
                                hold("X", 30),
                                hold("A X", 4),
                                hold("X", 30),
                                hold("", 5))));
    }

    @Test
    void aLetGoJustAfterLbIsNotATap() {
        assertEquals(
                List.of("SET_SHOT_TOWER", "IDLE"),
                names(requests(fm(SETTLE), hold("LB A", 30), hold("A", 5), hold("", 20))));
    }

    @Test
    void aModifierArrivingDuringAPressIsNotATapEvenWithNoChord() {
        Group g = fm(SETTLE);
        g.pad.teleop = false; // set shots off, so LB + A is no chord
        assertEquals(List.of(), requests(g, hold("A", 3), hold("LB A", 10), hold("", 20)));
    }

    @Test
    void holdingXThenPressingAKeepsTrackingUntilUnjamTakesOver() {
        assertEquals(
                List.of("TRACK_TARGET", "UNJAM", "IDLE"),
                names(requests(fm(SETTLE), hold("X", 30), hold("A X", 30), hold("", 5))));
    }

    @Test
    void lettingGoOfAWithXHeldTracksAtOnce() {
        List<String> r = requests(fm(SETTLE), hold("X", 30), hold("A X", 30), hold("X", 5));
        assertEquals("TRACK_TARGET@60", r.get(r.size() - 1));
    }

    @Test
    void lettingGoOfLbWithXHeldIsIdleThenTracking() {
        assertEquals(
                List.of("SET_SHOT_LEFT_TRENCH", "IDLE", "TRACK_TARGET", "IDLE"),
                names(requests(fm(SETTLE), hold("LB X", 30), hold("X", 30), hold("", 5))));
    }

    @Test
    void chordsActImmediately() {
        var r = requests(fm(SETTLE), hold("LB B", 10), hold("", 5));
        assertEquals("SET_SHOT_RIGHT_TRENCH@0", r.get(0));
    }

    // Other modifiers.

    /** RB and LT as modifiers for A, and A bare, as another robot might bind them. */
    private static Group otherModifiers() {
        return new Group(
                SETTLE,
                (g, p) -> {
                    var c = g.chords;
                    g.triggers.put("RB_A", c.chord(() -> p.rb, () -> p.a));
                    g.triggers.put("LT_A", c.chord(() -> p.lt, () -> p.a));
                    g.triggers.put("A", c.bare(() -> p.a));
                });
    }

    @Test
    void eachModifierHasItsOwnChord() {
        assertEquals(
                List.of("RB_A", "IDLE", "LT_A", "IDLE"),
                names(
                        requests(
                                otherModifiers(),
                                hold("RB A", 10),
                                hold("", 5),
                                hold("LT A", 10),
                                hold("", 5))));
    }

    @Test
    void theFirstChordMadeWinsWhenTwoModifiersAreHeld() {
        assertEquals(
                List.of("RB_A", "IDLE"),
                names(requests(otherModifiers(), hold("RB LT A", 10), hold("", 5))));
    }

    @Test
    void swappingModifiersMidPressGoesStraightToTheOtherChord() {
        assertEquals(
                List.of("RB_A", "LT_A", "IDLE"),
                names(requests(otherModifiers(), hold("RB A", 10), hold("LT A", 10), hold("", 5))));
    }

    @Test
    void onlyTheGroupsModifiersKeepTheBareButtonOff() {
        // LB is not one of this group's modifiers, so A with LB is still bare A.
        assertEquals(
                List.of("A", "IDLE"),
                names(requests(otherModifiers(), hold("LB A", 30), hold("", 5))));
        assertEquals(
                List.of("LT_A", "IDLE"),
                names(requests(otherModifiers(), hold("LT A", 30), hold("A", 5), hold("", 5))));
    }

    @Test
    void noSettleTimeActsAtOnce() {
        Group g = fm(0);
        assertEquals("UNJAM@0", requests(g, hold("A", 3)).get(0));
    }
}
