package frc.robot.pilot;

import static frc.robot.pilot.FaceChord.NONE;
import static frc.robot.pilot.FaceChord.SET_SHOT_LEFT_TRENCH;
import static frc.robot.pilot.FaceChord.SET_SHOT_RIGHT_TRENCH;
import static frc.robot.pilot.FaceChord.SET_SHOT_TOWER;
import static frc.robot.pilot.FaceChord.TRACK_TARGET;
import static frc.robot.pilot.FaceChord.UNJAM;
import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

class FaceChordTest {
    private static FaceChord held(boolean lb, boolean a, boolean x, boolean b) {
        return FaceChord.of(lb, a, x, b, true);
    }

    /** Replays button states the way Robot binds them. Returns the requests in order. */
    private static List<String> requests(boolean[][] states) {
        List<String> out = new ArrayList<>();
        FaceChord last = NONE;
        for (boolean[] s : states) {
            FaceChord now = held(s[0], s[1], s[2], s[3]);
            if (now != last) {
                out.add(now == NONE ? "IDLE" : now.name());
            }
            last = now;
        }
        return out;
    }

    private static final boolean T = true;
    private static final boolean F = false;

    @Test
    void lbPressedWhileHoldingXIsTheLeftTrenchShot() {
        // {lb, a, x, b}
        assertEquals(
                List.of("TRACK_TARGET", "SET_SHOT_LEFT_TRENCH", "TRACK_TARGET", "IDLE"),
                requests(
                        new boolean[][] {
                            {F, F, T, F}, {T, F, T, F}, {F, F, T, F}, {F, F, F, F},
                        }));
    }

    @Test
    void lbPressedWhileHoldingAIsTheTowerShot() {
        assertEquals(
                List.of("UNJAM", "SET_SHOT_TOWER", "UNJAM", "IDLE"),
                requests(
                        new boolean[][] {
                            {F, T, F, F}, {T, T, F, F}, {F, T, F, F}, {F, F, F, F},
                        }));
    }

    @Test
    void chordReleasedFaceButtonFirst() {
        assertEquals(
                List.of("SET_SHOT_LEFT_TRENCH", "IDLE"),
                requests(
                        new boolean[][] {
                            {T, F, F, F}, {T, F, T, F}, {T, F, F, F}, {F, F, F, F},
                        }));
        assertEquals(
                List.of("SET_SHOT_RIGHT_TRENCH", "IDLE"),
                requests(new boolean[][] {{T, F, F, T}, {F, F, F, T}, {F, F, F, F}}));
    }

    @Test
    void chordReleasedTogether() {
        assertEquals(
                List.of("SET_SHOT_TOWER", "IDLE"),
                requests(new boolean[][] {{T, T, F, F}, {F, F, F, F}}));
    }

    @Test
    void priorities() {
        assertEquals(SET_SHOT_TOWER, held(T, T, T, T));
        assertEquals(SET_SHOT_LEFT_TRENCH, held(T, F, T, T));
        assertEquals(SET_SHOT_RIGHT_TRENCH, held(T, F, F, T));
        assertEquals(UNJAM, held(F, T, T, F));
        assertEquals(TRACK_TARGET, held(F, F, T, T));
        assertEquals(NONE, held(F, F, F, T)); // bare B does nothing
        assertEquals(NONE, held(T, F, F, F)); // bare LB is not a face chord
    }

    @Test
    void setShotsAreTeleopOnly() {
        assertEquals(NONE, FaceChord.of(T, F, T, F, false));
        assertEquals(TRACK_TARGET, FaceChord.of(F, F, T, F, false));
        assertEquals(UNJAM, FaceChord.of(F, T, F, F, false));
    }
}
