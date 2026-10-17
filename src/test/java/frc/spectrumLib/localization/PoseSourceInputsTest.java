package frc.spectrumLib.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.spectrumLib.localization.PoseObservation.Kind;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.LogTable;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;

class PoseSourceInputsTest {
    private static PoseObservation obs(boolean tracking) {
        return new PoseObservation(
                "S",
                Kind.QUESTNAV,
                1.25,
                new Pose3d(1.1, 2.2, 0.3, new Rotation3d(0.01, -0.02, 2.4465647846629746)),
                0,
                Double.NaN,
                Double.NaN,
                Double.NaN,
                tracking);
    }

    private static PoseSourceInputs roundTrip(PoseSourceInputs in) {
        LogTable table = new LogTable(0);
        in.toLog(table);
        PoseSourceInputs out = new PoseSourceInputs();
        out.fromLog(table);
        return out;
    }

    @Test
    void observationsSurviveTheLog() {
        for (boolean tracking : new boolean[] {true, false}) {
            PoseSourceInputs in = new PoseSourceInputs();
            in.resize(1);
            in.set(0, obs(tracking));
            PoseSourceInputs out = roundTrip(in);
            assertEquals(in.get("S", 0), out.get("S", 0));
        }
    }

    @Test
    void emptyLoopSurvivesTheLog() {
        PoseSourceInputs in = new PoseSourceInputs();
        in.resize(0);
        assertEquals(0, roundTrip(in).count());
    }
}
