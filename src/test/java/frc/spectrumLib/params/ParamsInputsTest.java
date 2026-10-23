package frc.spectrumLib.params;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.LogTable;

class ParamsInputsTest {
    @Test
    void replayRestoresTheLoggedValuesAndSources() {
        var robot = new ParamStore();
        var g = new ParamGroup("g", robot::register);
        DoubleParam n = g.number("n", 1, "");
        IntParam i = g.integer("i", 1, "");
        BooleanParam b = g.bool("b", true);
        StringParam s = g.string("s", "x");
        robot.restore(n, 2.5, ParamSource.LOCAL);
        robot.restore(i, 7, ParamSource.LIVE);
        robot.restore(b, false, ParamSource.DEPLOY);
        robot.restore(s, "y", ParamSource.ROBOT);

        LogTable table = new LogTable(0);
        new ParamsInputs(robot).toLog(table);

        var replay = new ParamStore();
        var g2 = new ParamGroup("g", replay::register);
        DoubleParam n2 = g2.number("n", 1, "");
        IntParam i2 = g2.integer("i", 1, "");
        BooleanParam b2 = g2.bool("b", true);
        StringParam s2 = g2.string("s", "x");
        DoubleParam newer = g2.number("newer", 9, "");
        new ParamsInputs(replay).fromLog(table);

        assertEquals(2.5, n2.get());
        assertEquals(ParamSource.LOCAL, n2.source());
        assertEquals(7, i2.get());
        assertEquals(ParamSource.LIVE, i2.source());
        assertEquals(false, b2.get());
        assertEquals("y", s2.get());
        assertEquals(ParamSource.ROBOT, s2.source());
        assertEquals(9.0, newer.get(), "not in the log: keeps its own value");
        assertEquals(ParamSource.DEFAULT, newer.source());
    }
}
