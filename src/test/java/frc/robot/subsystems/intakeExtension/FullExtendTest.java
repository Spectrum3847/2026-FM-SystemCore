package frc.robot.subsystems.intakeExtension;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;

import frc.robot.subsystems.intakeExtension.FullExtend.Action;
import frc.robot.subsystems.intakeExtension.FullExtend.Phase;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class FullExtendTest {
    private static final double LOOP_MS = 10;

    private FullExtend extend;
    private double now;
    private boolean entered;

    @BeforeEach
    void setUp() {
        extend = new FullExtend(new FullExtend.Settings(2.8, 0.08, 0.3, 18, 0.67, 0.25, 0.33));
        now = 100_000;
        entered = true;
    }

    private Action step(boolean connected, double left, double right, double rpm) {
        Action action =
                extend.update(
                        new FullExtend.Inputs(
                                entered, true, connected, now, left, right, rpm, rpm));
        entered = false;
        now += LOOP_MS;
        return action;
    }

    /** Runs {@code seconds} of loops and returns how many asked for a relearn. */
    private int run(double seconds, boolean connected, double left, double right, double rpm) {
        int relearns = 0;
        for (int i = 0; i < Math.round(seconds * 1000 / LOOP_MS); i++) {
            if (step(connected, left, right, rpm) == Action.RELEARN) {
                relearns++;
            }
        }
        return relearns;
    }

    @Test
    void q34DropWhileHoldingKeepsHolding() {
        run(0.5, true, 2.78, 2.82, 0);
        assertEquals(Phase.HOLDING, extend.phase());

        // TXMAN1 Q34 t=608.84: Connected goes false on stale readings, the readings drop to 0 for
        // 0.5 s, then come back before Connected does.
        assertEquals(0, run(0.14, false, 2.78, 2.82, 0));
        for (int i = 0; i < 50; i++) {
            assertEquals(Action.HOLD, step(false, 0, 0, 0));
        }
        assertEquals(0, run(0.1, false, 2.787, 2.827, 0));
        assertEquals(0, run(1.0, true, 2.787, 2.827, 0));

        assertEquals(Phase.HOLDING, extend.phase());
        assertEquals(0, extend.relearns());
    }

    @Test
    void dropDuringFirstExtendDoesNotRelearn() {
        run(0.2, true, 0, 0, 300);
        run(0.2, true, 1.0, 1.0, 300);

        assertEquals(0, run(0.6, false, 0, 0, 0));
        assertEquals(Phase.DRIVING, extend.phase());

        run(0.3, true, 2.0, 2.0, 300);
        assertEquals(0, run(0.5, true, 2.79, 2.8, 0));
        assertEquals(Phase.HOLDING, extend.phase());
        assertEquals(0, extend.relearns());
    }

    @Test
    void poweredOnOutStillRelearns() {
        assertEquals(1, run(0.31, true, 0.02, 0.03, 0));
        assertEquals(0, run(0.5, true, 2.8, 2.8, 0));
        assertEquals(Phase.HOLDING, extend.phase());
    }

    @Test
    void pushedInWhileDisconnectedReextendsOnReconnect() {
        run(0.5, true, 2.78, 2.82, 0);
        run(0.5, false, 0, 0, 0);
        assertNotEquals(Action.HOLD, step(true, 1.9, 1.9, 0));
        assertEquals(Phase.DRIVING, extend.phase());
    }
}
