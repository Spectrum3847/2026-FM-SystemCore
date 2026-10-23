package frc.spectrumLib.params;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.spectrumLib.telemetry.TuneValue;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

class TuningModeTest {
    @AfterEach
    void off() {
        TuningMode.set(false);
    }

    /** Records what was published; the dashboard always asks for 99. */
    private static final class RecordingTransport implements ParamTransport {
        final List<String> published = new ArrayList<>();

        @Override
        public LiveValue publish(Param<?> param) {
            published.add(param.path());
            return LiveValue.of(() -> 99.0, v -> {});
        }

        @Override
        public LiveValue button(String name) {
            published.add(name);
            return LiveValue.of(() -> false, v -> {});
        }
    }

    @Test
    void offByDefault() {
        assertFalse(TuningMode.enabled());
    }

    @Test
    void tuneValueHoldsItsDefaultWhenOff() {
        TuneValue v = new TuneValue("TuningModeTest/x", 3.0);
        assertEquals(3.0, v.update());
        assertEquals(3.0, v.getSupplier().getAsDouble());
    }

    @Test
    void paramsIgnoreTheTransportWhenOff() {
        DoubleParam p = Params.group("tuningModeTest").number("off", 1.0, "");
        var t = new RecordingTransport();
        Params.start(null, null, null, t);
        assertTrue(t.published.isEmpty(), "nothing published for tuning");
        assertFalse(Params.liveTuning());
        assertEquals(1.0, p.get());
    }
}
