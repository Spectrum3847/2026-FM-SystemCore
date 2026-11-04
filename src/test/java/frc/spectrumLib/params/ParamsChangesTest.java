package frc.spectrumLib.params;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.HashMap;
import java.util.List;
import org.junit.jupiter.api.Test;

class ParamsChangesTest {
    @Test
    void eachChangedValueIsOneLineWithItsSource() {
        var store = new ParamStore();
        var g = new ParamGroup("Shooter", store::register);
        DoubleParam rpm = g.number("rpm", 3000, "");
        g.bool("enabled", true);
        var last = new HashMap<String, Object>();

        assertEquals(List.of(), Params.changes(store, last), "first values are not changes");
        store.restore(rpm, 3200.0, ParamSource.LIVE);
        assertEquals(
                List.of("param " + rpm.path() + ": 3000.0 -> 3200.0 (LIVE)"),
                Params.changes(store, last));
        assertEquals(List.of(), Params.changes(store, last), "unchanged since: nothing");
    }
}
