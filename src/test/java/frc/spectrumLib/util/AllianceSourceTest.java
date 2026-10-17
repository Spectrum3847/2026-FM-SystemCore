package frc.spectrumLib.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.spectrumLib.util.AllianceSource.Override;
import frc.spectrumLib.util.AllianceSource.Resolution;
import frc.spectrumLib.util.AllianceSource.Source;
import java.util.Optional;
import org.junit.jupiter.api.Test;
import org.wpilib.driverstation.Alliance;

class AllianceSourceTest {
    private static final Optional<Alliance> NONE = Optional.empty();
    private static final Optional<Alliance> BLUE = Optional.of(Alliance.BLUE);
    private static final Optional<Alliance> RED = Optional.of(Alliance.RED);

    private static void check(Resolution r, Alliance alliance, Source source) {
        assertEquals(alliance, r.alliance());
        assertEquals(source, r.source());
    }

    @Test
    void defaultsToBlue() {
        check(AllianceSource.resolve(Override.AUTO, NONE, NONE), Alliance.BLUE, Source.DEFAULT);
    }

    @Test
    void followsTheDs() {
        check(AllianceSource.resolve(Override.AUTO, RED, NONE), Alliance.RED, Source.DS);
        check(AllianceSource.resolve(Override.AUTO, BLUE, RED), Alliance.BLUE, Source.DS);
    }

    @Test
    void remembersTheLastKnown() {
        check(AllianceSource.resolve(Override.AUTO, NONE, RED), Alliance.RED, Source.LAST_KNOWN);
    }

    @Test
    void overrideWins() {
        check(AllianceSource.resolve(Override.RED, BLUE, BLUE), Alliance.RED, Source.OVERRIDE);
        check(AllianceSource.resolve(Override.BLUE, RED, NONE), Alliance.BLUE, Source.OVERRIDE);
        check(AllianceSource.resolve(Override.RED, NONE, NONE), Alliance.RED, Source.OVERRIDE);
    }
}
