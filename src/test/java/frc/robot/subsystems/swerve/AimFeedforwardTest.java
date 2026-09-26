package frc.robot.subsystems.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class AimFeedforwardTest {
    @Test
    void passesSmallRatesThroughUnchangedWithTheirSign() {
        assertEquals(0.4, AimFeedforward.of(0.4), 1e-12);
        assertEquals(-0.4, AimFeedforward.of(-0.4), 1e-12);
    }

    @Test
    void clampsToTwoRadiansPerSecondBothWays() {
        assertEquals(AimFeedforward.MAX_RAD_PER_SEC, AimFeedforward.of(9.0), 1e-12);
        assertEquals(-AimFeedforward.MAX_RAD_PER_SEC, AimFeedforward.of(-9.0), 1e-12);
        assertEquals(2.0, AimFeedforward.MAX_RAD_PER_SEC, 0);
    }

    @Test
    void notANumberIsNoFeedforward() {
        assertEquals(0, AimFeedforward.of(Double.NaN), 0);
        assertEquals(0, AimFeedforward.of(Double.POSITIVE_INFINITY), 0);
    }
}
