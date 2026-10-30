package frc.robot.configs;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import frc.robot.Robot;
import frc.robot.subsystems.hood.Hood.HoodConfig;
import frc.spectrumLib.hardware.CanInvariants;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.params.DoubleParam;
import frc.spectrumLib.params.Params;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.HashSet;
import java.util.Set;
import java.util.stream.Stream;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.hardware.hal.HAL;

/**
 * The soft limits in {@code params/soft-limits.json} are the code's, bit for bit, and no value the
 * params' bounds accept can leave a home position outside the limits ({@link ConfigInvariantsTest}
 * checks the code values; this checks every value a file could set).
 */
class SoftLimitParamsTest {
    private static final Path FILE = Path.of("src/main/deploy/params/soft-limits.json");

    @TempDir Path tmp;

    @AfterEach
    void resetParams() {
        Params.start(null, null, null, null);
    }

    static Stream<Arguments> robots() throws Exception {
        return ConfigInvariantsTest.robots();
    }

    @ParameterizedTest(name = "{0}")
    @MethodSource("robots")
    void everySoftLimitComesFromParamsAndTheFileHoldsTheCodeValue(String robot, Robot.Config config)
            throws IOException {
        JsonNode file = new ObjectMapper().readTree(FILE.toFile());
        for (Mechanism.Config m : ConfigInvariantsTest.mechanisms(config)) {
            if (!CanInvariants.hasSoftLimits(m)) {
                continue;
            }
            var limits = m.getTalonConfig().SoftwareLimitSwitch;
            DoubleParam forward = m.getForwardSoftLimitParam();
            DoubleParam reverse = m.getReverseSoftLimitParam();
            assertNotNull(forward, m.getName() + " has soft limits that are not params");
            assertNotNull(reverse, m.getName() + " has soft limits that are not params");
            assertEquals(0, Double.compare(limits.ForwardSoftLimitThreshold, forward.getDefault()));
            assertEquals(0, Double.compare(limits.ReverseSoftLimitThreshold, reverse.getDefault()));
            for (DoubleParam p : new DoubleParam[] {forward, reverse}) {
                JsonNode node = file.get(p.key());
                assertNotNull(node, FILE + " has no " + p.key());
                assertTrue(node.isNumber(), FILE + ": " + p.key());
                assertEquals(
                        0,
                        Double.compare(p.getDefault(), node.doubleValue()),
                        FILE + ": " + p.key());
            }
        }
    }

    @ParameterizedTest(name = "{0}")
    @MethodSource("robots")
    void theBoundsKeepEveryHomeBetweenTheLimits(String robot, Robot.Config config) {
        for (Mechanism.Config m : ConfigInvariantsTest.mechanisms(config)) {
            DoubleParam forward = m.getForwardSoftLimitParam();
            DoubleParam reverse = m.getReverseSoftLimitParam();
            if (forward == null) {
                continue;
            }
            assertTrue(
                    reverse.max() < forward.min(),
                    m.getName() + ": the bounds let the limits cross");
            var home = ConfigInvariantsTest.HOME_ROTATIONS.get(m.getName());
            assertNotNull(home, m.getName());
            for (double h : home.apply(config)) {
                assertTrue(
                        reverse.max() <= h && h <= forward.min(),
                        m.getName()
                                + " home "
                                + h
                                + " can fall outside limits in ["
                                + reverse.min()
                                + ", "
                                + reverse.max()
                                + "] and ["
                                + forward.min()
                                + ", "
                                + forward.max()
                                + "]");
            }
        }
    }

    @Test
    void everyKeyInTheFileIsUsed() throws Exception {
        Set<String> used = new HashSet<>();
        for (var a : robots().toList()) {
            for (Mechanism.Config m : ConfigInvariantsTest.mechanisms((Robot.Config) a.get()[1])) {
                if (m.getForwardSoftLimitParam() != null) {
                    used.add(m.getForwardSoftLimitParam().key());
                    used.add(m.getReverseSoftLimitParam().key());
                }
            }
        }
        new ObjectMapper()
                .readTree(FILE.toFile())
                .fieldNames()
                .forEachRemaining(k -> assertTrue(used.contains(k), k + " limits no mechanism"));
    }

    @Test
    void aMechanismTakesTheParamsOnceWhenBuilt() throws IOException {
        assertTrue(HAL.initialize());
        Files.writeString(
                tmp.resolve("soft-limits.json"), "{\"hoodForward\": 0.1, \"hoodReverse\": 0.5}");
        Params.start(tmp, "FM2026", null, null);

        HoodConfig config = new HoodConfig();
        Mechanism hood = new Mechanism(config, false) {};
        try {
            var limits = config.getTalonConfig().SoftwareLimitSwitch;
            assertEquals(0.1, limits.ForwardSoftLimitThreshold);
            assertEquals(0.024, limits.ReverseSoftLimitThreshold, "0.5 is out of bounds");
            assertTrue(limits.ForwardSoftLimitEnable && limits.ReverseSoftLimitEnable);
            assertFalse(hood.softLimitsNeedReboot());

            Files.writeString(tmp.resolve("soft-limits.json"), "{\"hoodForward\": 0.12}");
            Params.start(tmp, "FM2026", null, null);
            assertTrue(hood.softLimitsNeedReboot(), "a change after boot needs a reboot");
            assertEquals(0.1, limits.ForwardSoftLimitThreshold, "and is not applied before it");
        } finally {
            CommandScheduler.getInstance().unregisterSubsystem(hood);
        }
    }
}
