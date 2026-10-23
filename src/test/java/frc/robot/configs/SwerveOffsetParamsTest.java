package frc.robot.configs;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.SwerveConfig;
import frc.robot.subsystems.swerve.SwerveOffsetParams;
import frc.spectrumLib.params.Params;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Stream;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;
import org.wpilib.units.Units;

/**
 * The swerve offsets in {@code params/robots/<robot>/swerve.json} are each robot config's offsets,
 * bit for bit, so the params files change nothing until someone edits one on purpose.
 */
class SwerveOffsetParamsTest {
    private static final Path ROBOTS = Path.of("src/main/deploy/params/robots");
    private static final ObjectMapper JSON = new ObjectMapper();

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
    void deployFileHoldsTheConfigOffsets(String robot, Robot.Config config) throws IOException {
        double[] code = config.swerve.getEncoderOffsetRotations();
        assertNotNull(code, robot + " sets no swerve encoder offsets");
        Path file = ROBOTS.resolve(robot).resolve(SwerveOffsetParams.GROUP + ".json");
        assertTrue(Files.exists(file), file + " is missing");
        JsonNode values = JSON.readTree(file.toFile());
        List<String> keys = new ArrayList<>();
        values.fieldNames().forEachRemaining(keys::add);
        assertEquals(SwerveOffsetParams.KEYS, keys, file.toString());
        for (int i = 0; i < code.length; i++) {
            String key = SwerveOffsetParams.KEYS.get(i);
            JsonNode node = values.get(key);
            assertTrue(node.isNumber(), file + ": " + key);
            assertEquals(0, Double.compare(code[i], node.doubleValue()), file + ": " + key);
            assertTrue(
                    Math.abs(code[i]) <= SwerveOffsetParams.MAX_ABS_ROTATIONS,
                    robot + " " + key + " " + code[i] + " is outside the param's bounds");
        }
    }

    @Test
    void everyRobotFolderIsARobotConfig() throws Exception {
        List<Object> names = ConfigInvariantsTest.robots().map(a -> a.get()[0]).toList();
        try (Stream<Path> dirs = Files.list(ROBOTS)) {
            for (Path dir : dirs.toList()) {
                assertTrue(
                        names.contains(dir.getFileName().toString()),
                        dir + " names no robot config, so nothing ever reads it");
            }
        }
    }

    @Test
    void offsetsAreNeverSharedByEveryRobot() throws IOException {
        Path shared = ROBOTS.getParent().resolve(SwerveOffsetParams.GROUP + ".json");
        if (Files.exists(shared)) {
            JsonNode values = JSON.readTree(shared.toFile());
            for (String key : SwerveOffsetParams.KEYS) {
                assertFalse(values.has(key), shared + " would give every robot FM's " + key);
            }
        }
    }

    @Test
    void aFileValueReplacesTheOffsetAndAnOutOfBoundsOneIsRefused() throws IOException {
        SwerveConfig swerve = new FM2026().swerve;
        double[] code = swerve.getEncoderOffsetRotations();
        Path file = tmp.resolve("robots/FM2026/swerve.json");
        Files.createDirectories(file.getParent());
        Files.writeString(
                file, "{\"frontLeftEncoderOffset\": 0.125, \"frontRightEncoderOffset\": 24.902}");

        Params.start(tmp, "FM2026", null, null);
        SwerveOffsetParams.apply(swerve);

        assertArrayEquals(
                new double[] {0.125, code[1], code[2], code[3]},
                swerve.getEncoderOffsetRotations());
        assertEquals(0.125, swerve.getFrontLeftEncoderOffset().in(Units.Rotations));
        assertEquals(0.125, swerve.getFrontLeft().EncoderOffset);
        assertFalse(Params.checkFile(file).isEmpty(), "24.902 is out of bounds");

        SwerveOffsetParams.periodic();
        assertFalse(SwerveOffsetParams.changedSinceBoot());
        Files.writeString(file, "{\"frontLeftEncoderOffset\": 0.25}");
        Params.start(tmp, "FM2026", null, null);
        SwerveOffsetParams.periodic();
        assertTrue(SwerveOffsetParams.changedSinceBoot(), "a change after boot needs a reboot");
        assertEquals(0.125, swerve.getFrontLeft().EncoderOffset, "and is not applied before it");
    }

    @Test
    void noFileLeavesTheConfigUntouched() {
        SwerveConfig swerve = new FM2026().swerve;
        var before = swerve.getFrontLeft();
        double[] code = swerve.getEncoderOffsetRotations();

        Params.start(tmp, "FM2026", null, null);
        SwerveOffsetParams.apply(swerve);

        assertArrayEquals(code, swerve.getEncoderOffsetRotations());
        assertTrue(before == swerve.getFrontLeft(), "the module constants were not rebuilt");
    }
}
