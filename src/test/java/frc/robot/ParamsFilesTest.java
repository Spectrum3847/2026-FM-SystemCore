package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.robot.configs.FM2026;
import frc.robot.subsystems.swerve.SwerveOffsetParams;
import frc.spectrumLib.params.Params;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Stream;
import org.junit.jupiter.api.Test;

/** Every params file in git parses, and every key in it is a declared param with a valid value. */
class ParamsFilesTest {
    @Test
    void deployParamsFilesMatchTheDeclarations() throws IOException {
        RobotParams.load();
        SwerveOffsetParams.declare(new FM2026().swerve);
        List<String> problems = new ArrayList<>();
        try (Stream<Path> files = Files.walk(Path.of("src/main/deploy/params"))) {
            files.filter(f -> f.toString().endsWith(".json"))
                    .forEach(f -> problems.addAll(Params.checkFile(f)));
        }
        assertEquals(List.of(), problems);
    }
}
