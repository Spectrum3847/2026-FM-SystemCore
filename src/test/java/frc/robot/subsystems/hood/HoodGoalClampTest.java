package frc.robot.subsystems.hood;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.subsystems.hood.Hood.HoodConfig;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.params.Params;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.hardware.hal.HAL;

/** Hood goals are clamped to the soft limits the hood booted with, not to the params as edited. */
class HoodGoalClampTest {
    @TempDir Path tmp;

    @AfterEach
    void resetParams() {
        Params.start(null, null, null, null);
    }

    @Test
    void goalsPastTheBootedSoftLimitsGoToTheLimit() throws IOException {
        assertTrue(HAL.initialize(500, 0));
        // A narrowed top limit, 25.2 deg, inside the params' bounds.
        Files.writeString(tmp.resolve("soft-limits.json"), "{\"hoodForward\": 0.07}");
        Params.start(tmp, "FM2026", null, null);

        Mechanism hood = new Mechanism(new HoodConfig(), false) {};
        try {
            assertEquals(0.07, hood.clampToSoftLimits(40.0 / 360), 1e-12, "past the top");
            assertEquals(0.024, hood.clampToSoftLimits(5.0 / 360), 1e-12, "under the bottom");
            assertEquals(20.0 / 360, hood.clampToSoftLimits(20.0 / 360), 1e-12, "inside");

            // A params edit after boot needs a reboot to reach the motor, so the clamp keeps
            // the limit the motor has.
            Files.writeString(tmp.resolve("soft-limits.json"), "{\"hoodForward\": 0.1}");
            Params.start(tmp, "FM2026", null, null);
            assertEquals(0.07, hood.clampToSoftLimits(40.0 / 360), 1e-12);
        } finally {
            CommandScheduler.getInstance().unregisterSubsystem(hood);
        }
    }
}
