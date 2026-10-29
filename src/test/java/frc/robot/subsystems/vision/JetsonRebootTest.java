package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import org.junit.jupiter.api.Test;
import org.wpilib.hardware.hal.HAL;

class JetsonRebootTest {
    private String bootId = "";

    private final Jetson jetson =
            new Jetson(inputs -> inputs.bootId = bootId, List.of(), new int[0], () -> true);

    private void cycle(String id) {
        bootId = id;
        jetson.updateInputs();
        jetson.periodic(true);
    }

    @Test
    void aNewBootIdIsAReboot() {
        HAL.initialize(500, 0);
        cycle("");
        cycle("a");
        // A reconnect drops the value to empty, then it comes back unchanged: not a reboot.
        cycle("");
        cycle("a");
        assertFalse(jetson.rebooted());
        cycle("b");
        assertTrue(jetson.rebooted());
    }
}
