package frc.robot.configs;

import frc.robot.Robot.Config;
import java.util.OptionalDouble;

public class FM2026 extends Config {

    // Final Machine
    public FM2026() {
        super();
        swerve.configEncoderOffsets(-0.163818359375, 0.24902, 0.2724609375, -0.31005859375);
        // The battery check's thresholds were set from this SystemCore's own readings.
        batteryVoltsOffset = OptionalDouble.of(0);

        // Attached Mechanisms
        pilot.setAttached(true);
        operator.setAttached(true);
        fuelIntake.setAttached(true);
        intakeExtension.setAttached(true);
        launcher.setAttached(true);
        indexerTower.setAttached(true);
        indexerBed.setAttached(true);
        hood.setAttached(true);
    }
}
