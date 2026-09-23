package frc.robot.subsystems.fuelIntake;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import frc.robot.Robot;
import frc.robot.RobotSim;
import frc.spectrumLib.hardware.Rio;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.sim.RollerConfig;
import frc.spectrumLib.sim.RollerSim;
import frc.spectrumLib.telemetry.Telemetry;
import lombok.Getter;
import org.wpilib.math.util.Units;
import org.wpilib.smartdashboard.Mechanism2d;

/** The Fuel Intake subsystem. Responsible for intake and handling of fuel elements. */
public class FuelIntake extends Mechanism {

    public static class FuelIntakeConfig extends Config {

        /* Intake config values */
        @Getter private final double supplyCurrentLimit = 45;
        @Getter private final double statorCurrentLimit = 90;
        @Getter private final double lowerSupplyCurrentLimit = 45;
        @Getter private final double lowerSupplyCurrentTime = 1;
        @Getter private final double voltageLimit = 12;
        @Getter private final double velocityKp = 5;
        @Getter private final double velocityKv = 0;
        @Getter private final double velocityKs = 4;

        /* Sim Configs */
        @Getter private final double intakeX = Units.inchesToMeters(15);
        @Getter private final double intakeY = Units.inchesToMeters(23);
        @Getter private final double wheelDiameter = 6;

        public FuelIntakeConfig() {
            super("Intake", 5, Rio.RIO_CANBUS);
            configPIDGains(0, velocityKp, 0, 0);
            configFeedForwardGains(velocityKs, velocityKv, 0, 0);
            configGearRatio(1);
            configSupplyCurrentLimit(supplyCurrentLimit, true);
            configStatorCurrentLimit(statorCurrentLimit, true);
            configLowerSupplyCurrentLimit(lowerSupplyCurrentLimit);
            configLowerSupplyCurrentTime(lowerSupplyCurrentTime);
            configForwardTorqueCurrentLimit(statorCurrentLimit);
            configReverseTorqueCurrentLimit(statorCurrentLimit);
            configForwardVoltageLimit(voltageLimit);
            configReverseVoltageLimit(-voltageLimit);
            configNeutralBrakeMode(false);
            configCounterClockwise_Positive();
            setFollowerConfigs(
                    new FollowerConfig(
                            "Intake Right", 6, Rio.RIO_CANBUS, MotorAlignmentValue.Opposed));
        }
    }

    // ---- State Machine ----

    public enum WantedState {
        NEUTRAL,
        OFF,
        INTAKE,
        OUTTAKE,
        SLOW_INTAKE,
    }

    public enum SystemState {
        NEUTRAL,
        OFF,
        INTAKE,
        OUTTAKE,
        SLOW_INTAKE,
    }

    private WantedState wantedState = WantedState.NEUTRAL;
    private SystemState systemState = SystemState.NEUTRAL;

    public void setWantedState(WantedState state) {
        this.wantedState = state;
    }

    private SystemState handleStateTransition() {
        return switch (wantedState) {
            case NEUTRAL -> SystemState.NEUTRAL;
            case INTAKE -> SystemState.INTAKE;
            case OUTTAKE -> SystemState.OUTTAKE;
            case SLOW_INTAKE -> SystemState.SLOW_INTAKE;
            case OFF -> SystemState.OFF;
        };
    }

    private void applyStates() {
        double wantedVoltage = 0;
        switch (systemState) {
            case NEUTRAL:
                wantedVoltage = 0;
                break;
            case INTAKE:
                wantedVoltage = 12;
                break;
            case OUTTAKE:
                wantedVoltage = -12;
                break;
            case SLOW_INTAKE:
                wantedVoltage = 5;
                break;
            case OFF:
                stop();
                return;
        }
        final double finalWantedVoltage = wantedVoltage;
        setVoltageOutput(() -> finalWantedVoltage);
    }

    @Getter private final FuelIntakeConfig config;
    @Getter private FuelIntakeSim sim;

    public FuelIntake(FuelIntakeConfig config) {
        super(config);
        this.config = config;

        simulationInit();
        Telemetry.print(getName() + " Subsystem Initialized");
    }

    @Override
    public void periodic() {
        systemState = handleStateTransition();
        applyStates();
        logBatteryUsage();
        Telemetry.log("FuelIntake/WantedState", wantedState.toString());
        Telemetry.log("FuelIntake/SystemState", systemState.toString());
        Telemetry.log("FuelIntake/CurrentCommand", getCurrentCommandName());
        // Voltage, currents, temperature and connection at 10 Hz (offseason); the
        // same signals are also recorded every loop as inputs under Mechanisms/.
        logDiagnostics("FuelIntake");
        Telemetry.log("FuelIntake/RPM", getVelocityRPM(), "RPM");
    }

    // --------------------------------------------------------------------------------
    // Simulation
    // --------------------------------------------------------------------------------
    public void simulationInit() {
        if (isAttached()) {
            // Create a new RollerSim with the left view, the motor's sim state, and a 6 in diameter
            sim = new FuelIntakeSim(RobotSim.leftView, motor);
        }
    }

    // Must be called to enable the simulation
    // if roller position changes configure x and y to set position.
    // Simulation steps itself on spectrumLib SimLoop (200 Hz) since the offseason port.

    class FuelIntakeSim extends RollerSim {
        public FuelIntakeSim(Mechanism2d mech, TalonFX rollerMotorSim) {
            super(
                    new RollerConfig(config.getWheelDiameter())
                            .setPosition(config.getIntakeX(), config.getIntakeY())
                            .setMount(Robot.getIntakeExtension().getSim()),
                    mech,
                    rollerMotorSim,
                    config.getName());
        }
    }
}
