package frc.robot.subsystems.launcher;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import frc.rebuilt.ShotCalculator;
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

/** The Launcher subsystem. Four-motor flywheel that launches fuel at the hub. */
public class Launcher extends Mechanism {

    public static class LauncherConfig extends Config {

        /* Launcher config values */
        @Getter private final double supplyCurrentLimit = 80;
        @Getter private final double statorCurrentLimit = 180;
        @Getter private final double lowerSupplyCurrentLimit = 80;
        @Getter private final double lowerSupplyCurrentTime = 0;
        @Getter private final double forwardTorqueCurrentLimit = statorCurrentLimit;
        @Getter private final double reverseTorqueCurrentLimit = 10;
        @Getter private final double voltageLimit = 12;
        @Getter private final double velocityKp = 10;
        @Getter private final double velocityKv = 0;
        @Getter private final double velocityKs = 20;

        /**
         * Flywheel error inside which a volley may start (the shot gate, {@code ShotGate}). Was 100
         * and unused in 2026. Velocity TorqueCurrentFOC with kS 20 A and kP 10 A/rps settles 2 rps
         * (120 RPM) off target wherever friction does not cancel kS; the sim sits 124 RPM fast, so
         * 100 could never be met there. 9470 uses 150 for feed shots, the offseason bot 200.
         * CALIBRATE: tighten once {@code Shot/FlywheelErrorRPM} from the real robot shows its
         * steady-state error.
         */
        @Getter private final double onTargetToleranceRPM = 150;

        /* Sim Configs */
        @Getter private final double launcherX = Units.inchesToMeters(62.5);
        @Getter private final double launcherY = Units.inchesToMeters(60);
        @Getter private final double wheelDiameter = 4;

        public LauncherConfig() {
            super("Launcher", 46, Rio.CANIVORE);
            configPIDGains(0, velocityKp, 0, 0);
            configFeedForwardGains(velocityKs, velocityKv, 0, 0);
            configGearRatio(1);
            configLowerSupplyCurrentLimit(lowerSupplyCurrentLimit);
            configLowerSupplyCurrentTime(lowerSupplyCurrentTime);
            configSupplyCurrentLimit(supplyCurrentLimit, true);
            configStatorCurrentLimit(statorCurrentLimit, true);
            configForwardTorqueCurrentLimit(forwardTorqueCurrentLimit);
            configReverseTorqueCurrentLimit(reverseTorqueCurrentLimit);
            configNeutralBrakeMode(false);
            configForwardVoltageLimit(voltageLimit);
            configReverseVoltageLimit(-voltageLimit);
            configClockwise_Positive();
            setFollowerConfigs(
                    new FollowerConfig(
                            "Launcher Top Right", 47, Rio.CANIVORE, MotorAlignmentValue.Opposed),
                    new FollowerConfig(
                            "Launcher Bottom Left", 48, Rio.CANIVORE, MotorAlignmentValue.Aligned),
                    new FollowerConfig(
                            "Launcher Bottom Right",
                            49,
                            Rio.CANIVORE,
                            MotorAlignmentValue.Opposed));
        }
    }

    // ---- State Machine ----

    public enum WantedState {
        OFF,
        IDLE_PREP,
        SLOW_LAUNCH,
        AIM_AT_TARGET,
    }

    public enum SystemState {
        OFF,
        IDLE_PREP,
        SLOW_LAUNCH,
        AIM_AT_TARGET,
    }

    private WantedState wantedState = WantedState.OFF;
    private SystemState systemState = SystemState.OFF;

    public void setWantedState(WantedState state) {
        this.wantedState = state;
    }

    private SystemState handleStateTransition() {
        return switch (wantedState) {
            case OFF -> SystemState.OFF;
            case IDLE_PREP -> SystemState.IDLE_PREP;
            case SLOW_LAUNCH -> SystemState.SLOW_LAUNCH;
            case AIM_AT_TARGET -> SystemState.AIM_AT_TARGET;
        };
    }

    /**
     * Flywheel speed the current shot wants this loop, RPM; 0 when the flywheel is not on a shot.
     * Computed from logged inputs (the shot solution), never read back from the motor, so the shot
     * gate that compares it with {@link #getVelocityRPM()} replays.
     */
    @Getter private double shotTargetRPM = 0;

    private void applyStates() {
        double wantedRPM = 0;
        shotTargetRPM = 0;
        switch (systemState) {
            case OFF:
                stop();
                return;
            case IDLE_PREP:
                wantedRPM = 700;
                break;
            case SLOW_LAUNCH:
                wantedRPM = 400;
                break;
            case AIM_AT_TARGET:
                var params = ShotCalculator.getInstance().getParameters();
                wantedRPM = params.flywheelSpeed();
                shotTargetRPM = wantedRPM;
                break;
        }
        final double finalWantedRPM = wantedRPM;
        setVelocityTCFOCrpm(() -> finalWantedRPM);
    }

    @Getter private final LauncherConfig config;
    @Getter private LauncherSim sim;

    public Launcher(LauncherConfig config) {
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
        Telemetry.log("Launcher/WantedState", wantedState.toString());
        Telemetry.log("Launcher/SystemState", systemState.toString());
        Telemetry.log("Launcher/CurrentCommand", getCurrentCommandName());
        // Voltage, currents, temperature and connection at 10 Hz (offseason); the
        // same signals are also recorded every loop as inputs under Mechanisms/.
        logDiagnostics("Launcher");
        Telemetry.log("Launcher/RPM", getVelocityRPM(), "RPM");
    }

    // --------------------------------------------------------------------------------
    // Simulation
    // --------------------------------------------------------------------------------
    public void simulationInit() {
        if (isAttached()) {
            sim = new LauncherSim(RobotSim.leftView, motor);
        }
    }

    // Must be called to enable the simulation
    // if roller position changes configure x and y to set position.
    // Simulation steps itself on spectrumLib SimLoop (200 Hz) since the offseason port.

    class LauncherSim extends RollerSim {
        public LauncherSim(Mechanism2d mech, TalonFX rollerMotorSim) {
            super(
                    new RollerConfig(config.getWheelDiameter())
                            .setPosition(config.getLauncherX(), config.getLauncherY())
                            .setMount(Robot.getHood().getSim()),
                    mech,
                    rollerMotorSim,
                    config.getName());
        }
    }
}
