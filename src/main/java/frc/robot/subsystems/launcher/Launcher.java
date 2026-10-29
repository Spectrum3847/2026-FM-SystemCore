package frc.robot.subsystems.launcher;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import frc.rebuilt.ShotCalculator;
import frc.robot.Robot;
import frc.robot.RobotSim;
import frc.spectrumLib.hardware.CanBuses;
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
        // Spin-up is the largest draw; limited to keep the battery up while driving.
        @Getter private final double supplyCurrentLimit = 60;
        @Getter private final double statorCurrentLimit = 180;
        @Getter private final double lowerSupplyCurrentLimit = 60;
        @Getter private final double lowerSupplyCurrentTime = 0;
        @Getter private final double forwardTorqueCurrentLimit = statorCurrentLimit;
        @Getter private final double reverseTorqueCurrentLimit = 10;
        @Getter private final double voltageLimit = 12;
        @Getter private final double velocityKp = 10;
        @Getter private final double velocityKv = 0;
        @Getter private final double velocityKs = 20;

        /* Sim Configs */
        @Getter private final double launcherX = Units.inchesToMeters(62.5);
        @Getter private final double launcherY = Units.inchesToMeters(60);
        @Getter private final double wheelDiameter = 4;

        public LauncherConfig() {
            super("Launcher", 46, CanBuses.SHOOTER);
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
            // Off until turned on. No trip in FM's 20 TXMAN1 matches (longest spin-up 1.06 s);
            // in 3847's 28 logs, only during E11's CAN dropout.
            configVelocityTrackingStop(1000.0 / 60, 2.0, false);
            setFollowerConfigs(
                    new FollowerConfig(
                            "Launcher Top Right",
                            47,
                            CanBuses.SHOOTER,
                            MotorAlignmentValue.Opposed),
                    new FollowerConfig(
                            "Launcher Bottom Left",
                            48,
                            CanBuses.SHOOTER,
                            MotorAlignmentValue.Aligned),
                    new FollowerConfig(
                            "Launcher Bottom Right",
                            49,
                            CanBuses.SHOOTER,
                            MotorAlignmentValue.Opposed));
        }
    }

    // ---- State Machine ----

    public enum WantedState {
        OFF,
        IDLE_PREP,
        SLOW_LAUNCH,
        AIM_AT_TARGET,
        /** The selected set shot's speed, off the hub model at the spot's range. */
        SET_SHOT,
    }

    public enum SystemState {
        OFF,
        IDLE_PREP,
        SLOW_LAUNCH,
        AIM_AT_TARGET,
        SET_SHOT,
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
            case SET_SHOT -> SystemState.SET_SHOT;
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
            case SET_SHOT:
                wantedRPM = ShotCalculator.getSetShotFlywheelRPM();
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
        Telemetry.time("Scheduler/Periodic/Launcher");
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
        Telemetry.timeEnd("Scheduler/Periodic/Launcher");
    }

    // --------------------------------------------------------------------------------
    // Simulation
    // --------------------------------------------------------------------------------
    public void simulationInit() {
        if (isAttached()) {
            sim = new LauncherSim(RobotSim.leftView, motor);
        }
    }

    // Must be called to enable the simulation.
    // Set x and y when the roller position changes.
    // Simulation steps itself on spectrumLib SimLoop (200 Hz).

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
