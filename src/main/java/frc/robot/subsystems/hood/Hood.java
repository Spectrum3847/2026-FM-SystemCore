package frc.robot.subsystems.hood;

import com.ctre.phoenix6.hardware.TalonFX;
import frc.rebuilt.ShotCalculator;
import frc.robot.Constants;
import frc.robot.RobotSim;
import frc.spectrumLib.checks.HomingCheck;
import frc.spectrumLib.hardware.CanBuses;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.params.DoubleParam;
import frc.spectrumLib.params.ParamGroup;
import frc.spectrumLib.params.Params;
import frc.spectrumLib.sim.ArmConfig;
import frc.spectrumLib.sim.ArmSim;
import frc.spectrumLib.telemetry.Alert;
import frc.spectrumLib.telemetry.Telemetry;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;
import org.wpilib.driverstation.RobotState;
import org.wpilib.math.util.Units;
import org.wpilib.smartdashboard.Mechanism2d;
import org.wpilib.system.Timer;
import org.wpilib.util.Alert.Level;

/** The Hood subsystem. Positions the hood that sets the fuel launch angle. */
public class Hood extends Mechanism {

    public static class HoodConfig extends Config {

        /** Best guess at boot, until homing finds the stop. */
        @Getter private final double initPosition = 9;

        private static final double MAX_ROTATIONS = 0.137;
        private static final double MIN_ROTATIONS = 0.024;

        @Getter private final double maxRotations = MAX_ROTATIONS;
        @Getter private final double minRotations = MIN_ROTATIONS;

        private static final ParamGroup SOFT_LIMITS = Params.group("soft-limits");

        /**
         * 49.3 deg. No higher than that: where the top stop is is not recorded in this code, so
         * raising the limit is a code change. No lower than 20 deg, well above the bottom stop, so
         * the two limits can never cross.
         */
        public static final DoubleParam FORWARD_SOFT_LIMIT =
                SOFT_LIMITS.number(
                        "hoodForward",
                        MAX_ROTATIONS,
                        "rot",
                        Units.degreesToRotations(20),
                        MAX_ROTATIONS);

        /**
         * 8.64 deg, 0.36 deg under the 9 deg bottom stop on purpose. At most the stop itself, where
         * the hood boots and homes (initPosition, HoodHoming stopDeg): above it the hood would sit
         * outside its own limits. At least 1 deg under the stop.
         */
        public static final DoubleParam REVERSE_SOFT_LIMIT =
                SOFT_LIMITS.number(
                        "hoodReverse",
                        MIN_ROTATIONS,
                        "rot",
                        Units.degreesToRotations(8),
                        Units.degreesToRotations(9));

        /* Hood config values */
        @Getter private final double supplyCurrentLimit = 40;
        @Getter private final double statorCurrentLimit = 60;
        @Getter private final double lowerSupplyCurrentLimit = 40;
        @Getter private final double lowerSupplyCurrentTime = 1;
        @Getter private final double positionKp = 3000;
        @Getter private final double positionKi = 0;
        @Getter private final double positionKd = 220;
        @Getter private final double positionKv = 0;
        @Getter private final double positionKs = 25;
        @Getter private final double positionKa = 0;
        @Getter private final double positionKg = 0;

        @Getter private final double gearRatio = 51.667;
        @Getter private final double mmCruiseVelocity = 50;
        @Getter private final double mmAcceleration = 200;
        @Getter private final double mmJerk = 1000;
        @Getter private final double holdMaxSpeedRPM = 18;

        /* Sim Configs */
        @Getter private final double hoodX = Units.inchesToMeters(62.5);
        @Getter private final double hoodY = Units.inchesToMeters(50);
        @Getter private final double simRatio = 51.667;
        @Getter private final double length = Units.inchesToMeters(10);

        public HoodConfig() {
            super("Hood", 15, CanBuses.SHOOTER);
            configMinMaxRotations(minRotations, maxRotations);
            configPIDGains(0, positionKp, positionKi, positionKd);
            configFeedForwardGains(positionKs, positionKv, positionKa, positionKg);
            configMotionMagic(mmCruiseVelocity, mmAcceleration, mmJerk);
            configGearRatio(gearRatio);
            configSupplyCurrentLimit(supplyCurrentLimit, true);
            configStatorCurrentLimit(statorCurrentLimit, true);
            configLowerSupplyCurrentLimit(lowerSupplyCurrentLimit);
            configLowerSupplyCurrentTime(lowerSupplyCurrentTime);
            configForwardTorqueCurrentLimit(statorCurrentLimit);
            configReverseTorqueCurrentLimit(statorCurrentLimit);
            configSoftLimits(FORWARD_SOFT_LIMIT, REVERSE_SOFT_LIMIT);
            configNeutralBrakeMode(true);
            configClockwise_Positive();
            // Off until turned on. Against FM's 20 TXMAN1 matches it trips only in Q06, where the
            // hood stuck above home pushing up to 47 A for 4 and 13 s; the longest normal
            // excursion is 1.0 s. 1 deg would also trip on the pre-homing hood resting 1 deg under
            // its home goal (8 matches).
            configPositionTrackingStop(Units.degreesToRotations(2), 2.0, false);
        }
    }

    // ---- State Machine ----

    public enum WantedState {
        HOME,
        STOPPED,
        AIM_AT_TARGET,
        /** The selected set shot's angle, off the hub model at the spot's range. */
        SET_SHOT,
    }

    public enum SystemState {
        HOME,
        STOPPED,
        AIM_AT_TARGET,
        SET_SHOT,
    }

    @Getter private WantedState wantedState = WantedState.HOME;
    private SystemState systemState = SystemState.HOME;

    public void setWantedState(WantedState state) {
        this.wantedState = state;
    }

    private SystemState handleStateTransition() {
        return switch (wantedState) {
            case HOME -> SystemState.HOME;
            case STOPPED -> SystemState.STOPPED;
            case AIM_AT_TARGET -> SystemState.AIM_AT_TARGET;
            case SET_SHOT -> SystemState.SET_SHOT;
        };
    }

    /**
     * Hood angle the current shot wants this loop, degrees; NaN when the hood is not on a shot.
     * Computed from logged inputs, so the shot gate that compares it with {@link
     * #getPositionDegrees()} replays.
     */
    @Getter private double shotTargetDegrees = Double.NaN;

    private boolean goalWasClamped = false;

    private void applyStates(HoodHoming.Action action) {
        double requestedDegrees = homing.getParams().stopDeg();
        switch (systemState) {
            case HOME, STOPPED:
                break;
            case AIM_AT_TARGET:
                requestedDegrees = ShotCalculator.getInstance().getParameters().hoodAngle();
                break;
            case SET_SHOT:
                requestedDegrees = ShotCalculator.getSetShotHoodDegrees();
                break;
        }
        // The shot model can ask past the soft limits the hood booted with (a narrowed hoodForward
        // param). The hood would stall at the limit while the shot gate waited out its timeout,
        // so it goes to the limit, and the gate and the tracking stop compare against that.
        final double requested = requestedDegrees;
        double wantedDegrees =
                rotationsToDegrees(() -> clampToSoftLimits(degreesToRotations(() -> requested)));
        boolean clamped = Math.abs(wantedDegrees - requestedDegrees) > 1e-9;
        boolean onShot =
                systemState == SystemState.AIM_AT_TARGET || systemState == SystemState.SET_SHOT;
        shotTargetDegrees = onShot ? wantedDegrees : Double.NaN;
        // Hood angle the current state asked for, before clamping to the soft limits.
        Telemetry.log("Hood/RequestedDegrees", requestedDegrees, "degrees");
        // The request was past a soft limit, so the hood goes to the limit instead.
        Telemetry.log("Hood/GoalClamped", clamped);
        if (clamped && !goalWasClamped) {
            Telemetry.print(
                    String.format(
                            "Hood: %.1f deg asked, past the soft limits; going to %.1f deg",
                            requestedDegrees, wantedDegrees));
        }
        goalWasClamped = clamped;
        // Homing owns the motor while it runs, and refuses setpoints until the hood is zeroed.
        if (action == HoodHoming.Action.DRIVE_TO_STOP) {
            setVoltageOutputNoSoftLimit(() -> homing.getParams().volts());
            return;
        }
        if (action == HoodHoming.Action.HOLD || systemState == SystemState.STOPPED) {
            stop();
            return;
        }
        final double finalWantedDegrees = wantedDegrees;
        final double finalWantedPosition = degreesToRotations(() -> finalWantedDegrees);
        setMMPositionFoc(() -> finalWantedPosition);
    }

    @Getter private final HoodConfig config;
    @Getter private HoodSim sim;

    private final HoodHoming homing = new HoodHoming(HoodHoming.Params.fm());
    private final Alert homingTimeoutAlert =
            new Alert(
                    "Hood homing timed out before finding the bottom stop - check the hood;"
                            + " unless Hood/Zeroed, it stays in brake until the next mode start",
                    Level.HIGH);

    /**
     * The hood has found its bottom stop since boot and its readings show the zero, so its angle
     * can be trusted. Until then it holds in brake and the shot gate holds.
     */
    public boolean isZeroed() {
        return assumedZeroedForReplay || (homing.isZeroed() && !isPositionWritePending());
    }

    private final HoodHoming.LogMarker homingLogMarker = new HoodHoming.LogMarker();

    /** Replaying a log from before homing existed: see {@link HoodHoming#assumeZeroedForReplay}. */
    private boolean assumedZeroedForReplay = false;

    private HoodHoming.State homingState = HoodHoming.State.NOT_ZEROED;

    /** The homing state for the pre-match check; the hood homes itself at auto start. */
    public HomingCheck.State getHomingCheckState() {
        if (!getInputs().connected) {
            return HomingCheck.State.DISCONNECTED;
        }
        return switch (homingState) {
            case ZEROED -> HomingCheck.State.HOMED;
            case NOT_ZEROED, HOMING -> HomingCheck.State.HOMES_WHEN_ENABLED;
            case TIMED_OUT -> HomingCheck.State.FAILED;
        };
    }

    public Hood(HoodConfig config) {
        super(config);
        this.config = config;

        setInitialPosition();

        simulationInit();
        Telemetry.print(getName() + " Subsystem Initialized");
    }

    private void setInitialPosition() {
        if (isAttached()) {
            motor.setPosition(degreesToRotations(() -> config.getInitPosition()));
        }
    }

    @Override
    public void periodic() {
        Telemetry.time("Scheduler/Periodic/Hood");
        Logger.processInputs("Hood/Homing", homingLogMarker);
        assumedZeroedForReplay =
                HoodHoming.assumeZeroedForReplay(
                        Constants.currentMode == Constants.Mode.REPLAY,
                        homingLogMarker.recordedWithHoming);
        // Right after a zero the readings are still from the old one: no angle until they catch up.
        double positionDeg = isPositionWritePending() ? Double.NaN : getPositionDegrees();
        var homed =
                assumedZeroedForReplay
                        ? HoodHoming.assumedZeroed()
                        : homing.update(
                                new HoodHoming.Inputs(
                                        Timer.getTimestamp(),
                                        RobotState.isEnabled(),
                                        RobotState.isAutonomous(),
                                        RobotState.isTeleop(),
                                        isMotorConnected(),
                                        motorResetThisLoop(),
                                        positionDeg,
                                        getVelocityRPM() * 6.0, // mechanism RPM to deg/s
                                        wantedState == WantedState.HOME,
                                        isTrackingStopped()));
        if (homed.resetPosition()) {
            setMotorPosition(() -> degreesToRotations(() -> homing.getParams().stopDeg()));
        }
        homingTimeoutAlert.set(homed.timedOut());
        homingState = homed.state();
        systemState = handleStateTransition();
        applyStates(homed.action());
        logBatteryUsage();
        // What SuperStructure asked of the hood.
        Telemetry.log("Hood/WantedState", wantedState.toString());
        // The state the hood is actually in, after its own transitions.
        Telemetry.log("Hood/SystemState", systemState.toString());
        // The command requiring the hood, if any.
        Telemetry.log("Hood/CurrentCommand", getCurrentCommandName());
        // The hood has found its bottom stop since boot, so its angle is real.
        Telemetry.log("Hood/Zeroed", isZeroed());
        // Replaying a log from before hood homing existed, so the hood is taken as zeroed.
        Telemetry.log("Hood/Homing/AssumedZeroedForReplay", assumedZeroedForReplay);
        // NotZeroed, Homing, TimedOut or Zeroed.
        Telemetry.log("Hood/HomingState", homed.state().label);
        // Voltage, currents, temperature and connection at 10 Hz (offseason); the
        // same signals are also recorded every loop as inputs under Mechanisms/.
        logDiagnostics("Hood");
        // Hood angle.
        Telemetry.log("Hood/PositionDegrees", getPositionDegrees(), "degrees");
        // Hood speed, mechanism RPM.
        Telemetry.log("Hood/RPM", getVelocityRPM(), "RPM");
        Telemetry.timeEnd("Scheduler/Periodic/Hood");
    }

    // --------------------------------------------------------------------------------
    // Simulation
    // --------------------------------------------------------------------------------
    public void simulationInit() {
        if (isAttached()) {
            sim = new HoodSim(RobotSim.leftView, motor);
        }
    }

    // Must be called to enable the simulation.
    // Set x and y when the roller position changes.
    // Simulation steps itself on spectrumLib SimLoop (200 Hz).

    class HoodSim extends ArmSim {
        public HoodSim(Mechanism2d mech, TalonFX armMotorSim) {
            super(
                    new ArmConfig(
                                    config.hoodX,
                                    config.hoodY,
                                    config.simRatio,
                                    config.length,
                                    90,
                                    180 - 9,
                                    180 - 9)
                            .setSimulatedGravity(false),
                    mech,
                    armMotorSim,
                    config.getName());
        }
    }
}
