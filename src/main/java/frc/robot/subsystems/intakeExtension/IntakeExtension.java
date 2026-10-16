package frc.robot.subsystems.intakeExtension;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;
import frc.robot.RobotSim;
import frc.spectrumLib.hardware.CanBuses;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.sim.LinearConfig;
import frc.spectrumLib.sim.LinearSim;
import frc.spectrumLib.telemetry.Telemetry;
import lombok.Getter;
import org.wpilib.command2.Command;
import org.wpilib.command2.InstantCommand;
import org.wpilib.driverstation.RobotState;
import org.wpilib.math.util.Units;
import org.wpilib.smartdashboard.Mechanism2d;
import org.wpilib.system.Timer;
import org.wpilib.util.Color;
import org.wpilib.util.Color8Bit;

/**
 * The Intake Extension subsystem. Extends and retracts the fuel intake.
 *
 * <p>The deploy is a rack-and-pinion driven by two independent motors: a left axis (this class, CAN
 * id 4, Clockwise_Positive) and a right axis ({@link IntakeExtensionRight}, CAN id 5,
 * CounterClockwise_Positive). Each side runs its own closed-loop position control rather than one
 * following the other, so a side that skips teeth on the rack can be driven on its own to resync.
 *
 * <p>Normal deploy/retract states command both axes to the same setpoint. The {@code RESYNC} state
 * re-establishes truth by driving each side independently into the fully-extended hard stop (using
 * a soft-limit-bypassing voltage), detecting the stall, and re-zeroing that side's encoder at
 * {@code maxRotations}. This recovers from a tooth skip, which otherwise leaves the motor encoder
 * reading a position the rack is no longer at.
 */
public class IntakeExtension extends Mechanism {

    public static class IntakeExtensionConfig extends Config {

        @Getter private final double initPosition = 0;
        @Getter private final double triggerTolerance = 5;

        /* Intake Extension config settings */
        @Getter private final double zeroSpeed = -0.1;
        @Getter private final double holdMaxSpeedRPM = 18;

        @Getter private final double maxRotations = 2.8;
        @Getter private final double minRotations = 0.0;

        @Getter private final double supplyCurrentLimit = 20;
        @Getter private final double statorCurrentLimit = 40;
        @Getter private final double lowerSupplyCurrentLimit = 20;
        @Getter private final double lowerSupplyCurrentTime = 0;

        @Getter private final double positionKp = 13;
        @Getter private final double positionKi = 0;
        @Getter private final double positionKd = 0;
        @Getter private final double positionKv = 1.0;
        @Getter private final double positionKs = 2.0;
        @Getter private final double positionKa = 0;
        @Getter private final double positionKg = 0;
        @Getter private final double gearRatio = 11.25;
        @Getter private final double mmCruiseVelocity = 100;
        @Getter private final double mmAcceleration = 300;
        @Getter private final double mmJerk = 1000;
        @Getter private final double slowMmCruiseVelocity = 4;
        @Getter private final double slowMmAcceleration = 20;
        @Getter private final double slowMmJerk = 1000;

        @Getter private final double sensorToMechanismRatio = 11.25;
        @Getter private final double rotorToSensorRatio = 1;
        @Getter private final double CANcoderRotorToSensorRatio = 1.7;
        @Getter private final double CANcoderSensorToMechanismRatio = 1;
        @Getter private final double CANcoderOffset = 0;
        @Getter private final boolean CANcoderAttached = false;

        /* Resync / stall-homing settings (toward the fully-extended hard stop) */
        @Getter private final double homingVoltage = 6;
        @Getter private final double homingStallRPM = 50.0;
        @Getter private final double homingMinTimeSecs = 0.3;
        @Getter private final double homingStallDebounceSecs = 0.15;
        @Getter private final double homingTimeoutSecs = 3.0;

        /*
         * Full extend drives out, then stops pushing (from the offseason bot). Once both sides have
         * been still for the steady time, at the target or stalled short of it, the motors go to
         * neutral and brake mode holds the extension. On FM the left side stops about 0.1 to 0.2
         * rot short of the right and sat at its 40 A stator limit for as long as the intake was
         * out (2026-10-16 log). 1 rot is about 3 in of travel (0.955 in drum).
         */
        @Getter private final double extendSteadySecs = 0.3;

        @Getter private final double extendSteadyMaxRPM = 18;
        @Getter private final double extendAtTargetToleranceRotations = 0.08;

        /** While holding, pushed back in by this much (about 2 in) drives out again. */
        @Getter private final double extendReextendRotations = 0.67;

        /*
         * Out-point relearn, at the same stall. The encoders are not zeroed when the code starts,
         * so zero is wherever the motors powered on. If that was with the intake out, the first
         * full extend cannot move, and the stop is taken as max.
         */

        /**
         * Relearn only when the extension moved less than this since the extend began. A jam or a
         * tooth skip part way out moves first and then stalls, and taking that as max would be
         * wrong; RESYNC is the fix for those.
         */
        @Getter private final double relearnMaxTravelRotations = 0.25;

        /** Both sides must read within this of each other, or one is bound up short. */
        @Getter private final double relearnMaxSkewRotations = 0.33;

        /* Sim Configs */
        @Getter private final double intakeX = Units.inchesToMeters(70);
        @Getter private final double intakeY = Units.inchesToMeters(23);
        @Getter private final double extensionMass = 10.0;
        @Getter private final double drumRadiusMeters = Units.inchesToMeters(0.955 / 2);
        @Getter private final double extensionGearing = 11.25;
        @Getter private final double angle = 180;
        @Getter private final double staticLength = 10;
        @Getter private final double movingLength = 55;
        @Getter private final double lineWidth = 20;
        @Getter private final double maxExtensionHeight = 40;

        public IntakeExtensionConfig() {
            super("IntakeExtension", 4, CanBuses.MECHANISMS);
            configMinMaxRotations(minRotations, maxRotations);
            configPIDGains(0, positionKp, positionKi, positionKd);
            configFeedForwardGains(positionKs, positionKv, positionKa, positionKg);
            configMotionMagic(mmCruiseVelocity, mmAcceleration, mmJerk);
            configSupplyCurrentLimit(supplyCurrentLimit, true);
            configStatorCurrentLimit(statorCurrentLimit, true);
            configLowerSupplyCurrentLimit(lowerSupplyCurrentLimit);
            configLowerSupplyCurrentTime(lowerSupplyCurrentTime);
            configGearRatio(gearRatio);
            configForwardTorqueCurrentLimit(statorCurrentLimit);
            configReverseTorqueCurrentLimit(statorCurrentLimit);
            configForwardSoftLimit(maxRotations, true);
            configReverseSoftLimit(minRotations, true);
            configNeutralBrakeMode(true);
            configClockwise_Positive();
        }

        public IntakeExtensionConfig applyMotorConfig(TalonFX motor) {
            TalonFXConfigurator configurator = motor.getConfigurator();
            TalonFXConfiguration talonConfigMod = getTalonConfig();

            configurator.apply(talonConfigMod);
            talonConfig = talonConfigMod;
            return this;
        }
    }

    // ================================================================================
    // Right Axis. Independent, closed-loop, mirror-mounted.
    // ================================================================================

    /**
     * The right deploy axis. A standalone {@link Mechanism} (not a follower) so it can be driven
     * independently of the left during a resync. Gains, limits, and geometry mirror the left
     * config; only the CAN id, name, and motor inversion differ (the right gearbox is mirrored, so
     * it is {@code CounterClockwise_Positive} where the left is {@code Clockwise_Positive}, giving
     * both axes the same "positive = extend" convention).
     */
    public static class IntakeExtensionRight extends Mechanism {

        public static class RightConfig extends Config {
            public RightConfig(IntakeExtensionConfig left) {
                super("IntakeExtensionRight", 5, CanBuses.MECHANISMS);
                setAttached(left.isAttached());
                configMinMaxRotations(left.getMinRotations(), left.getMaxRotations());
                configPIDGains(0, left.getPositionKp(), left.getPositionKi(), left.getPositionKd());
                configFeedForwardGains(
                        left.getPositionKs(),
                        left.getPositionKv(),
                        left.getPositionKa(),
                        left.getPositionKg());
                configMotionMagic(
                        left.getMmCruiseVelocity(), left.getMmAcceleration(), left.getMmJerk());
                configSupplyCurrentLimit(left.getSupplyCurrentLimit(), true);
                configStatorCurrentLimit(left.getStatorCurrentLimit(), true);
                configLowerSupplyCurrentLimit(left.getLowerSupplyCurrentLimit());
                configLowerSupplyCurrentTime(left.getLowerSupplyCurrentTime());
                configGearRatio(left.getGearRatio());
                configForwardTorqueCurrentLimit(left.getStatorCurrentLimit());
                configReverseTorqueCurrentLimit(left.getStatorCurrentLimit());
                configForwardSoftLimit(left.getMaxRotations(), true);
                configReverseSoftLimit(left.getMinRotations(), true);
                configNeutralBrakeMode(true);
                configCounterClockwise_Positive();
            }
        }

        @Getter private final RightConfig rightConfig;

        public IntakeExtensionRight(IntakeExtensionConfig leftConfig) {
            super(new RightConfig(leftConfig));
            this.rightConfig = (RightConfig) super.config;
            Telemetry.print(getName() + " Subsystem Initialized");
        }

        /** Closed-loop Motion Magic to an absolute rotation target. */
        public void goToRotations(double rotations) {
            setMMPosition(() -> rotations);
        }

        /** Slow (dynamic Motion Magic voltage) move to a rotation target. */
        public void goToRotationsSlow(
                double rotations, double cruiseVelocity, double acceleration, double jerk) {
            setDynMMPositionVoltage(
                    () -> rotations, () -> cruiseVelocity, () -> acceleration, () -> jerk);
        }

        /** Open-loop voltage that bypasses soft limits. Used to drive into the hard stop. */
        public void driveHomingVoltage(double volts) {
            setVoltageOutputNoSoftLimit(() -> volts);
        }

        /** Seeds this axis's encoder to a known starting position (rotations). */
        public void setInitialPosition(double rotations) {
            if (isAttached()) {
                motor.setPosition(rotations);
            }
        }

        /** Re-zeroes this axis at the fully-extended hard stop. */
        public void zeroAtMax() {
            setMotorPosition(() -> rightConfig.getMaxRotations());
        }

        /** Holds the axis (neutral output). */
        public void stopAxis() {
            stop();
        }

        @Override
        public void periodic() {
            logBatteryUsage();
            Telemetry.log("IntakeExtensionRight/CurrentCommand", getCurrentCommandName());
            // Voltage, currents, temperature and connection at 10 Hz (offseason); the
            // same signals are also recorded every loop as inputs under Mechanisms/.
            logDiagnostics("IntakeExtensionRight");
            Telemetry.log("IntakeExtensionRight/Position", getPositionRotations(), "rotations");
            Telemetry.log("IntakeExtensionRight/RPM", getVelocityRPM(), "RPM");
        }
    }

    // ---- Subsystem plumbing ----

    @Getter private final IntakeExtensionConfig config;
    @Getter private IntakeExtensionSim sim;
    private final IntakeExtensionRight right;

    public IntakeExtension(IntakeExtensionConfig config) {
        super(config);
        this.config = config;
        this.right = new IntakeExtensionRight(config);

        // No encoder zeroing here, as on the offseason bot. The TalonFX keeps counting across
        // robot-code restarts, so zeroing in the constructor threw the position away on every
        // deploy: deployed with the intake out, retract did nothing and extend drove into the
        // stop. Zero is wherever the extension was at motor power-on. If that was with the
        // intake out, the first full extend relearns the out point (applyFullExtend).

        simulationInit();
        Telemetry.print(getName() + " Subsystem Initialized");
    }

    private void setInitialPosition() {
        if (isAttached()) {
            double initialRotations = config.getInitPosition();
            motor.setPosition(initialRotations);
            right.setInitialPosition(initialRotations);
        }
    }

    public void resetCurrentPositionToMax() {
        if (isAttached()) {
            motor.setPosition(config.getMaxRotations());
        }
        if (right.isAttached()) right.zeroAtMax();
    }

    public Command resetCurrentPositionToMaxCommand() {
        return new InstantCommand(this::resetCurrentPositionToMax);
    }

    public Command resetToInitialPos() {
        return new InstantCommand(this::setInitialPosition);
    }

    /** Sets brake mode on both deploy axes. */
    @Override
    public void setBrakeMode(boolean isInBrake) {
        super.setBrakeMode(isInBrake);
        if (right.isAttached()) right.setBrakeMode(isInBrake);
    }

    // ---- State Machine ----

    public enum WantedState {
        STOPPED,
        FULL_EXTEND,
        CONDITIONAL_EXTEND,
        FULL_RETRACT,
        SLOW_CLOSE,
        RESYNC,
    }

    public enum SystemState {
        STOPPED,
        FULL_EXTEND,
        FULL_RETRACT,
        SLOW_CLOSE,
        HOMING,
    }

    private WantedState wantedState = WantedState.STOPPED;
    private SystemState systemState = SystemState.STOPPED;
    private SystemState previousSystemState = SystemState.STOPPED;
    private boolean sentOutByIntakeState = false;

    public void setWantedState(WantedState state) {
        this.wantedState = state;
    }

    private SystemState handleStateTransition() {
        return switch (wantedState) {
            case STOPPED -> SystemState.STOPPED;
            case FULL_EXTEND -> {
                sentOutByIntakeState = true;
                yield SystemState.FULL_EXTEND;
            }
            case CONDITIONAL_EXTEND ->
                    sentOutByIntakeState ? SystemState.FULL_EXTEND : SystemState.STOPPED;
            case FULL_RETRACT -> {
                sentOutByIntakeState = false;
                yield SystemState.FULL_RETRACT;
            }
            case SLOW_CLOSE -> SystemState.SLOW_CLOSE;
            case RESYNC -> SystemState.HOMING;
        };
    }

    private void applyStates() {
        switch (systemState) {
            case FULL_EXTEND:
                applyFullExtend();
                break;
            case FULL_RETRACT:
                commandBoth(0, false);
                break;
            case SLOW_CLOSE:
                commandBoth(25, true);
                break;
            case HOMING:
                applyHoming();
                break;
            case STOPPED:
                stop();
                if (right.isAttached()) right.stopAxis();
                return;
        }
    }

    /**
     * Commands both axes to the same position.
     *
     * @param percent target as a percentage of max rotations (0 to 100)
     * @param slow whether to use the slow (dynamic Motion Magic voltage) profile
     */
    private void commandBoth(double percent, boolean slow) {
        final double rotations = percentToRotations(() -> percent);
        if (slow) {
            setDynMMPositionVoltage(
                    () -> rotations,
                    () -> config.getSlowMmCruiseVelocity(),
                    () -> config.getSlowMmAcceleration(),
                    () -> config.getSlowMmJerk());
            if (right.isAttached()) {
                right.goToRotationsSlow(
                        rotations,
                        config.getSlowMmCruiseVelocity(),
                        config.getSlowMmAcceleration(),
                        config.getSlowMmJerk());
            }
        } else {
            setMMPosition(() -> rotations);
            if (right.isAttached()) right.goToRotations(rotations);
        }
    }

    // ---- Full extend: drive out, then stop pushing ----

    private enum ExtendPhase {
        DRIVING,
        HOLDING
    }

    private ExtendPhase extendPhase = ExtendPhase.DRIVING;
    private final Timer extendSteadyTimer = new Timer();
    private boolean extendSteadyTiming = false;
    private double extendStartRotations = 0;
    private double holdStartRotations = 0;
    private int outPointRelearns = 0;

    /**
     * Drives both sides out until they have been still for the steady time, then stops pushing and
     * lets brake mode hold. If the extension is pushed back in while holding, it drives out again.
     *
     * <p>At that stall, if the extend barely moved, both sides read the same place and it is short
     * of the target, the out stop is taken as max. That is a power-on (or a motor restart) with the
     * intake out. A jam or tooth skip part way out moves first and then stalls; RESYNC is the fix
     * for those.
     */
    private void applyFullExtend() {
        // The sim models only the left axis; its right axis has no physics and wanders, so it
        // counts only off the sim (on the robot, and in a replay of a robot log).
        boolean useRight = right.isAttached() && Constants.currentMode != Constants.Mode.SIM;
        double leftPos = getPositionRotations();
        double rightPos = useRight ? right.getPositionRotations() : leftPos;
        double position = (leftPos + rightPos) / 2.0;

        // Disabled, the motors are off and every extend looks like a stall. Start the extend over
        // when the robot is enabled.
        if (previousSystemState != SystemState.FULL_EXTEND || !RobotState.isEnabled()) {
            extendPhase = ExtendPhase.DRIVING;
            extendStartRotations = position;
            extendSteadyTiming = false;
        }

        boolean still =
                Math.abs(getVelocityRPM()) < config.getExtendSteadyMaxRPM()
                        && (!useRight
                                || Math.abs(right.getVelocityRPM())
                                        < config.getExtendSteadyMaxRPM());
        boolean steady = false;
        if (still && RobotState.isEnabled()) {
            if (!extendSteadyTiming) {
                extendSteadyTiming = true;
                extendSteadyTimer.restart();
            }
            steady = extendSteadyTimer.hasElapsed(config.getExtendSteadySecs());
        } else {
            extendSteadyTiming = false;
        }

        switch (extendPhase) {
            case DRIVING -> {
                if (!steady) {
                    commandBoth(100, false);
                    return;
                }
                boolean atTarget =
                        position
                                >= config.getMaxRotations()
                                        - config.getExtendAtTargetToleranceRotations();
                boolean barelyMoved =
                        Math.abs(position - extendStartRotations)
                                < config.getRelearnMaxTravelRotations();
                boolean sidesAgree =
                        Math.abs(leftPos - rightPos) <= config.getRelearnMaxSkewRotations();
                if (!atTarget && barelyMoved && sidesAgree) {
                    resetCurrentPositionToMax();
                    outPointRelearns++;
                    Telemetry.print("IntakeExtension: out point relearned at the stop");
                    position = config.getMaxRotations();
                }
                // At the target or stalled short of it: stop pushing.
                extendPhase = ExtendPhase.HOLDING;
                holdStartRotations = position;
                stopBoth();
            }
            case HOLDING -> {
                if (holdStartRotations - position >= config.getExtendReextendRotations()) {
                    extendPhase = ExtendPhase.DRIVING;
                    extendStartRotations = position;
                    extendSteadyTiming = false;
                    commandBoth(100, false);
                } else {
                    stopBoth();
                }
            }
        }
    }

    private void stopBoth() {
        stop();
        if (right.isAttached()) right.stopAxis();
    }

    // ---- Resync / stall homing ----

    private final Timer homingTimer = new Timer();
    private boolean leftHomed = false;
    private boolean rightHomed = false;
    // Timestamp (homingTimer seconds) each side was last seen moving above the stall speed.
    private double leftLastMoving = 0;
    private double rightLastMoving = 0;

    /**
     * Drives each side independently into the fully-extended hard stop, re-zeroing that side's
     * encoder at {@code maxRotations} once it stalls. A tooth skip corrupts the motor encoder, so
     * the hard stop is the only reliable position reference. Homing uses a soft-limit-bypassing
     * voltage so it can reach the physical stop even when the (stale) encoder thinks the soft limit
     * is already reached.
     */
    private void applyHoming() {
        // Re-arm on entry to the HOMING state.
        if (previousSystemState != SystemState.HOMING) {
            homingTimer.restart();
            leftHomed = false;
            rightHomed = false;
            leftLastMoving = 0;
            rightLastMoving = 0;
        }

        boolean timedOut = homingTimer.get() >= config.getHomingTimeoutSecs();

        // ── Left side (this mechanism) ──
        if (!leftHomed) {
            if (detectLeftStall()) {
                setMotorPosition(() -> config.getMaxRotations());
                stop();
                leftHomed = true;
            } else if (timedOut) {
                Telemetry.print("IntakeExtension: LEFT resync timed out");
                stop();
                leftHomed = true;
            } else {
                setVoltageOutputNoSoftLimit(() -> config.getHomingVoltage());
            }
        } else {
            stop();
        }

        // ── Right side (independent axis) ──
        if (right.isAttached()) {
            if (!rightHomed) {
                if (detectRightStall()) {
                    right.zeroAtMax();
                    right.stopAxis();
                    rightHomed = true;
                } else if (timedOut) {
                    Telemetry.print("IntakeExtension: RIGHT resync timed out");
                    right.stopAxis();
                    rightHomed = true;
                } else {
                    right.driveHomingVoltage(config.getHomingVoltage());
                }
            } else {
                right.stopAxis();
            }
        } else {
            rightHomed = true; // No right axis attached, so nothing to resync.
        }
    }

    private boolean detectLeftStall() {
        double now = homingTimer.get();
        if (Math.abs(getVelocityRPM()) >= config.getHomingStallRPM()) {
            leftLastMoving = now;
        }
        return isStalled(now, leftLastMoving);
    }

    private boolean detectRightStall() {
        double now = homingTimer.get();
        if (Math.abs(right.getVelocityRPM()) >= config.getHomingStallRPM()) {
            rightLastMoving = now;
        }
        return isStalled(now, rightLastMoving);
    }

    /**
     * A side is stalled once it has gone the debounce window without moving above the stall speed,
     * but only after the minimum drive time has elapsed (so the initial pre-motion zero velocity is
     * not mistaken for a stall).
     */
    private boolean isStalled(double now, double lastMoving) {
        if (now < config.getHomingMinTimeSecs()) {
            return false;
        }
        return (now - lastMoving) >= config.getHomingStallDebounceSecs();
    }

    /** Whether the most recent resync has finished homing both sides. */
    public boolean isResyncComplete() {
        return systemState == SystemState.HOMING && leftHomed && rightHomed;
    }

    /**
     * Runs a full resync: drives both sides into the extended hard stop, re-zeros each, then
     * returns the subsystem to {@code STOPPED}.
     *
     * @return a command that completes once both sides are re-zeroed
     */
    public Command resyncCommand() {
        return startEnd(
                        () -> setWantedState(WantedState.RESYNC),
                        () -> setWantedState(WantedState.STOPPED))
                .until(this::isResyncComplete)
                .withName("IntakeExtension.resync");
    }

    @Override
    public void periodic() {
        systemState = handleStateTransition();
        applyStates();
        logBatteryUsage();
        Telemetry.log("IntakeExtension/WantedState", wantedState.toString());
        Telemetry.log("IntakeExtension/SystemState", systemState.toString());
        Telemetry.log("IntakeExtension/CurrentCommand", getCurrentCommandName());
        // Voltage, currents, temperature and connection at 10 Hz (offseason); the
        // same signals are also recorded every loop as inputs under Mechanisms/.
        logDiagnostics("IntakeExtension");
        Telemetry.log("IntakeExtension/Position", getPositionRotations(), "rotations");
        Telemetry.log("IntakeExtension/RPM", getVelocityRPM(), "RPM");
        Telemetry.log("IntakeExtension/LeftHomed", leftHomed);
        Telemetry.log("IntakeExtension/RightHomed", rightHomed);
        Telemetry.log("IntakeExtension/OutPointRelearns", outPointRelearns);
        Telemetry.log("IntakeExtension/ExtendPhase", extendPhase.toString());

        previousSystemState = systemState;
    }

    // --------------------------------------------------------------------------------
    // Simulation
    // --------------------------------------------------------------------------------
    public void simulationInit() {
        if (isAttached()) {
            sim = new IntakeExtensionSim(RobotSim.leftView, motor);
        }
    }

    // Simulation steps itself on spectrumLib SimLoop (200 Hz).

    class IntakeExtensionSim extends LinearSim {
        public IntakeExtensionSim(Mechanism2d mech, TalonFX intakeExtensionMotorSim) {
            super(
                    new LinearConfig(
                                    config.getIntakeX(),
                                    config.getIntakeY(),
                                    config.getExtensionGearing(),
                                    config.getDrumRadiusMeters())
                            .setAngle(config.getAngle())
                            .setMovingLength(config.getMovingLength())
                            .setStaticLength(config.getStaticLength())
                            .setMaxHeight(config.getMaxExtensionHeight())
                            .setLineWidth(config.getLineWidth())
                            .setColor(new Color8Bit(Color.LIGHT_GRAY)),
                    mech,
                    intakeExtensionMotorSim,
                    config.getName());
        }
    }
}
