// Based on
// https://github.com/CrossTheRoadElec/Phoenix6-Examples/blob/main/java/SwerveWithPathPlanner/src/main/java/frc/robot/subsystems/CommandSwerveDrivetrain.java
package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;
import frc.rebuilt.Field;
import frc.rebuilt.FieldHelpers;
import frc.rebuilt.ShotCalculator;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.spectrumLib.framework.RobotLoop;
import frc.spectrumLib.hardware.CanConfigBudget;
import frc.spectrumLib.localization.PoseFusion;
import frc.spectrumLib.telemetry.Alert;
import frc.spectrumLib.telemetry.Telemetry;
import frc.spectrumLib.util.Util;
import java.util.Optional;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.Command;
import org.wpilib.command2.Subsystem;
import org.wpilib.command2.button.Trigger;
import org.wpilib.driverstation.Alert.Level;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.driverstation.MatchState;
import org.wpilib.hardware.hal.HALUtil;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rectangle2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveDriveOdometry;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.util.Units;
import org.wpilib.system.Notifier;
import org.wpilib.system.RobotController;
import org.wpilib.system.Timer;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;

/**
 * FM's swerve drivetrain: Phoenix 6's {@link SwerveDrivetrain} for control and 250 Hz odometry,
 * with the robot pose owned by {@link PoseFusion} on the main loop so it can be replayed.
 *
 * <h2>Where the pose comes from</h2>
 *
 * <p>CTRE's odometry thread still runs every module at 250 Hz. Its telemetry callback, on that
 * thread, only copies each sample into a bounded queue. Each robot loop {@link #periodic()} drains
 * the queue into {@link SwerveInputs}, records it with AdvantageKit, and hands every sample to
 * {@link PoseFusion}, which is the pose everything on the robot reads ({@link #getRobotPose()}).
 * Vision measurements go to {@link PoseFusion} too (see {@code Vision}). CTRE's internal estimate
 * is kept aligned with the fused pose ({@link #syncCtrePose()}) because its field-centric requests
 * rotate by its own heading.
 *
 * <h2>From the 2026 offseason branch</h2>
 *
 * <p>Per-module current and connection logging at 10 Hz, the {@link SwerveAlignment} encoder
 * publisher, skipping the bus optimisation on a dead bus ({@link CanConfigBudget}), and never doing
 * log work on CTRE's odometry thread. FM's own drive modes (pilot aim at target through {@link
 * ShotCalculator}) are kept from {@code main}; the offseason bot's turret-pivot mode is not.
 *
 * <h2>Simulation</h2>
 *
 * <p>maple-sim has no WPILib 2027 build, so the sim is CTRE's own {@code updateSimState}. To give
 * the vision comparison something to do, the sim keeps a separate perfect <em>truth</em> odometry
 * and feeds the robot code a version with wheel-radius and gyro-drift error ({@link SimErrors}).
 */
public class Swerve extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements Subsystem {

    // ── State machine ──────────────────────────────────────────────────────────────────
    public enum WantedState {
        TELEOP_DRIVE,
        PILOT_AIM_AT_TARGET,
        X_BRAKE,
        IDLE
    }

    public enum SystemState {
        TELEOP_DRIVE,
        PILOT_AIM_AT_TARGET,
        X_BRAKE,
        IDLE
    }

    private WantedState wantedState = WantedState.IDLE;
    private SystemState systemState = SystemState.IDLE;

    public static final double TRANSLATION_ERROR_MARGIN_METERS = Units.inchesToMeters(1.0);
    public static final double DRIVE_TO_POINT_STATIC_FRICTION_CONSTANT = 0.02;
    private static final double SKEW_COMPENSATION_SCALAR = -0.03;

    @Getter @Setter private double teleopVelocityCoefficient = 1.0;
    @Getter @Setter private double rotationVelocityCoefficient = 1.0;

    @Getter private final SwerveConfig config;
    private Notifier simNotifier = null;

    private final Alert pigeonAlert = new Alert("Pigeon IMU Disconnected", Level.HIGH);

    private final SwerveRequest.ApplyRobotVelocity autoRequest =
            new SwerveRequest.ApplyRobotVelocity()
                    .withDriveRequestType(DriveRequestType.Velocity)
                    .withSteerRequestType(SteerRequestType.Position)
                    .withDesaturateWheelVelocities(true);

    private final SwerveRequest.ApplyFieldVelocity fieldCentricDrive =
            new SwerveRequest.ApplyFieldVelocity()
                    .withDriveRequestType(DriveRequestType.Velocity)
                    .withSteerRequestType(SteerRequestType.Position);

    private final SwerveRequest.FieldCentricFacingAngle driveAtAngleRequest =
            new SwerveRequest.FieldCentricFacingAngle()
                    .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
                    .withSteerRequestType(SwerveModule.SteerRequestType.Position);

    private final SwerveRequest.SwerveDriveBrake xBrake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();

    /** Publishes raw CANcoder data for the swerve alignment tool. */
    private final SwerveAlignment alignment;

    // ── Replayable pose ────────────────────────────────────────────────────────────────

    @Getter private final SwerveInputs inputs = new SwerveInputs();

    /** The robot pose and every per-source comparison track. */
    @Getter private final PoseFusion poseFusion;

    /** One odometry sample, copied off CTRE's odometry thread. */
    private record OdometrySample(
            double ctreTimestamp, Rotation2d rawHeading, SwerveModulePosition[] positions) {}

    /**
     * Samples waiting for the main loop. 250 Hz is two or three per 10 ms loop; 64 rides out a loop
     * stall of a quarter second. When full, the oldest sample is the one lost.
     */
    private final ArrayBlockingQueue<OdometrySample> odometryQueue = new ArrayBlockingQueue<>(64);

    private final StatusSignal<AngularVelocity> yawRateSignal;
    private final StatusSignal<Angle> pitchSignal;
    private final StatusSignal<Angle> rollSignal;

    /** Module stator/supply currents: logged at 10 Hz, so 20 Hz frames are plenty. */
    private static final double MODULE_CURRENT_HZ = 20;

    /** Last heading the aim request was asked to hold, for a replayable at-rotation check. */
    private Rotation2d aimTarget = Rotation2d.kZero;

    /**
     * Constructs a new Swerve drive subsystem.
     *
     * @param config drivetrain constants and module configurations
     */
    public Swerve(SwerveConfig config) {
        super(
                TalonFX::new,
                TalonFX::new,
                CANcoder::new,
                config.getDrivetrainConstants(),
                250.0,
                config.getModules());

        this.config = config;

        Pigeon2 pigeon = getPigeon2();
        yawRateSignal = pigeon.getAngularVelocityZWorld(false);
        pitchSignal = pigeon.getPitch(false);
        rollSignal = pigeon.getRoll(false);
        // Pitch and roll only. The yaw rate (AngularVelocityZWorld) is one of the signals CTRE's
        // 250 Hz odometry thread waits on for latency compensation; setting it to 100 Hz here
        // would starve that wait (WaitForAll -1003, "CAN message is stale" on the Pigeon).
        BaseStatusSignal.setUpdateFrequencyForAll(100, pitchSignal, rollSignal);

        SwerveModulePosition[] startPositions = new SwerveModulePosition[getModules().length];
        for (int i = 0; i < startPositions.length; i++) {
            startPositions[i] = new SwerveModulePosition();
        }
        poseFusion =
                new PoseFusion(
                        getKinematics(),
                        Rotation2d.kZero,
                        startPositions,
                        new double[] {0.1, 0.1, 0.1});

        if (Constants.hasHardware()) {
            registerTelemetry(this::queueOdometrySample);
        }
        if (Constants.currentMode == Constants.Mode.SIM) {
            startSimThread();
        }

        configurePathPlanner();

        // Configure heading PID on the shared drive-at-angle request
        driveAtAngleRequest.HeadingController =
                new PhoenixPIDController(
                        config.getKPRotationController(),
                        config.getKIRotationController(),
                        config.getKDRotationController());
        driveAtAngleRequest.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
        driveAtAngleRequest
                .withDeadband(
                        config.getLinearSpeedAt12Volts().baseUnitMagnitude()
                                * config.getAimDeadband())
                .withRotationalDeadband(
                        config.getAngularSpeedAt12Volts().baseUnitMagnitude()
                                * config.getAimDeadband())
                .withMaxAbsRotationalRate(config.getAngularSpeedAt12Volts());

        this.register();

        // Eight motors' worth of per-signal config calls, and only an optimisation: on a dead bus
        // it is boot latency for nothing, so it is skipped once the CAN config budget is spent.
        if (!CanConfigBudget.exhausted()) {
            optimizeBusUtilization();
        }
        // Must come after optimizeBusUtilization(), which silences the CANcoder signals it wants.
        alignment = new SwerveAlignment(getModules(), config);

        var modules = getModules();
        moduleCurrentSignals = new BaseStatusSignal[modules.length * 4];
        moduleCurrentKeys = new String[modules.length * 4];
        moduleConnectedKeys = new String[modules.length * 2];
        for (int i = 0; i < modules.length; i++) {
            moduleCurrentSignals[4 * i] = modules[i].getDriveMotor().getStatorCurrent(false);
            moduleCurrentSignals[4 * i + 1] = modules[i].getDriveMotor().getSupplyCurrent(false);
            moduleCurrentSignals[4 * i + 2] = modules[i].getSteerMotor().getStatorCurrent(false);
            moduleCurrentSignals[4 * i + 3] = modules[i].getSteerMotor().getSupplyCurrent(false);

            // Built once so the 10 Hz tick does no string concatenation.
            String module =
                    i < SwerveAlignment.MODULE_NAMES.length
                            ? SwerveAlignment.MODULE_NAMES[i]
                            : "Module" + i;
            moduleCurrentKeys[4 * i] = CURRENTS_PREFIX + module + "/DriveStatorCurrent";
            moduleCurrentKeys[4 * i + 1] = CURRENTS_PREFIX + module + "/DriveSupplyCurrent";
            moduleCurrentKeys[4 * i + 2] = CURRENTS_PREFIX + module + "/SteerStatorCurrent";
            moduleCurrentKeys[4 * i + 3] = CURRENTS_PREFIX + module + "/SteerSupplyCurrent";
            moduleConnectedKeys[2 * i] = "Swerve/Modules/" + module + "/DriveConnected";
            moduleConnectedKeys[2 * i + 1] = "Swerve/Modules/" + module + "/SteerConnected";
        }
        // optimizeBusUtilization() above turned off every signal nobody had given a rate, these
        // included, so without this the currents (and the DriveConnected/SteerConnected checks and
        // the battery logger's swerve share built on them) read frozen values on the robot.
        if (!CanConfigBudget.exhausted()) {
            BaseStatusSignal.setUpdateFrequencyForAll(MODULE_CURRENT_HZ, moduleCurrentSignals);
        }

        Telemetry.print(getName() + " Subsystem Initialized");
    }

    // --------------------------------------------------------------------------------
    // Odometry → inputs → fusion
    // --------------------------------------------------------------------------------

    /**
     * CTRE's telemetry callback, on the odometry thread. Copies the sample and returns; no logging,
     * no allocation beyond the copy, no locks beyond the queue's.
     */
    private void queueOdometrySample(SwerveDriveState state) {
        SwerveModulePosition[] positions = new SwerveModulePosition[state.ModulePositions.length];
        for (int i = 0; i < positions.length; i++) {
            positions[i] = state.ModulePositions[i].copy();
        }
        OdometrySample sample = new OdometrySample(state.Timestamp, state.RawHeading, positions);
        if (!odometryQueue.offer(sample)) {
            odometryQueue.poll();
            odometryQueue.offer(sample);
        }
    }

    /** Fills {@link #inputs} from the hardware. Not called in replay. */
    private void readHardware() {
        // Phoenix's clock and the HAL's monotonic clock are read back to back, so the difference
        // converts a CTRE sample time into the Timer.getTimestamp() base the pose estimator and
        // vision share. Not Timer.getTimestamp() itself: AdvantageKit pins that to the start of the
        // cycle, which would shift every sample by however far into the loop this line runs.
        double phoenixMinusWpilib =
                Utils.getCurrentTimeSeconds() - HALUtil.getMonotonicTime() * 1e-6;

        int n = odometryQueue.size();
        double[] times = new double[n];
        double[] yaws = new double[n];
        double[] distances = new double[n * 4];
        double[] angles = new double[n * 4];
        int s = 0;
        OdometrySample sample;
        while (s < n && (sample = odometryQueue.poll()) != null) {
            SwerveModulePosition[] positions = sample.positions();
            Rotation2d heading = sample.rawHeading();
            if (sim != null) {
                sim.truth.update(heading, positions);
                heading = sim.corruptHeading(heading, sample.ctreTimestamp());
                positions = sim.corruptPositions(positions);
            }
            times[s] = sample.ctreTimestamp() - phoenixMinusWpilib;
            yaws[s] = heading.getRadians();
            for (int m = 0; m < 4 && m < positions.length; m++) {
                distances[s * 4 + m] = positions[m].distance;
                angles[s * 4 + m] = positions[m].angle.getRadians();
            }
            s++;
        }
        if (s < n) {
            times = java.util.Arrays.copyOf(times, s);
            yaws = java.util.Arrays.copyOf(yaws, s);
            distances = java.util.Arrays.copyOf(distances, s * 4);
            angles = java.util.Arrays.copyOf(angles, s * 4);
        }
        inputs.odometryTimestamps = times;
        inputs.odometryYawRadians = yaws;
        inputs.odometryModuleDistances = distances;
        inputs.odometryModuleAngles = angles;

        // A copy: getState() hands back the object the odometry thread keeps writing, and
        // AdvantageKit serializes these arrays after this line.
        SwerveDriveState state = getStateCopy();
        inputs.ctrePose = state.Pose;
        inputs.robotVelocity = state.Velocity;
        inputs.moduleVelocities = state.ModuleVelocities;
        inputs.moduleTargets = state.ModuleTargets;
        inputs.odometryPeriodSeconds = state.OdometryPeriod;
        inputs.successfulDaqs = state.SuccessfulDaqs;
        inputs.failedDaqs = state.FailedDaqs;

        var status = BaseStatusSignal.refreshAll(yawRateSignal, pitchSignal, rollSignal);
        inputs.gyroConnected = status.isOK();
        inputs.gyroYawRateRadPerSec = Units.degreesToRadians(yawRateSignal.getValueAsDouble());
        inputs.gyroPitchDegrees = pitchSignal.getValueAsDouble();
        inputs.gyroRollDegrees = rollSignal.getValueAsDouble();
    }

    private final SwerveModulePosition[] scratchPositions = {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
    };

    /** Integrates this loop's logged odometry samples into the fused pose. */
    private void integrateOdometry() {
        for (int s = 0; s < inputs.odometryTimestamps.length; s++) {
            for (int m = 0; m < 4; m++) {
                scratchPositions[m] =
                        new SwerveModulePosition(
                                inputs.odometryModuleDistances[s * 4 + m],
                                Rotation2d.fromRadians(inputs.odometryModuleAngles[s * 4 + m]));
            }
            poseFusion.addOdometry(
                    inputs.odometryTimestamps[s],
                    Rotation2d.fromRadians(inputs.odometryYawRadians[s]),
                    scratchPositions,
                    s == inputs.odometryTimestamps.length - 1);
        }
    }

    /**
     * Keeps CTRE's internal estimate on the fused pose. CTRE's field-centric and facing-angle
     * requests rotate by its own heading, so if it drifted from the fused heading the robot would
     * drive a few degrees off what the pilot asked. Reset only on a visible disagreement, not every
     * loop.
     */
    private void syncCtrePose() {
        if (!Constants.hasHardware()) {
            return;
        }
        Pose2d fused = poseFusion.getPose();
        Pose2d ctre = inputs.ctrePose;
        boolean translationOff = fused.getTranslation().getDistance(ctre.getTranslation()) > 0.02;
        boolean headingOff =
                Math.abs(fused.getRotation().minus(ctre.getRotation()).getDegrees()) > 0.5;
        if (translationOff || headingOff) {
            super.resetPose(fused);
        }
    }

    // --------------------------------------------------------------------------------
    // Periodic
    // --------------------------------------------------------------------------------

    /**
     * Reads the drivetrain into {@link #inputs}, records them, and integrates this loop's odometry
     * into the fused pose. Called by {@code Robot} at the top of every loop, before vision and
     * before the scheduler, so everything this loop sees the same pose.
     */
    public void updateInputs() {
        if (Constants.hasHardware()) {
            readHardware();
        }
        Logger.processInputs("Swerve", inputs);
        integrateOdometry();
        if (sim != null) {
            poseFusion.setTruth(Optional.of(sim.truth.getPose()));
        }
    }

    @Override
    public void periodic() {
        systemState = handleStateTransition();
        applyStates();

        Telemetry.log("Swerve/WantedState", wantedState.toString());
        Telemetry.log("Swerve/SystemState", systemState.toString());
        Telemetry.log("Swerve/CurrentCommand", getCurrentCommandName());
        Telemetry.log("Swerve/TeleopVelocityCoefficient", getTeleopVelocityCoefficient());
        Telemetry.log("Swerve/RotationVelocityCoefficient", getRotationVelocityCoefficient());
        Telemetry.log("Swerve/State/Pose", getRobotPose());
        Telemetry.log("Swerve/State/TargetStates", inputs.moduleTargets);
        Telemetry.log("Swerve/State/MeasuredStates", inputs.moduleVelocities);
        Telemetry.log("Swerve/State/MeasuredSpeeds", inputs.robotVelocity);
        Telemetry.log("Swerve/OdometrySamplesThisLoop", inputs.odometryTimestamps.length);
        if (Constants.hasHardware()) {
            logBatteryUsage();
            alignment.log();
        }

        pigeonAlert.set(!inputs.gyroConnected);
    }

    /**
     * Called by {@code Robot} after vision has been fused, so CTRE is re-aligned with the pose this
     * loop's measurements produced.
     */
    public void afterVision() {
        syncCtrePose();
        poseFusion.log();
    }

    // ── Currents ──────────────────────────────────────────────────────────────

    /** Log key prefix for the drivetrain's current telemetry. */
    private static final String CURRENTS_PREFIX = "Swerve/Currents/";

    /** Drive and steer stator and supply current signals for every module, refreshed together. */
    private BaseStatusSignal[] moduleCurrentSignals = new BaseStatusSignal[0];

    /** Log key for each entry of {@link #moduleCurrentSignals}, in the same order. */
    private String[] moduleCurrentKeys = new String[0];

    /**
     * {@code Swerve/Modules/<name>/DriveConnected} and {@code .../SteerConnected}, two per module.
     *
     * <p>From the offseason branch: in the 2026-09-19 Chezy P8 match the CANivore bus died
     * mid-teleop and the eight drivetrain motors left no direct evidence at all; the failure had to
     * be read off their currents at 10 Hz. These ride the same 10 Hz refresh as the currents.
     */
    private String[] moduleConnectedKeys = new String[0];

    private double driveSupplyCurrent;
    private double steerSupplyCurrent;

    /**
     * Reports drive and steer supply current to the battery logger every loop and logs per-module
     * and summed currents at 10 Hz. Diagnostic outputs only; nothing decides on them, so they are
     * read straight from the signals rather than through {@link SwerveInputs}.
     */
    protected void logBatteryUsage() {
        if (Telemetry.slowLogThisLoop() && moduleCurrentSignals.length > 0) {
            BaseStatusSignal.refreshAll(moduleCurrentSignals);
            double driveStatorCurrent = 0;
            double steerStatorCurrent = 0;
            driveSupplyCurrent = 0;
            steerSupplyCurrent = 0;
            for (int i = 0; i < moduleCurrentSignals.length; i += 4) {
                double driveStator = moduleCurrentSignals[i].getValueAsDouble();
                double driveSupply = moduleCurrentSignals[i + 1].getValueAsDouble();
                double steerStator = moduleCurrentSignals[i + 2].getValueAsDouble();
                double steerSupply = moduleCurrentSignals[i + 3].getValueAsDouble();

                driveStatorCurrent += driveStator;
                driveSupplyCurrent += driveSupply;
                steerStatorCurrent += steerStator;
                steerSupplyCurrent += steerSupply;

                Telemetry.log(moduleCurrentKeys[i], driveStator);
                Telemetry.log(moduleCurrentKeys[i + 1], driveSupply);
                Telemetry.log(moduleCurrentKeys[i + 2], steerStator);
                Telemetry.log(moduleCurrentKeys[i + 3], steerSupply);
                // A signal whose refresh failed is a motor that did not answer.
                Telemetry.log(
                        moduleConnectedKeys[i / 2], moduleCurrentSignals[i].getStatus().isOK());
                Telemetry.log(
                        moduleConnectedKeys[i / 2 + 1],
                        moduleCurrentSignals[i + 2].getStatus().isOK());
            }
            Telemetry.log(CURRENTS_PREFIX + "DriveStatorCurrent", driveStatorCurrent);
            Telemetry.log(CURRENTS_PREFIX + "SteerStatorCurrent", steerStatorCurrent);
            Telemetry.log(CURRENTS_PREFIX + "DriveSupplyCurrent", driveSupplyCurrent);
            Telemetry.log(CURRENTS_PREFIX + "SteerSupplyCurrent", steerSupplyCurrent);
        }
        Robot.getBatteryLogger().reportCurrentUsage("Mechanisms/SwerveSteer", steerSupplyCurrent);
        Robot.getBatteryLogger().reportCurrentUsage("Mechanisms/SwerveDrive", driveSupplyCurrent);
    }

    // -----------------------------------------------------------------------
    // State machine
    // -----------------------------------------------------------------------

    protected String getCurrentCommandName() {
        Command currentCommand = this.getCurrentCommand();
        if (currentCommand != null) {
            return currentCommand.getName();
        }
        return "none";
    }

    private SystemState handleStateTransition() {
        return switch (wantedState) {
            case TELEOP_DRIVE -> SystemState.TELEOP_DRIVE;
            case PILOT_AIM_AT_TARGET -> SystemState.PILOT_AIM_AT_TARGET;
            case X_BRAKE -> SystemState.X_BRAKE;
            case IDLE -> SystemState.IDLE;
        };
    }

    private void applyStates() {
        switch (systemState) {
            default:
            case IDLE:
                setControl(idleRequest);
                break;
            case PILOT_AIM_AT_TARGET:
                // Teleop aiming and every auto launch (Auton.launch() runs LAUNCH_WITH_SQUEEZE,
                // which lands here), so both get the feedforward.
                var params = ShotCalculator.getInstance().getParameters();
                ChassisVelocities joystick = calculateSpeedsBasedOnJoystickInputs();
                aimTarget = params.driveAngle();
                double aimFeedforward = AimFeedforward.of(params.driveAngularVelocity());
                setControl(
                        driveAtAngleRequest
                                .withVelocityX(joystick.vx)
                                .withVelocityY(joystick.vy)
                                .withTargetDirection(aimTarget)
                                .withTargetRateFeedforward(aimFeedforward));
                Telemetry.log(
                        "Swerve/Aim/TargetRateFeedforward",
                        Units.radiansToDegrees(aimFeedforward),
                        "deg/s");
                Telemetry.log(
                        "Swerve/Aim/HeadingErrorDeg",
                        Units.radiansToDegrees(getAimHeadingErrorRadians()),
                        "degrees");
                break;
            case TELEOP_DRIVE:
                setControl(fieldCentricDrive.withVelocity(calculateSpeedsBasedOnJoystickInputs()));
                break;
            case X_BRAKE:
                setControl(xBrake);
                break;
        }
    }

    private ChassisVelocities calculateSpeedsBasedOnJoystickInputs() {
        Optional<Alliance> alliance = MatchState.getAlliance();
        if (alliance.isEmpty()) {
            return new ChassisVelocities(0, 0, 0);
        }
        boolean blue = alliance.get() == Alliance.BLUE;

        double xMagnitude = Robot.getPilot().getDriveFwdPositive();
        double yMagnitude = Robot.getPilot().getDriveLeftPositive();
        double angularMagnitude = Robot.getPilot().getDriveCCWPositive();

        double xVelocity = (blue ? xMagnitude : -xMagnitude) * teleopVelocityCoefficient;
        double yVelocity = (blue ? yMagnitude : -yMagnitude) * teleopVelocityCoefficient;
        double angularVelocity = angularMagnitude * rotationVelocityCoefficient;

        Rotation2d heading = getRobotPose().getRotation();
        Rotation2d skewCompensationFactor =
                Rotation2d.fromRadians(
                        getCurrentRobotChassisSpeeds().omega * SKEW_COMPENSATION_SCALAR);

        return new ChassisVelocities(xVelocity, yVelocity, angularVelocity)
                .toRobotRelative(heading)
                .toFieldRelative(heading.plus(skewCompensationFactor));
    }

    // --------------------------------------------------------------------------------
    // Pose Methods
    // --------------------------------------------------------------------------------

    /**
     * The robot's fused pose: odometry plus every enabled vision source, integrated on the main
     * loop from logged inputs.
     *
     * @return the robot pose on the field
     */
    public Pose2d getRobotPose() {
        return poseFusion.getPose();
    }

    /**
     * Get the robot's pose at a specific timestamp using interpolation.
     *
     * @param timestampSeconds the timestamp to sample at, {@code Timer.getTimestamp()} base
     * @return the interpolated pose, or current pose if the timestamp is not in the buffer
     */
    public Pose2d getPoseAtTimestamp(double timestampSeconds) {
        return poseFusion.sampleAt(timestampSeconds).orElse(getRobotPose());
    }

    /** Resets every pose track (and, in sim, the simulated robot) to {@code pose}. */
    @Override
    public void resetPose(Pose2d pose) {
        poseFusion.resetPose(pose);
        if (sim != null) {
            sim.resetTruth(pose);
        }
        if (Constants.hasHardware()) {
            super.resetPose(pose);
        }
    }

    // --------------------------------------------------------------------------------
    // Zone Triggers
    // --------------------------------------------------------------------------------

    public Trigger inXzone(double minXmeter, double maxXmeter) {
        return new Trigger(
                () -> Util.inRange(() -> getRobotPose().getX(), () -> minXmeter, () -> maxXmeter));
    }

    public Trigger inYzone(double minYmeter, double maxYmeter) {
        return new Trigger(
                () -> Util.inRange(() -> getRobotPose().getY(), () -> minYmeter, () -> maxYmeter));
    }

    /**
     * Whether the robot is in an X band of the field, flipped for red.
     *
     * @param minXmeter the minimum X coordinate in meters
     * @param maxXmeter the maximum X coordinate in meters
     * @return the Trigger
     */
    public Trigger inXzoneAlliance(double minXmeter, double maxXmeter) {
        return new Trigger(
                () ->
                        Util.inRange(
                                FieldHelpers.flipXifRed(getRobotPose().getX()),
                                minXmeter,
                                maxXmeter));
    }

    /**
     * Whether the robot is in a Y band of the field, flipped for red.
     *
     * @param minYmeter the minimum Y coordinate in meters
     * @param maxYmeter the maximum Y coordinate in meters
     * @return the Trigger
     */
    public Trigger inYzoneAlliance(double minYmeter, double maxYmeter) {
        return new Trigger(
                () ->
                        Util.inRange(
                                FieldHelpers.flipYifRed(getRobotPose().getY()),
                                minYmeter,
                                maxYmeter));
    }

    private static final double NEUTRAL_DEPTH_METERS = Units.inchesToMeters(283.0);
    private static final double NEUTRAL_LENGTH_METERS = Units.inchesToMeters(317.7);
    private static final double ENEMY_ALLIANCE_DEPTH_METERS = Units.inchesToMeters(180.0);

    private static final Rectangle2d NEUTRAL_ZONE =
            new Rectangle2d(
                    new Translation2d(
                            Field.fieldLength / 2.0 - NEUTRAL_DEPTH_METERS / 2.0,
                            Field.fieldWidth / 2.0 - NEUTRAL_LENGTH_METERS / 2.0),
                    new Translation2d(
                            Field.fieldLength / 2.0 + NEUTRAL_DEPTH_METERS / 2.0,
                            Field.fieldWidth / 2.0 + NEUTRAL_LENGTH_METERS / 2.0));

    private static final Rectangle2d ENEMY_ALLIANCE_ZONE =
            new Rectangle2d(
                    new Translation2d(Field.fieldLength - ENEMY_ALLIANCE_DEPTH_METERS, 0),
                    new Translation2d(Field.fieldLength, Field.fieldWidth));

    /** Returns {@code true} when the robot is inside the neutral zone. */
    public boolean isInNeutralZone() {
        return NEUTRAL_ZONE.contains(getRobotPose().getTranslation());
    }

    /**
     * Returns {@code true} when the robot is inside the opposing alliance's zone (pose X is flipped
     * for red so the same rectangle works for both alliances).
     */
    public boolean isInEnemyAllianceZone() {
        Pose2d pose = getRobotPose();
        return ENEMY_ALLIANCE_ZONE.contains(
                new Translation2d(FieldHelpers.flipXifRed(pose.getX()), pose.getY()));
    }

    public Trigger inNeutralZone() {
        return new Trigger(this::isInNeutralZone);
    }

    public Trigger inEnemyAllianceZone() {
        return new Trigger(this::isInEnemyAllianceZone);
    }

    public Trigger inFieldRight() {
        final double halfWidth = Units.feetToMeters(27.0) / 2.0;
        return new Trigger(() -> getRobotPose().getY() < halfWidth);
    }

    public Trigger inFieldLeft() {
        final double halfWidth = Units.feetToMeters(27.0) / 2.0;
        return new Trigger(() -> getRobotPose().getY() >= halfWidth);
    }

    // --------------------------------------------------------------------------------
    // Speed Checks
    // --------------------------------------------------------------------------------

    /**
     * Returns {@code true} if the robot is moving faster than the threshold.
     *
     * @param thresholdSpeed the speed threshold in meters per second
     * @return {@code true} if the current linear speed exceeds the threshold
     */
    public boolean isGoingTooFast(double thresholdSpeed) {
        ChassisVelocities speeds = getCurrentRobotChassisSpeeds();
        return Math.hypot(speeds.vx, speeds.vy) > thresholdSpeed;
    }

    public Trigger overSpeedTrigger(double thresholdSpeed) {
        return new Trigger(() -> isGoingTooFast(thresholdSpeed));
    }

    /**
     * Robot-relative chassis velocity measured from the modules this loop (a logged input).
     *
     * @return the current robot chassis velocity
     */
    public ChassisVelocities getCurrentRobotChassisSpeeds() {
        return inputs.robotVelocity;
    }

    /** Gyro yaw rate this loop, rad/s (a logged input). */
    public double getYawRateRadPerSec() {
        return inputs.gyroYawRateRadPerSec;
    }

    // --------------------------------------------------------------------------------
    // Reorientation Methods
    // --------------------------------------------------------------------------------

    private void applyReorient(double angleDegrees) {
        Pose2d pose = getRobotPose();
        resetPose(new Pose2d(pose.getX(), pose.getY(), Rotation2d.fromDegrees(angleDegrees)));
    }

    protected Command reorient(double angleDegrees) {
        return runOnce(() -> applyReorient(angleDegrees));
    }

    public Command reorientForward() {
        return reorient(0).withName("reorientForward");
    }

    public Command reorientLeft() {
        return reorient(90).withName("reorientLeft");
    }

    public Command reorientBack() {
        return reorient(180).withName("reorientBack");
    }

    public Command reorientRight() {
        return reorient(270).withName("reorientRight");
    }

    protected double getClosestCardinal() {
        double heading = getRotation().getRadians();
        if (heading > -Math.PI / 4 && heading <= Math.PI / 4) {
            return 0;
        } else if (heading > Math.PI / 4 && heading <= 3 * Math.PI / 4) {
            return 90;
        } else if (heading > 3 * Math.PI / 4 || heading <= -3 * Math.PI / 4) {
            return 180;
        } else {
            return 270;
        }
    }

    protected Command cardinalReorient() {
        return runOnce(() -> applyReorient(getClosestCardinal()));
    }

    public boolean frontClosestToAngle(double angleDegrees) {
        double heading = getRotation().getDegrees();
        double flippedHeading = heading > 0 ? heading - 180 : heading + 180;
        return getRotationDifference(heading, angleDegrees)
                < getRotationDifference(flippedHeading, angleDegrees);
    }

    /** Shortest absolute difference between two angles, degrees (0-180). */
    public double getRotationDifference(double angle1, double angle2) {
        double diff = Math.abs(angle1 - angle2) % 360;
        return diff > 180 ? 360 - diff : diff;
    }

    Rotation2d getRotation() {
        return getRobotPose().getRotation();
    }

    double getRotationRadians() {
        return getRobotPose().getRotation().getRadians();
    }

    // --------------------------------------------------------------------------------
    // Request Methods
    // --------------------------------------------------------------------------------

    /** Sets a control request continuously; ignores disable so commands are continuous. */
    Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get())).ignoringDisable(true);
    }

    public void setWantedState(WantedState state) {
        this.wantedState = state;
    }

    /**
     * Fused heading minus the heading the aim request is holding, wrapped to (-pi, pi]. Read from
     * the fused pose, so it replays.
     *
     * @return the heading error in radians, or NaN when the drivetrain is not aiming
     */
    public double getAimHeadingErrorRadians() {
        if (systemState != SystemState.PILOT_AIM_AT_TARGET) {
            return Double.NaN;
        }
        return getRotation().minus(aimTarget).getRadians();
    }

    /** Whether the drivetrain is holding a heading on the shot solution this loop. */
    public boolean isAiming() {
        return systemState == SystemState.PILOT_AIM_AT_TARGET;
    }

    public boolean isAtDesiredRotation() {
        return isAtDesiredRotation(Units.degreesToRadians(10.0));
    }

    /**
     * Whether the robot heading is within tolerance of the aim target. Computed from the fused pose
     * rather than CTRE's heading controller so it replays.
     *
     * @param toleranceRadians the allowed heading error in radians
     * @return {@code true} while aiming and inside the tolerance
     */
    public boolean isAtDesiredRotation(double toleranceRadians) {
        if (systemState != SystemState.PILOT_AIM_AT_TARGET) {
            return false;
        }
        return Math.abs(getRotation().minus(aimTarget).getRadians()) < toleranceRadians;
    }

    // --------------------------------------------------------------------------------
    // Path Planner Configuration
    // --------------------------------------------------------------------------------

    private void configurePathPlanner() {
        // Seed robot to in front of blue hub (Paths will change this starting position)
        resetPose(
                new Pose2d(
                        Field.getBlueHubCenter().getX() - 2,
                        Field.getBlueHubCenter().getY(),
                        Rotation2d.fromDegrees(0)));

        // PathPlanner's own view of what it is doing, as AdvantageKit outputs: the path being
        // followed, where the controller wants the robot, and the pose it is using. Called from
        // PathPlanner's commands, so on the main loop.
        PathPlannerLogging.setLogActivePathCallback(
                poses ->
                        Logger.recordOutput(
                                "PathPlanner/ActivePath", poses.toArray(new Pose2d[0])));
        PathPlannerLogging.setLogTargetPoseCallback(
                pose -> Logger.recordOutput("PathPlanner/TargetPose", pose));
        PathPlannerLogging.setLogCurrentPoseCallback(
                pose -> Logger.recordOutput("PathPlanner/CurrentPose", pose));

        try {
            var ppConfig = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                    this::getRobotPose,
                    this::resetPose,
                    this::getCurrentRobotChassisSpeeds,
                    (speeds, feedforwards) ->
                            setControl(
                                    autoRequest
                                            .withVelocity(
                                                    speeds.discretize(RobotLoop.periodSeconds()))
                                            .withWheelForceFeedforwardsX(
                                                    feedforwards.robotRelativeForcesX())
                                            .withWheelForceFeedforwardsY(
                                                    feedforwards.robotRelativeForcesY())),
                    new PPHolonomicDriveController(
                            new PIDConstants(4, 0, 0), new PIDConstants(3, 0, 0)),
                    ppConfig,
                    () -> MatchState.getAlliance().orElse(Alliance.BLUE) == Alliance.RED,
                    this);
        } catch (Exception ex) {
            DriverStationErrors.reportError(
                    "Failed to load PathPlanner config and configure AutoBuilder",
                    ex.getStackTrace());
        }
    }

    // --------------------------------------------------------------------------------
    // Simulation
    // --------------------------------------------------------------------------------

    /**
     * Deliberate odometry error in simulation, so the comparison between vision sources has
     * something to correct. The truth pose is the uncorrupted odometry.
     */
    public static final class SimErrors {
        /** Measured wheel distance over true distance. 1.02 is a worn-tread two percent. */
        public static final double WHEEL_SCALE = 1.02;

        /** Gyro drift, degrees per second. */
        public static final double GYRO_DRIFT_DEG_PER_SEC = 0.05;
    }

    private final class SimState {
        final SwerveDriveOdometry truth;
        private double driftOffsetDeg = 0;
        private double firstSampleTime = Double.NaN;

        SimState() {
            SwerveModulePosition[] zero = new SwerveModulePosition[4];
            for (int i = 0; i < 4; i++) {
                zero[i] = new SwerveModulePosition();
            }
            truth = new SwerveDriveOdometry(getKinematics(), Rotation2d.kZero, zero);
        }

        Rotation2d corruptHeading(Rotation2d heading, double t) {
            if (Double.isNaN(firstSampleTime)) {
                firstSampleTime = t;
            }
            return heading.plus(
                    Rotation2d.fromDegrees(
                            driftOffsetDeg
                                    + (t - firstSampleTime) * SimErrors.GYRO_DRIFT_DEG_PER_SEC));
        }

        SwerveModulePosition[] corruptPositions(SwerveModulePosition[] positions) {
            SwerveModulePosition[] out = new SwerveModulePosition[positions.length];
            for (int i = 0; i < positions.length; i++) {
                out[i] =
                        new SwerveModulePosition(
                                positions[i].distance * SimErrors.WHEEL_SCALE, positions[i].angle);
            }
            return out;
        }

        void resetTruth(Pose2d pose) {
            // The truth teleports with the robot; the drift keeps accumulating from here.
            truth.resetPose(pose);
        }
    }

    /** Simulation state, or {@code null} off the desktop sim. */
    private SimState sim = null;

    private void startSimThread() {
        sim = new SimState();
        final double[] lastTime = {Timer.getTimestamp()};
        /* Run simulation at a faster rate so PID gains behave more reasonably */
        simNotifier =
                new Notifier(
                        () -> {
                            double now = Timer.getTimestamp();
                            double dt = now - lastTime[0];
                            lastTime[0] = now;
                            updateSimState(dt, RobotController.getBatteryVoltage());
                        });
        simNotifier.startPeriodic(config.getSimLoopPeriod());
    }

    /** Truth pose in simulation, empty elsewhere. */
    public Optional<Pose2d> getSimTruthPose() {
        return sim == null ? Optional.empty() : Optional.of(sim.truth.getPose());
    }
}
