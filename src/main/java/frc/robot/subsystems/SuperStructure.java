package frc.robot.subsystems;

import frc.rebuilt.ShotCalculator;
import frc.robot.Robot;
import frc.robot.subsystems.fuelIntake.FuelIntake;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.indexerBed.IndexerBed;
import frc.robot.subsystems.indexerTower.IndexerTower;
import frc.robot.subsystems.intakeExtension.IntakeExtension;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.swerve.Swerve;
import frc.spectrumLib.telemetry.Telemetry;
import frc.spectrumLib.util.Util;
import java.util.function.BooleanSupplier;
import lombok.Getter;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.InstantCommand;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.command2.button.Trigger;
import org.wpilib.system.Timer;

public class SuperStructure extends SubsystemBase {

    @Getter private final Swerve swerve;
    @Getter private final FuelIntake fuelIntake;
    @Getter private final IntakeExtension intakeExtension;
    @Getter private final IndexerTower indexerTower;
    @Getter private final IndexerBed indexerBed;
    @Getter private final Launcher launcher;
    @Getter private final Hood hood;

    private static final double REGULAR_TELEOP_TRANSLATION_COEFFICIENT = 1.0;
    private static final double SHOOTING_TELEOP_TRANSLATION_COEFFICIENT = 0.1;

    public enum WantedSuperState {
        IDLE,
        INTAKE_FUEL,
        TRACK_TARGET,
        LAUNCH_WITH_SQUEEZE,
        LAUNCH_WITH_SQUEEZE_WITH_NO_DELAY,
        LAUNCH_WITHOUT_SQUEEZE,
        LAUNCH_WITH_BRAKE,
        AUTON_TRACK_TARGET,
        AUTON_INTAKE_FUEL,
        UNJAM,
        EJECT,
        FORCE_HOME,
    }

    public enum CurrentSuperState {
        IDLE,
        INTAKE_FUEL,
        TRACK_TARGET,
        LAUNCH_WITH_SQUEEZE,
        LAUNCH_WITH_SQUEEZE_WITH_NO_DELAY,
        LAUNCH_WITHOUT_SQUEEZE,
        LAUNCH_WITH_BRAKE,
        AUTON_IDLE,
        AUTON_TRACK_TARGET,
        AUTON_INTAKE_FUEL,
        UNJAM,
        EJECT,
        FORCE_HOME,
    }

    @Getter private WantedSuperState wantedSuperState = WantedSuperState.IDLE;
    @Getter private CurrentSuperState currentSuperState = CurrentSuperState.IDLE;
    private CurrentSuperState previousSuperState = CurrentSuperState.IDLE;

    public SuperStructure(
            Swerve swerve,
            FuelIntake fuelIntake,
            IntakeExtension intakeExtension,
            IndexerTower indexerTower,
            IndexerBed indexerBed,
            Launcher launcher,
            Hood hood) {
        this.swerve = swerve;
        this.fuelIntake = fuelIntake;
        this.intakeExtension = intakeExtension;
        this.indexerTower = indexerTower;
        this.indexerBed = indexerBed;
        this.launcher = launcher;
        this.hood = hood;
        this.shotGate =
                new ShotGate(
                        ShotGate.Tolerances.fm(launcher.getConfig().getOnTargetToleranceRPM()));
    }

    private final Timer intakeSqueezeTimer = new Timer();
    private final double secondsToSqueeze = 1.0;

    private static boolean isSqueezeState(CurrentSuperState state) {
        return state == CurrentSuperState.LAUNCH_WITH_SQUEEZE;
    }

    @Override
    public void periodic() {
        currentSuperState = handleStateTransitions();

        // Restart the squeeze timer exactly once when first entering a squeeze state
        if (isSqueezeState(currentSuperState) && !isSqueezeState(previousSuperState)) {
            intakeSqueezeTimer.restart();
        }

        // Before applyStates(): the launch states read the gate to pick the indexer states.
        updateShotGate();

        applyStates();

        previousSuperState = currentSuperState;

        Telemetry.log("SuperStructure/WantedSuperState", wantedSuperState.toString());
        Telemetry.log("SuperStructure/CurrentSuperState", currentSuperState.toString());
        Telemetry.log(
                "SuperStructure/IntakeSqueezeTimerElapsed", intakeSqueezeTimer.get(), "seconds");
    }

    // -- Shot readiness gate ------------------------------------------------------
    //
    // FM on 2026 main fed the moment RT was pulled, whether or not the flywheel was up to speed,
    // the
    // hood had arrived or the robot had finished turning to the hub. Now the indexers hold fuel
    // short of the flywheel until ShotGate says the shot is ready, and time out to feeding anyway
    // after a second, so the gate can delay a shot but never stop one. Operator Y bypasses it.

    private final ShotGate shotGate;

    /** This loop's gate decision. */
    @Getter private ShotGate.Decision shotDecision = ShotGate.Decision.IDLE;

    /** Operator hold to feed regardless of the gate, for a bad sensor or a deliberate dump. */
    private BooleanSupplier feedOverride = () -> false;

    /**
     * Vision estimate age past which range stops voting (offseason ddd564a). Without an accepted
     * estimate the distance is whatever odometry was seeded with, so the range check is not
     * measuring anything; the mechanism checks still gate the shot. Generous on purpose: shorter
     * only makes the gate more permissive after a short dropout.
     */
    private static final double POSE_TRUST_TIMEOUT_SECONDS = 3.0;

    /**
     * Held-fuel indexer states. The tower (brake mode, the stage that lifts fuel into the flywheel)
     * stops, which holds what is in it; the bed keeps slow-indexing as it does while intaking, so
     * the tower is full when the gate opens.
     */
    private static final IndexerTower.WantedState TOWER_HOLD_STATE = IndexerTower.WantedState.OFF;

    private static final IndexerBed.WantedState BED_HOLD_STATE = IndexerBed.WantedState.SLOW_INDEX;

    private long launchingLoops = 0;
    private long heldLoops = 0;
    private long volleys = 0;
    private boolean feedingLastLoop = false;

    /**
     * Sets the control that bypasses the gate while held. Polled every loop (a supplier, not a
     * scheduled command, so there is no interrupt edge case).
     *
     * @param override true while the operator wants the gate ignored
     */
    public void setFeedOverride(BooleanSupplier override) {
        this.feedOverride = override;
    }

    /** Whether the current state is one that feeds the flywheel. */
    public boolean isLaunching() {
        return isLaunchState(currentSuperState);
    }

    private static boolean isLaunchState(CurrentSuperState state) {
        return state == CurrentSuperState.LAUNCH_WITH_SQUEEZE
                || state == CurrentSuperState.LAUNCH_WITH_SQUEEZE_WITH_NO_DELAY
                || state == CurrentSuperState.LAUNCH_WITHOUT_SQUEEZE
                || state == CurrentSuperState.LAUNCH_WITH_BRAKE;
    }

    /** Whether fuel is being fed into the flywheel this loop. */
    public boolean isFeeding() {
        return shotDecision.feed();
    }

    /**
     * Reads the shot's state from logged inputs only (flywheel and hood motor inputs, the fused
     * pose, vision's estimate age, the operator's gamepad) and runs the gate.
     */
    private void updateShotGate() {
        boolean launching = isLaunching();
        double shotTargetRpm = launcher.getShotTargetRPM();
        // The shot solution is already computed this loop whenever the launcher is on a shot.
        boolean onShot = shotTargetRpm > 0;
        ShotCalculator.ShootingParameters params =
                onShot ? ShotCalculator.getInstance().getParameters() : null;

        double sinceVision = Robot.getVision().secondsSinceFusedEstimate();
        boolean poseTrusted = sinceVision <= POSE_TRUST_TIMEOUT_SECONDS;
        boolean feedShot = params != null && isRobotInFeedZone();
        double aimTolerance =
                params == null
                        ? ShotGate.MAX_AIM_TOLERANCE_RAD
                        : ShotGate.aimToleranceRad(params.distance(), feedShot);
        boolean checkAim = swerve.isAiming();
        double headingError = swerve.getAimHeadingErrorRadians();
        boolean override = feedOverride.getAsBoolean();

        shotDecision =
                shotGate.update(
                        new ShotGate.Inputs(
                                launching,
                                override,
                                Timer.getTimestamp(),
                                launcher.getVelocityRPM(),
                                shotTargetRpm,
                                hood.getPositionDegrees(),
                                hood.getVelocityRPM() * 6.0, // mechanism RPM to deg/s
                                hood.getShotTargetDegrees(),
                                checkAim,
                                headingError,
                                aimTolerance,
                                poseTrusted,
                                params != null && params.isValid()));

        boolean feeding = shotDecision.feed();
        if (launching) {
            launchingLoops++;
            if (!feeding) {
                heldLoops++;
            }
        }
        if (feeding && !feedingLastLoop) {
            volleys++;
        }
        feedingLastLoop = feeding;

        Telemetry.logDash("Shot/Ready", shotDecision.ready());
        Telemetry.logDash("Shot/BlockedReason", shotDecision.blocker().label);
        Telemetry.logDash("Shot/Feeding", feeding);
        Telemetry.logDash("Shot/FeedReason", shotDecision.reason().label);
        Telemetry.logDash("Shot/Override", override);
        Telemetry.log("Shot/SecondsHeld", shotDecision.secondsHeld(), "seconds");
        Telemetry.log("Shot/FlywheelErrorRPM", launcher.getVelocityRPM() - shotTargetRpm, "RPM");
        Telemetry.log(
                "Shot/HoodErrorDeg",
                hood.getPositionDegrees() - hood.getShotTargetDegrees(),
                "degrees");
        Telemetry.log("Shot/HeadingErrorDeg", Math.toDegrees(headingError), "degrees");
        Telemetry.log("Shot/AimToleranceDeg", Math.toDegrees(aimTolerance), "degrees");
        Telemetry.log("Shot/CheckAim", checkAim);
        Telemetry.log("Shot/PoseTrusted", poseTrusted);
        Telemetry.log("Shot/InRange", params != null && params.isValid());
        Telemetry.log("Shot/LaunchingLoops", launchingLoops);
        Telemetry.log("Shot/HeldLoops", heldLoops);
        Telemetry.log("Shot/Volleys", volleys);
    }

    /**
     * The indexers for a launch: full speed while the gate allows it, fuel held short of the
     * flywheel otherwise.
     */
    private void applyGatedFeed() {
        if (shotDecision.feed()) {
            indexerTower.setWantedState(IndexerTower.WantedState.INDEX_MAX);
            indexerBed.setWantedState(IndexerBed.WantedState.INDEX_MAX);
        } else {
            indexerTower.setWantedState(TOWER_HOLD_STATE);
            indexerBed.setWantedState(BED_HOLD_STATE);
        }
    }

    private CurrentSuperState handleStateTransitions() {
        return switch (wantedSuperState) {
            case IDLE ->
                    Util.autoMode.getAsBoolean() || Util.disabled.getAsBoolean()
                            ? CurrentSuperState.AUTON_IDLE
                            : CurrentSuperState.IDLE;
            case INTAKE_FUEL -> CurrentSuperState.INTAKE_FUEL;
            case TRACK_TARGET -> CurrentSuperState.TRACK_TARGET;
            case LAUNCH_WITH_SQUEEZE -> CurrentSuperState.LAUNCH_WITH_SQUEEZE;
            case LAUNCH_WITH_SQUEEZE_WITH_NO_DELAY ->
                    CurrentSuperState.LAUNCH_WITH_SQUEEZE_WITH_NO_DELAY;
            case LAUNCH_WITHOUT_SQUEEZE -> CurrentSuperState.LAUNCH_WITHOUT_SQUEEZE;
            case LAUNCH_WITH_BRAKE -> CurrentSuperState.LAUNCH_WITH_BRAKE;
            case AUTON_TRACK_TARGET -> CurrentSuperState.AUTON_TRACK_TARGET;
            case AUTON_INTAKE_FUEL -> CurrentSuperState.AUTON_INTAKE_FUEL;
            case UNJAM -> CurrentSuperState.UNJAM;
            case EJECT -> CurrentSuperState.EJECT;
            case FORCE_HOME -> CurrentSuperState.FORCE_HOME;
        };
    }

    private void applyStates() {
        switch (currentSuperState) {
            case IDLE:
                applyIdle();
                break;
            case INTAKE_FUEL:
                intakeFuel();
                break;
            case TRACK_TARGET:
                trackTarget();
                break;
            case LAUNCH_WITH_SQUEEZE:
                launchWithSqueeze();
                break;
            case LAUNCH_WITH_SQUEEZE_WITH_NO_DELAY:
                launchWithSqueezeWithNoDelay();
                break;
            case LAUNCH_WITHOUT_SQUEEZE:
                launchWithoutSqueeze();
                break;
            case LAUNCH_WITH_BRAKE:
                launchWithBrake();
                break;
            case AUTON_IDLE:
                applyAutonIdle();
                break;
            case AUTON_INTAKE_FUEL:
                autonIntakeFuel();
                break;
            case AUTON_TRACK_TARGET:
                autonTrackTarget();
                break;
            case UNJAM:
                unjam();
                break;
            case EJECT:
                eject();
                break;
            case FORCE_HOME:
                forceHome();
                break;
        }
    }

    // ── State methods ──────────────────────────────────────────────────────────

    private void applyIdle() {
        swerve.setWantedState(Swerve.WantedState.TELEOP_DRIVE);
        swerve.setTeleopVelocityCoefficient(REGULAR_TELEOP_TRANSLATION_COEFFICIENT);
        fuelIntake.setWantedState(FuelIntake.WantedState.NEUTRAL);
        indexerTower.setWantedState(IndexerTower.WantedState.OFF);
        indexerBed.setWantedState(IndexerBed.WantedState.OFF);
        intakeExtension.setWantedState(IntakeExtension.WantedState.STOPPED);
        launcher.setWantedState(Launcher.WantedState.IDLE_PREP);
        hood.setWantedState(Hood.WantedState.HOME);
    }

    private void intakeFuel() {
        swerve.setWantedState(Swerve.WantedState.TELEOP_DRIVE);
        swerve.setTeleopVelocityCoefficient(REGULAR_TELEOP_TRANSLATION_COEFFICIENT);
        fuelIntake.setWantedState(FuelIntake.WantedState.INTAKE);
        indexerTower.setWantedState(IndexerTower.WantedState.OFF);
        indexerBed.setWantedState(IndexerBed.WantedState.SLOW_INDEX);
        intakeExtension.setWantedState(IntakeExtension.WantedState.FULL_EXTEND);
        launcher.setWantedState(Launcher.WantedState.IDLE_PREP);
        hood.setWantedState(Hood.WantedState.HOME);
    }

    private void trackTarget() {
        swerve.setWantedState(Swerve.WantedState.PILOT_AIM_AT_TARGET);
        swerve.setTeleopVelocityCoefficient(REGULAR_TELEOP_TRANSLATION_COEFFICIENT);
        fuelIntake.setWantedState(FuelIntake.WantedState.NEUTRAL);
        indexerTower.setWantedState(IndexerTower.WantedState.OFF);
        indexerBed.setWantedState(IndexerBed.WantedState.OFF);
        intakeExtension.setWantedState(IntakeExtension.WantedState.CONDITIONAL_EXTEND);
        launcher.setWantedState(Launcher.WantedState.AIM_AT_TARGET);
        hood.setWantedState(Hood.WantedState.AIM_AT_TARGET);
    }

    private void launchWithSqueeze() {
        swerve.setWantedState(Swerve.WantedState.PILOT_AIM_AT_TARGET);
        swerve.setTeleopVelocityCoefficient(SHOOTING_TELEOP_TRANSLATION_COEFFICIENT);
        fuelIntake.setWantedState(FuelIntake.WantedState.INTAKE);
        applyGatedFeed();
        launcher.setWantedState(Launcher.WantedState.AIM_AT_TARGET);
        hood.setWantedState(Hood.WantedState.AIM_AT_TARGET);

        if (intakeSqueezeTimer.hasElapsed(secondsToSqueeze)) {
            intakeExtension.setWantedState(IntakeExtension.WantedState.SLOW_CLOSE);
            intakeSqueezeTimer.stop();
        } else {
            intakeExtension.setWantedState(IntakeExtension.WantedState.FULL_EXTEND);
        }
    }

    private void launchWithSqueezeWithNoDelay() {
        swerve.setWantedState(Swerve.WantedState.PILOT_AIM_AT_TARGET);
        swerve.setTeleopVelocityCoefficient(SHOOTING_TELEOP_TRANSLATION_COEFFICIENT);
        fuelIntake.setWantedState(FuelIntake.WantedState.INTAKE);
        applyGatedFeed();
        intakeExtension.setWantedState(IntakeExtension.WantedState.SLOW_CLOSE);
        launcher.setWantedState(Launcher.WantedState.AIM_AT_TARGET);
        hood.setWantedState(Hood.WantedState.AIM_AT_TARGET);
    }

    private void launchWithoutSqueeze() {
        swerve.setWantedState(Swerve.WantedState.PILOT_AIM_AT_TARGET);
        swerve.setTeleopVelocityCoefficient(SHOOTING_TELEOP_TRANSLATION_COEFFICIENT);
        fuelIntake.setWantedState(FuelIntake.WantedState.INTAKE);
        applyGatedFeed();
        intakeExtension.setWantedState(IntakeExtension.WantedState.FULL_EXTEND);
        launcher.setWantedState(Launcher.WantedState.AIM_AT_TARGET);
        hood.setWantedState(Hood.WantedState.AIM_AT_TARGET);
    }

    private void launchWithBrake() {
        swerve.setWantedState(Swerve.WantedState.X_BRAKE);
        fuelIntake.setWantedState(FuelIntake.WantedState.SLOW_INTAKE);
        applyGatedFeed();
        intakeExtension.setWantedState(IntakeExtension.WantedState.FULL_EXTEND);
        launcher.setWantedState(Launcher.WantedState.AIM_AT_TARGET);
        hood.setWantedState(Hood.WantedState.AIM_AT_TARGET);
    }

    private void applyAutonIdle() {
        swerve.setWantedState(Swerve.WantedState.IDLE);
        fuelIntake.setWantedState(FuelIntake.WantedState.NEUTRAL);
        indexerTower.setWantedState(IndexerTower.WantedState.OFF);
        indexerBed.setWantedState(IndexerBed.WantedState.OFF);
        intakeExtension.setWantedState(IntakeExtension.WantedState.STOPPED);
        launcher.setWantedState(Launcher.WantedState.IDLE_PREP);
        hood.setWantedState(Hood.WantedState.HOME);
    }

    private void autonIntakeFuel() {
        fuelIntake.setWantedState(FuelIntake.WantedState.INTAKE);
        indexerTower.setWantedState(IndexerTower.WantedState.OFF);
        indexerBed.setWantedState(IndexerBed.WantedState.SLOW_INDEX);
        intakeExtension.setWantedState(IntakeExtension.WantedState.FULL_EXTEND);
        launcher.setWantedState(Launcher.WantedState.IDLE_PREP);
        hood.setWantedState(Hood.WantedState.HOME);
    }

    private void autonTrackTarget() {
        fuelIntake.setWantedState(FuelIntake.WantedState.NEUTRAL);
        indexerTower.setWantedState(IndexerTower.WantedState.OFF);
        indexerBed.setWantedState(IndexerBed.WantedState.OFF);
        intakeExtension.setWantedState(IntakeExtension.WantedState.CONDITIONAL_EXTEND);
        launcher.setWantedState(Launcher.WantedState.AIM_AT_TARGET);
        hood.setWantedState(Hood.WantedState.AIM_AT_TARGET);
    }

    private void unjam() {
        swerve.setWantedState(Swerve.WantedState.TELEOP_DRIVE);
        swerve.setTeleopVelocityCoefficient(REGULAR_TELEOP_TRANSLATION_COEFFICIENT);
        fuelIntake.setWantedState(FuelIntake.WantedState.NEUTRAL);
        indexerTower.setWantedState(IndexerTower.WantedState.UNJAM);
        indexerBed.setWantedState(IndexerBed.WantedState.UNJAM);
        intakeExtension.setWantedState(IntakeExtension.WantedState.CONDITIONAL_EXTEND);
        launcher.setWantedState(Launcher.WantedState.OFF);
        hood.setWantedState(Hood.WantedState.HOME);
    }

    private void eject() {
        swerve.setWantedState(Swerve.WantedState.TELEOP_DRIVE);
        swerve.setTeleopVelocityCoefficient(REGULAR_TELEOP_TRANSLATION_COEFFICIENT);
        fuelIntake.setWantedState(FuelIntake.WantedState.OUTTAKE);
        indexerTower.setWantedState(IndexerTower.WantedState.UNJAM);
        indexerBed.setWantedState(IndexerBed.WantedState.UNJAM);
        intakeExtension.setWantedState(IntakeExtension.WantedState.CONDITIONAL_EXTEND);
        launcher.setWantedState(Launcher.WantedState.OFF);
        hood.setWantedState(Hood.WantedState.HOME);
    }

    private void forceHome() {
        swerve.setWantedState(Swerve.WantedState.TELEOP_DRIVE);
        swerve.setTeleopVelocityCoefficient(REGULAR_TELEOP_TRANSLATION_COEFFICIENT);
        fuelIntake.setWantedState(FuelIntake.WantedState.NEUTRAL);
        indexerTower.setWantedState(IndexerTower.WantedState.OFF);
        indexerBed.setWantedState(IndexerBed.WantedState.OFF);
        intakeExtension.setWantedState(IntakeExtension.WantedState.FULL_RETRACT);
        launcher.setWantedState(Launcher.WantedState.OFF);
        hood.setWantedState(Hood.WantedState.HOME);
    }

    // ── Public API ─────────────────────────────────────────────────────────────

    public Command coastMechanisms() {
        return Commands.runOnce(
                        () -> {
                            intakeExtension.setBrakeMode(false);
                            hood.setBrakeMode(false);
                        })
                .ignoringDisable(true);
    }

    public Command brakeMechanisms() {
        return Commands.runOnce(
                        () -> {
                            intakeExtension.setBrakeMode(true);
                            hood.setBrakeMode(true);
                        })
                .ignoringDisable(true);
    }

    // Allocation-free boolean checks — use these in per-loop code (e.g. ShotCalculator).
    public boolean isRobotInNeutralZone() {
        return swerve.isInNeutralZone();
    }

    public boolean isRobotInEnemyZone() {
        return swerve.isInEnemyAllianceZone();
    }

    public boolean isRobotInFeedZone() {
        return isRobotInEnemyZone() || isRobotInNeutralZone();
    }

    public boolean isRobotInScoreZone() {
        return !isRobotInFeedZone();
    }

    // Trigger factories — use these for binding-time composition only.
    public Trigger robotInNeutralZone() {
        return new Trigger(this::isRobotInNeutralZone);
    }

    public Trigger robotInEnemyZone() {
        return new Trigger(this::isRobotInEnemyZone);
    }

    public Trigger robotInFeedZone() {
        return new Trigger(this::isRobotInFeedZone);
    }

    public Trigger robotInScoreZone() {
        return new Trigger(this::isRobotInScoreZone);
    }

    public void setWantedSuperState(WantedSuperState state) {
        this.wantedSuperState = state;
    }

    public Command setStateCommand(WantedSuperState state) {
        return new InstantCommand(() -> setWantedSuperState(state));
    }
}
