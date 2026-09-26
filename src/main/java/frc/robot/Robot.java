package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.CANBus.CANBusStatus;
import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import frc.rebuilt.ShiftHelpers;
import frc.rebuilt.ShotCalculator;
import frc.rebuilt.ShotCalculator.SetShot;
import frc.robot.auton.Auton;
import frc.robot.configs.FM2026;
import frc.robot.configs.PHOTON2026;
import frc.robot.configs.PM2026;
import frc.robot.operator.Operator;
import frc.robot.operator.Operator.OperatorConfig;
import frc.robot.pilot.Pilot;
import frc.robot.pilot.Pilot.PilotConfig;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.WantedSuperState;
import frc.robot.subsystems.fuelIntake.FuelIntake;
import frc.robot.subsystems.fuelIntake.FuelIntake.FuelIntakeConfig;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.Hood.HoodConfig;
import frc.robot.subsystems.indexerBed.IndexerBed;
import frc.robot.subsystems.indexerBed.IndexerBed.IndexerBedConfig;
import frc.robot.subsystems.indexerTower.IndexerTower;
import frc.robot.subsystems.indexerTower.IndexerTower.IndexerTowerConfig;
import frc.robot.subsystems.intakeExtension.IntakeExtension;
import frc.robot.subsystems.intakeExtension.IntakeExtension.IntakeExtensionConfig;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.launcher.Launcher.LauncherConfig;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConfig;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.Vision.VisionConfig;
import frc.spectrumLib.framework.RobotLoop;
import frc.spectrumLib.framework.SpectrumRobot;
import frc.spectrumLib.hardware.CanBuses;
import frc.spectrumLib.hardware.CanConfigBudget;
import frc.spectrumLib.hardware.Rio;
import frc.spectrumLib.telemetry.Alert;
import frc.spectrumLib.telemetry.BatteryLogger;
import frc.spectrumLib.telemetry.DashboardReceiver;
import frc.spectrumLib.telemetry.LogStorage;
import frc.spectrumLib.telemetry.SystemLoadMonitor;
import frc.spectrumLib.telemetry.Telemetry;
import frc.spectrumLib.telemetry.Telemetry.PrintPriority;
import frc.spectrumLib.util.BackgroundSampler;
import frc.spectrumLib.util.CrashTracker;
import frc.spectrumLib.util.Util;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;
import lombok.Getter;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.command2.Commands;
import org.wpilib.driverstation.Alert.Level;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.net.WebServer;
import org.wpilib.smartdashboard.Field2d;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.system.Filesystem;
import org.wpilib.system.RobotController;
import org.wpilib.system.Timer;
import org.wpilib.units.Units;

/**
 * FM (2026 "Final Machine") on SystemCore, competing as 8515.
 *
 * <p>The robot code is FM's from 2026-Spectrum {@code main}; the loop structure, telemetry, CAN
 * health logging and pre-match checks are from the {@code 2026-offseason-bot} branch; logging and
 * replay are AdvantageKit. See README.md.
 *
 * <h2>Loop order</h2>
 *
 * <ol>
 *   <li>{@link Swerve#updateInputs()}: drivetrain inputs recorded, odometry integrated.
 *   <li>{@link Vision#periodic()}: every pose source read, gated, logged and fused.
 *   <li>The command scheduler: subsystems (SuperStructure, mechanisms, Swerve's drive request),
 *       then commands.
 * </ol>
 */
public class Robot extends SpectrumRobot {
    @Getter private static RobotSim robotSim;
    @Getter private static Config config;
    @Getter private static final Field2d field2d = new Field2d();
    public static boolean autonWarmedUp = false;

    /** CPU, loop period, GC and memory on the dashboard, with alerts when they stay bad. */
    private SystemLoadMonitor systemLoad;

    public static class Config {
        public final SwerveConfig swerve = new SwerveConfig();
        public final PilotConfig pilot = new PilotConfig();
        public final OperatorConfig operator = new OperatorConfig();
        public final FuelIntakeConfig fuelIntake = new FuelIntakeConfig();
        public final IntakeExtensionConfig intakeExtension = new IntakeExtensionConfig();
        public final IndexerTowerConfig indexerTower = new IndexerTowerConfig();
        public final IndexerBedConfig indexerBed = new IndexerBedConfig();
        public final LauncherConfig launcher = new LauncherConfig();
        public final HoodConfig hood = new HoodConfig();
        public final VisionConfig vision = new VisionConfig();
    }

    @Getter private static Swerve swerve;
    @Getter private static FuelIntake fuelIntake;
    @Getter private static IntakeExtension intakeExtension;
    @Getter private static IndexerTower indexerTower;
    @Getter private static IndexerBed indexerBed;
    @Getter private static Operator operator;
    @Getter private static Pilot pilot;
    @Getter private static Launcher launcher;
    @Getter private static Hood hood;
    @Getter private static Vision vision;
    @Getter private static Leds leds;
    @Getter private static Auton auton;
    @Getter private static SuperStructure superStructure;
    @Getter private static BatteryLogger batteryLogger;

    /** Every CAN bus this layout uses ({@link CanBuses#inUse()}), by log label. */
    private static final java.util.Map<String, CANBus> canBuses = new java.util.LinkedHashMap<>();

    public Robot() {
        super(Constants.LOOP_PERIOD_SECONDS);
        SystemLoadMonitor.threadCensus("jvm");
        startLogging();
        SystemLoadMonitor.threadCensus("logging");

        /*
         * Phoenix otherwise starts writing .hoot signal logs for every CAN device a second after
         * the first enable. Nobody replays them, and on 2026-09-05 they were a large share of the
         * roboRIO's SD card. AdvantageKit's log is the record now. SignalLogger.start() still works
         * for a deliberate capture.
         */
        SignalLogger.enableAutoLogging(false);

        Telemetry.start(PrintPriority.NORMAL);

        try {
            Telemetry.print("--- Robot Init Starting ---");

            // Set up the config. On SystemCore Rio.id reads the controller's serial number.
            switch (Rio.id) {
                case PHOTON2026:
                    config = new PHOTON2026();
                    break;
                case PM_2026:
                    config = new PM2026();
                    break;
                default: // FM, SIM and UNKNOWN
                    config = new FM2026();
                    break;
            }

            CanBuses.inUse().forEach((label, name) -> canBuses.put(label, CanBuses.forName(name)));
            SystemLoadMonitor.threadCensus("canbuses");
            if (Constants.currentMode == Constants.Mode.REAL
                    && CanBuses.USE_CANIVORE
                    && !canBuses.get("CANivore").getStatus().Status.isOK()) {
                // No CANivore at all (not plugged in, or canivore-usb not installed on the
                // SystemCore): do not spend a minute timing out every device on it.
                CanConfigBudget.exhaust("CANivore '" + CanBuses.CANIVORE + "' not found");
            } else if (Constants.currentMode == Constants.Mode.REAL && noCanTraffic()) {
                // Buses up but nothing talking on any of them (a bench controller, or every
                // device unpowered): the same minute of timeouts, so the same shortcut.
                CanConfigBudget.exhaust("no CAN traffic on any bus");
            }
            if (Constants.hasHardware()) {
                canStatus =
                        BackgroundSampler.every(
                                1.0,
                                () -> {
                                    var status =
                                            new java.util.LinkedHashMap<String, CANBusStatus>();
                                    canBuses.forEach(
                                            (label, bus) -> status.put(label, bus.getStatus()));
                                    return status;
                                });
            }

            pilot = new Pilot(config.pilot);
            operator = new Operator(config.operator);
            SystemLoadMonitor.threadCensus("controllers");

            batteryLogger = new BatteryLogger();
            SystemLoadMonitor.threadCensus("batteryLogger");

            swerve = new Swerve(config.swerve);
            SystemLoadMonitor.threadCensus("swerve");

            intakeExtension = new IntakeExtension(config.intakeExtension);
            SystemLoadMonitor.threadCensus("intakeExtension");

            fuelIntake = new FuelIntake(config.fuelIntake);

            hood = new Hood(config.hood);

            launcher = new Launcher(config.launcher);
            SystemLoadMonitor.threadCensus("hood+launcher");

            indexerTower = new IndexerTower(config.indexerTower);

            indexerBed = new IndexerBed(config.indexerBed);
            SystemLoadMonitor.threadCensus("indexers");

            superStructure =
                    new SuperStructure(
                            swerve,
                            fuelIntake,
                            intakeExtension,
                            indexerTower,
                            indexerBed,
                            launcher,
                            hood);

            auton = new Auton(superStructure);
            SystemLoadMonitor.threadCensus("auton");
            vision = new Vision(config.vision, swerve);
            SystemLoadMonitor.threadCensus("vision");
            systemLoad = new SystemLoadMonitor();
            SystemLoadMonitor.threadCensus("systemLoad");

            if (Constants.currentMode == Constants.Mode.SIM) {
                robotSim = new RobotSim(superStructure);
                configureSimBindings();
            }

            configureBindings();

            batteryLogger.setEnabled(true);

            SmartDashboard.putData("Field2d", field2d);
            // The commands running now, for Elastic's Scheduler widget (as in the offseason code).
            SmartDashboard.putData("Scheduler", CommandScheduler.getInstance());
            // Build the ShotCalculator now so its Hub Model Chooser is on the dashboard before
            // enabling.
            ShotCalculator.getInstance();
            if (Constants.hasHardware()) {
                WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
            }

            if (Constants.currentMode == Constants.Mode.REAL) {
                setMainThreadPriority(Constants.MAIN_THREAD_RT_PRIORITY);
            }
            Telemetry.print("--- Robot Init Complete ---");

        } catch (Throwable t) {
            // intercept error and log it
            CrashTracker.logThrowableCrash(t);
            throw t;
        }

        // Offseason: 6.0 V rather than 4.6 V. A worse sag than Chezy's 8.8 V would take the
        // controller below its own reset point before a 4.6 V brownout ever tripped, and a reboot
        // mid-match is far worse than a second of disabled outputs.
        //
        // SystemCore image on the alpha-6 bench unit rejects this (HAL -1098, "handle"), which
        // crashed the constructor. The brownout API is treated as optional: if setting it
        // fails, the brownout telemetry that reads it is skipped too.
        try {
            RobotController.setBrownoutVoltage(Units.Volts.of(6.0));
            RobotController.isBrownedOut();
            powerApiAvailable = true;
        } catch (RuntimeException e) {
            powerApiAvailable = false;
            Telemetry.print(
                    "Brownout voltage API unavailable on this controller: " + e.getMessage(),
                    PrintPriority.HIGH);
        }
    }

    private final Alert mainThreadPriorityAlert = new Alert("", Level.MEDIUM);

    /** Makes the calling (main) thread real-time; see {@link Constants#MAIN_THREAD_RT_PRIORITY}. */
    @SuppressWarnings("deprecation") // WPILib deprecates it as a warning to know what you are doing
    private void setMainThreadPriority(int priority) {
        if (priority <= 0) {
            return;
        }
        // Read it back rather than trusting the return value: on alpha-6 SystemCore it reports
        // false even when the priority took (2026-09-23 bench unit: false, then 15).
        org.wpilib.system.Threads.setCurrentThreadPriority(priority);
        int now = org.wpilib.system.Threads.getCurrentThreadPriority();
        boolean ok = now == priority;
        Logger.recordMetadata("MainThreadPriority", Integer.toString(now));
        Telemetry.print(
                "Main thread real-time priority "
                        + priority
                        + (ok ? " set" : " FAILED")
                        + " (now "
                        + now
                        + ")");
        mainThreadPriorityAlert.setText("Main thread real-time priority " + priority + " failed");
        mainThreadPriorityAlert.set(!ok);
    }

    /**
     * Battery voltage below which a disabled robot raises the swap-the-battery alert. CALIBRATE
     * against a meter: SystemCore's reading has been reported ~1.5 V low (SystemcoreTesting #306).
     */
    private static final double LOW_BATTERY_VOLTS = 11.8;

    private final Alert lowBatteryAlert = new Alert("", Level.MEDIUM);
    private final org.wpilib.math.filter.Debouncer lowBatteryDebounce =
            new org.wpilib.math.filter.Debouncer(3.0);

    /**
     * Before a match (disabled), a battery that stays under {@link #LOW_BATTERY_VOLTS} for three
     * seconds raises an alert. Disabled only: under load while driving the voltage sags by design.
     */
    private void checkBatteryBeforeMatch() {
        double volts = RuntimeInputs.batteryVoltage();
        boolean low =
                lowBatteryDebounce.calculate(
                        RobotState.isDisabled() && volts > 1 && volts < LOW_BATTERY_VOLTS);
        lowBatteryAlert.setText(String.format("Battery low: %.1f V, swap before the match", volts));
        lowBatteryAlert.set(low);
    }

    /** Whether the HAL's brownout API works here (it did not on the 2026-09 SystemCore). */
    private boolean powerApiAvailable = false;

    /**
     * Configures AdvantageKit: metadata, where the log goes, and in replay where it comes from.
     * Must run before anything records an input.
     */
    private void startLogging() {
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("Robot", "FM-2026-SystemCore");
        Logger.recordMetadata("TeamNumber", "8515");
        Logger.recordMetadata("LoopPeriodSeconds", Double.toString(Constants.LOOP_PERIOD_SECONDS));
        Logger.recordMetadata("RuntimeMode", Constants.currentMode.name());
        // Which robot config this controller selected (by serial number); replay always builds
        // FM2026, so a log from another robot says so here.
        Logger.recordMetadata("RobotIdentity", Rio.id.name());
        Logger.recordMetadata("ControllerSerial", Rio.serial());
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        Logger.recordMetadata(
                "GitDirty",
                switch (BuildConstants.DIRTY) {
                    case 0 -> "All changes committed";
                    case 1 -> "Uncommitted changes";
                    default -> "Unknown";
                });

        switch (Constants.currentMode) {
            case REAL:
                // A USB stick if one is inserted (AdvantageKit's default /U/logs), otherwise the
                // SystemCore's own storage. Without a stick /U/logs cannot be opened and the
                // match is not logged at all.
                Logger.addDataReceiver(new WPILOGWriter(LogStorage.chooseFolder()));
                Logger.recordMetadata("LogFolder", LogStorage.folder());
                // Dashboard keys to NT at ~50 Hz; the log file keeps everything, every cycle.
                Logger.addDataReceiver(new DashboardReceiver(new NT4Publisher(), ntEveryN()));
                break;
            case SIM:
                // Log to NT for AdvantageScope and to ./logs for replay practice.
                Logger.addDataReceiver(new WPILOGWriter("logs"));
                Logger.addDataReceiver(new DashboardReceiver(new NT4Publisher(), ntEveryN()));
                break;
            case REPLAY:
                setUseTiming(false); // Run as fast as possible
                String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(
                        new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
                break;
        }
        Logger.start();
        if (Constants.currentMode == Constants.Mode.REAL) {
            LogStorage.start();
        }

        // Dashboard keys: what the Elastic layout reads goes to NetworkTables; everything else
        // stays in the log unless the mirror switch is on (default on in the simulator).
        Telemetry.startDashboard(Constants.currentMode == Constants.Mode.SIM);
        try {
            var layout =
                    java.nio.file.Files.readString(
                            new java.io.File(Filesystem.getDeployDirectory(), "elastic-layout.json")
                                    .toPath());
            Telemetry.addDashboardKeysFromElasticLayout(layout);
        } catch (java.io.IOException e) {
            Telemetry.print("No elastic-layout.json in deploy; only logDash keys go to NT");
        }
    }

    /** Cycles per NetworkTables publish, for about 50 Hz whatever the loop rate. */
    private static int ntEveryN() {
        return (int) Math.max(1, Math.round(0.02 / Constants.LOOP_PERIOD_SECONDS));
    }

    public void configureBindings() {
        // LT alone → intake fuel; do nothing if RT is already held (RT+LT handled below)
        pilot.LT.onTrue(
                Commands.either(
                        superStructure.setStateCommand(WantedSuperState.INTAKE_FUEL),
                        Commands.none(),
                        pilot.RT.negate()));

        // RT alone → launch; do nothing if LT is already held (RT+LT handled below)
        pilot.RT.onTrue(
                Commands.either(
                        superStructure.setStateCommand(WantedSuperState.LAUNCH_WITH_SQUEEZE),
                        Commands.none(),
                        pilot.LT.negate()));

        // RT + LT both held → launch (intake stays extended; resolves to LAUNCH_WITHOUT_SQUEEZE)
        pilot.RT
                .and(pilot.LT)
                .onTrue(superStructure.setStateCommand(WantedSuperState.LAUNCH_WITHOUT_SQUEEZE));

        // LT released while RT still held → launch (no delay; resolves to
        // LAUNCH_WITH_SQUEEZE_WITH_NO_DELAY)
        pilot.LT.onFalse(
                Commands.either(
                        superStructure.setStateCommand(
                                WantedSuperState.LAUNCH_WITH_SQUEEZE_WITH_NO_DELAY),
                        Commands.none(),
                        pilot.RT));

        // RT released while LT still held → resume intaking
        pilot.RT.onFalse(
                Commands.either(
                        superStructure.setStateCommand(WantedSuperState.INTAKE_FUEL),
                        Commands.none(),
                        pilot.LT));

        // Both released → idle
        pilot.RT.or(pilot.LT).onFalse(superStructure.setStateCommand(WantedSuperState.IDLE));

        pilot.LT.and(pilot.LB).onTrue(superStructure.setStateCommand(WantedSuperState.EJECT));
        pilot.LT.and(pilot.LB).onFalse(superStructure.setStateCommand(WantedSuperState.IDLE));

        // Fixed shots: park at the spot, point the back at the hub, hold the chord. Hood and
        // flywheel from the spot's range, fed once they are there; no pose read, no aim checked.
        // See ShotCalculator.SetShot. Bound before the bare X and A below so that letting go of LB
        // first lands in what the buttons now say (track target, unjam), not IDLE.
        pilot.setShotTower_LB_A.onTrue(superStructure.setShotCommand(SetShot.TOWER));
        pilot.setShotLeftTrench_LB_X.onTrue(superStructure.setShotCommand(SetShot.LEFT_TRENCH));
        pilot.setShotRightTrench_LB_B.onTrue(superStructure.setShotCommand(SetShot.RIGHT_TRENCH));
        pilot.anySetShot.onFalse(superStructure.setStateCommand(WantedSuperState.IDLE));

        pilot.trackTarget_X.onTrue(superStructure.setStateCommand(WantedSuperState.TRACK_TARGET));
        pilot.trackTarget_X.onFalse(superStructure.setStateCommand(WantedSuperState.IDLE));

        pilot.unjam_A.onTrue(superStructure.setStateCommand(WantedSuperState.UNJAM));
        pilot.unjam_A.onFalse(superStructure.setStateCommand(WantedSuperState.IDLE));

        pilot.selectButton.onTrue(superStructure.setStateCommand(WantedSuperState.FORCE_HOME));
        pilot.selectButton.onFalse(superStructure.setStateCommand(WantedSuperState.IDLE));

        pilot.dPadUp.and(pilot.LB).onTrue(swerve.reorientForward());
        pilot.dPadLeft.and(pilot.LB).onTrue(swerve.reorientLeft());
        pilot.dPadDown.and(pilot.LB).onTrue(swerve.reorientBack());
        pilot.dPadRight.and(pilot.LB).onTrue(swerve.reorientRight());

        Util.disabled.and(pilot.AButton).onTrue(superStructure.coastMechanisms());
        Util.disabled.and(pilot.BButton).onTrue(superStructure.brakeMechanisms());

        Util.disabled.and(operator.AButton).onTrue(superStructure.coastMechanisms());
        Util.disabled.and(operator.BButton).onTrue(superStructure.brakeMechanisms());

        // Re-origin the QuestNav onto the current robot pose (disabled or not).
        operator.LB.and(operator.XButton).onTrue(vision.resetQuestCommand());

        operator.LB
                .and(operator.YButton)
                .onTrue(
                        Commands.parallel(
                                        intakeExtension.resetCurrentPositionToMaxCommand(),
                                        operator.rumbleCommand(1, 0.5))
                                .ignoringDisable(true));

        operator.selectButton.onTrue(superStructure.setStateCommand(WantedSuperState.FORCE_HOME));
        operator.selectButton.onFalse(superStructure.setStateCommand(WantedSuperState.IDLE));

        // Held: feed regardless of the shot gate (SuperStructure / ShotGate).
        superStructure.setFeedOverride(operator.feedOverride_Y);

        operator.dPadDown.onTrue(ShotCalculator.decreaseHoodAngleOffset());
        operator.dPadUp.onTrue(ShotCalculator.increaseHoodAngleOffset());
        operator.dPadRight.onTrue(ShotCalculator.decreaseDriveAngleOffset());
        operator.dPadLeft.onTrue(ShotCalculator.increaseDriveAngleOffset());

        // Reset hub shift timer when enabling
        Util.teleop.onTrue(Commands.runOnce(ShiftHelpers::initialize));
        Util.autoMode.onTrue(Commands.runOnce(ShiftHelpers::initialize));
        Util.disabled.onTrue(Commands.runOnce(ShiftHelpers::initialize).ignoringDisable(true));

        // Auton Triggers
        Auton.autonIntake.onTrue(
                superStructure.setStateCommand(WantedSuperState.AUTON_INTAKE_FUEL));
        Auton.autonShotPrep.onTrue(
                superStructure.setStateCommand(WantedSuperState.AUTON_TRACK_TARGET));
        Auton.autonUnjam.onTrue(
                Commands.sequence(
                        superStructure.setStateCommand(WantedSuperState.UNJAM),
                        Commands.waitSeconds(1),
                        superStructure.setStateCommand(WantedSuperState.LAUNCH_WITH_SQUEEZE)));
        Auton.autonClearState.onTrue(superStructure.setStateCommand(WantedSuperState.IDLE));
    }

    public void configureSimBindings() {
        RobotSim.simLaunching().whileTrue(robotSim.ballSimLaunchFuel());
    }

    /* ROBOT PERIODIC  */
    @Override
    public void robotPeriodic() {
        RobotLoop.next();
        RuntimeInputs.update();
        Telemetry.periodic();
        Alert.periodic();
        LogStorage.periodic();
        systemLoad.periodic();

        // Latched here rather than in the mode inits so every mode is covered by the same check.
        // Never cleared: a disable does not un-run an auto.
        if (RobotState.isEnabled()) {
            hasBeenEnabled = true;
        }
        try {
            Telemetry.time("Scheduler/robotPeriodic");

            // Start every loop with an empty shot-solution cache so the first mechanism to ask
            // computes it from this loop's pose.
            ShotCalculator.getInstance().clearShootingParameters();

            // Drivetrain inputs and odometry, then vision, so this loop's pose correction lands
            // before any mechanism or command reads the pose.
            Telemetry.time("Scheduler/Swerve");
            swerve.updateInputs();
            Telemetry.timeEnd("Scheduler/Swerve");

            Telemetry.time("Scheduler/Vision");
            vision.periodic();
            Telemetry.timeEnd("Scheduler/Vision");

            Telemetry.time("Scheduler/CommandScheduler");
            CommandScheduler.getInstance().run();
            Telemetry.timeEnd("Scheduler/CommandScheduler");

            Telemetry.log("Match Data/MatchTime", MatchState.getMatchTime(), "seconds");
            var shift = ShiftHelpers.getOfficialShiftInfo();
            Telemetry.log("Match Data/InShift", shift.active());
            Telemetry.logDash("Match Data/TimeLeftInShift", shift.remainingTime(), "seconds");

            batteryLogger.setBatteryVoltage(RuntimeInputs.batteryVoltage());
            checkBatteryBeforeMatch();
            // Every loop: a brownout is a few hundred milliseconds.
            if (powerApiAvailable) {
                Telemetry.log("SystemStats/BrownedOut", RobotController.isBrownedOut());
            }
            // SystemCore reports no input current (the roboRIO did); the battery logger sums
            // the mechanisms instead.
            batteryLogger.logPower();

            logCanBusStatus();

            // For dashboards (AdvantageKit logs the pose itself every loop); 10 Hz is plenty.
            if (Telemetry.slowLogThisLoop()) {
                field2d.setRobotPose(swerve.getRobotPose());
            }

            Telemetry.timeEnd("Scheduler/robotPeriodic");
        } catch (Throwable t) {
            // intercept error and log it
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    /**
     * Whether every bus reads zero utilization twice, 0.3 s apart. Any powered Phoenix device
     * broadcasts status frames continuously, so a bus with devices on it is never at zero.
     */
    private static boolean noCanTraffic() {
        for (int attempt = 0; attempt < 2; attempt++) {
            try {
                Thread.sleep(300); // wall clock: the robot clock is frozen during init
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                return false;
            }
            for (CANBus bus : canBuses.values()) {
                if (bus.getStatus().BusUtilization > 0) {
                    return false;
                }
            }
        }
        return true;
    }

    /** Time of the last CAN bus status log. */
    private double lastCanStatusSeconds = Double.NEGATIVE_INFINITY;

    /**
     * Every bus's status by log label, read once a second on {@link BackgroundSampler}'s thread:
     * {@code getStatus()} can block for up to a millisecond, which on the real-time main thread is
     * a late loop. Null off the robot.
     */
    private static BackgroundSampler.Latest<java.util.Map<String, CANBusStatus>> canStatus;

    private long lastCanStatusSequence = 0;

    /** Logs CAN bus health once a second, plus the match identity, which the 2026 logs lacked. */
    private void logCanBusStatus() {
        double now = Timer.getTimestamp();
        if (now - lastCanStatusSeconds < 1.0) {
            return;
        }
        lastCanStatusSeconds = now;

        if (canStatus != null && canStatus.sequence() != lastCanStatusSequence) {
            lastCanStatusSequence = canStatus.sequence();
            canStatus.get().forEach(this::logOneCanBus);
        }

        Telemetry.log("CANConfig/BudgetSpentSeconds", CanConfigBudget.getSpentSeconds());
        Telemetry.log("CANConfig/FailedCalls", CanConfigBudget.getFailedCalls());
        Telemetry.logDash("CANConfig/BudgetExhausted", CanConfigBudget.exhausted());

        Telemetry.log("Match Data/EventName", MatchState.getEventName());
        Telemetry.log("Match Data/MatchType", MatchState.getMatchType().name());
        Telemetry.log("Match Data/MatchNumber", MatchState.getMatchNumber());
        Telemetry.log("Match Data/ReplayNumber", MatchState.getReplayNumber());
        Telemetry.log(
                "Match Data/Alliance", MatchState.getAlliance().map(Enum::name).orElse("NONE"));
        Telemetry.log("Match Data/Station", MatchState.getLocation().orElse(0));
        Telemetry.log("Match Data/FMSAttached", RobotState.isFMSAttached());
        if (powerApiAvailable) {
            Telemetry.log(
                    "SystemStats/BrownoutVoltage", RobotController.getBrownoutVoltage(), "volts");
        }
    }

    /**
     * Logs one bus's health under {@code <prefix>/}, status code included: every counter reads 0 on
     * a bus whose status read itself failed, which looks exactly like a healthy idle bus.
     */
    private void logOneCanBus(String prefix, CANBusStatus canInfo) {
        Telemetry.logDash(prefix + "/BusUtilization", canInfo.BusUtilization * 100, "%");
        Telemetry.log(prefix + "/BusOffCount", canInfo.BusOffCount);
        Telemetry.log(prefix + "/TxFullCount", canInfo.TxFullCount);
        Telemetry.log(prefix + "/ReceiveErrorCounter", canInfo.REC);
        Telemetry.log(prefix + "/TransmitErrorCounter", canInfo.TEC);
        Telemetry.log(prefix + "/Status", canInfo.Status.getName());
        Telemetry.logDash(prefix + "/StatusOK", canInfo.Status.isOK());
    }

    @Override
    public void disabledInit() {
        Telemetry.print("### Disabled Init Starting ### ");

        // Offseason: honoured only before the first enable -- see mayPlaceAtAutoStart().
        placeAtAutoStart = true;

        if (!autonWarmedUp) {
            Command autonStartCommand =
                    Commands.sequence(
                                    FollowPathCommand.warmupCommand(),
                                    // No PathfindingCommand warmup: FM's autos never pathfind, and
                                    // warming it up starts PathPlanner's AD* planning thread, which
                                    // then runs for the whole match and finishes on its own
                                    // schedule (not replayable). If pathfinding is added, use
                                    // AdvantageKit's LocalADStarAK so it replays.
                                    Commands.runOnce(
                                            () -> {
                                                Telemetry.log("Initialized", true);
                                                autonWarmedUp = true;
                                            }))
                            .ignoringDisable(true);
            CommandScheduler.getInstance().schedule(autonStartCommand);
        }

        Telemetry.print("### Disabled Init Complete ### ");
    }

    String autoName = "";

    /** Paths of the currently selected auto, kept so the start pose can be re-applied. */
    private List<PathPlannerPath> selectedAutoPaths = new ArrayList<>();

    /** Set whenever the robot should be put back on the selected auto's starting pose. */
    private boolean placeAtAutoStart = true;

    /**
     * True once the robot has been enabled at least once since boot. After that the pose
     * estimator's own output is always a better answer than the start of a path the robot has
     * already driven (offseason, from the 2026-09-19 Chezy practice match).
     */
    private boolean hasBeenEnabled = false;

    @Override
    public void disabledPeriodic() {
        String fullAutoName = auton.getAutonomousCommand().getName();
        boolean leftStart = !fullAutoName.endsWith(" - Right");
        List<PathPlannerPath> pathPlannerPaths = new ArrayList<>();

        // The alliance belongs in the reload key, not just the auto name: the red flip is applied
        // in the reload branch below.
        String selectionKey =
                fullAutoName + "|" + MatchState.getAlliance().map(Enum::name).orElse("NONE");

        if (fullAutoName.equals("Do Nothing")) {
            field2d.getObject("Auto Routine").setPoses(new ArrayList<>());
            if (!autoName.equals(selectionKey)) {
                logAutoSelection(fullAutoName, true, "");
            }
            autoName = selectionKey;
            autoFileMissingAlert.set(false);
            selectedAutoPaths = new ArrayList<>();
            return;
        }

        // Strip " - Left" / " - Right" suffix to get the base path name
        String baseAutoName = fullAutoName;
        if (baseAutoName.endsWith(" - Left") || baseAutoName.endsWith(" - Right")) {
            baseAutoName = baseAutoName.substring(0, baseAutoName.lastIndexOf(" - "));
        }

        if (!autoName.equals(selectionKey)) {
            autoName = selectionKey;
            Telemetry.log("Auton Warmed Up", false);
            boolean autoFileFound = AutoBuilder.getAllAutoNames().contains(baseAutoName);
            logAutoSelection(fullAutoName, autoFileFound, baseAutoName);
            selectedAutoPaths = new ArrayList<>();

            if (autoFileFound) {
                try {
                    pathPlannerPaths = PathPlannerAuto.getPathGroupFromAutoFile(baseAutoName);
                } catch (IOException | ParseException e) {
                    Telemetry.print("Could not load path planner paths");
                }

                // Flip the paths if on red alliance
                Optional<Alliance> alliance = MatchState.getAlliance();
                if (alliance.isPresent() && alliance.get() == Alliance.RED) {
                    pathPlannerPaths =
                            pathPlannerPaths.stream()
                                    .map(PathPlannerPath::flipPath)
                                    .collect(Collectors.toList());
                }

                // Mirror the paths if starting on the right
                if (!leftStart) {
                    pathPlannerPaths =
                            pathPlannerPaths.stream()
                                    .map(PathPlannerPath::mirrorPath)
                                    .collect(Collectors.toList());
                }

                if (!pathPlannerPaths.isEmpty()) {
                    selectedAutoPaths = pathPlannerPaths;
                    placeAtAutoStart = true;

                    // Warm up the starting path
                    Command warmUpPath =
                            Commands.sequence(
                                            AutoBuilder.followPath(pathPlannerPaths.get(0))
                                                    .withTimeout(0.5),
                                            Commands.runOnce(
                                                    () -> {
                                                        Telemetry.print(
                                                                "Auton Warmed Up",
                                                                PrintPriority.HIGH);
                                                        Telemetry.log("Auton Warmed Up", true);
                                                    }))
                                    .ignoringDisable(true);
                    CommandScheduler.getInstance().schedule(warmUpPath);
                } else {
                    Telemetry.print("Warning: No paths loaded for auto: " + baseAutoName);
                }

                // Convert path points to poses
                List<Pose2d> poses = new ArrayList<>();
                for (PathPlannerPath path : pathPlannerPaths) {
                    poses.addAll(
                            path.getAllPathPoints().stream()
                                    .map(
                                            point ->
                                                    new Pose2d(
                                                            point.position.getX(),
                                                            point.position.getY(),
                                                            Rotation2d.kZero))
                                    .collect(Collectors.toList()));
                }
                field2d.getObject("Auto Routine").setPoses(poses);
            } else {
                field2d.getObject("Auto Routine").setPoses(new ArrayList<>());
            }
        }

        if (placeAtAutoStart && !selectedAutoPaths.isEmpty()) {
            if (mayPlaceAtAutoStart()) {
                Pose2d start =
                        selectedAutoPaths.get(0).getStartingHolonomicPose().orElse(new Pose2d());
                swerve.resetPose(start);
            }
            placeAtAutoStart = false;
        }

        checkStartPose();
    }

    /** Only before the first enable on the robot; always in simulation. */
    private boolean mayPlaceAtAutoStart() {
        // RuntimeInputs, not Constants.currentMode: replaying a real match runs in REPLAY mode,
        // and must refuse the placement exactly as the robot did.
        return RuntimeInputs.isSimulation() || !hasBeenEnabled;
    }

    // -- Start pose check (offseason) ---------------------------------------------------------

    private static final double START_POSE_ALERT_METERS = 0.5;
    private static final double START_HEADING_ALERT_DEG = 10.0;
    private static final double START_POSE_ALERT_HOLD_SECONDS = 1.0;

    private double startPoseErrorSinceSeconds = Double.NaN;
    private final Alert startPoseAlert = new Alert("", Level.HIGH);
    private final Alert autoFileMissingAlert = new Alert("", Level.HIGH);
    private final Alert startPoseUnverifiedAlert =
            new Alert(
                    "Start pose UNVERIFIED - no Limelight has seeded the pose, so the robot cannot"
                            + " tell whether it is on the auto's start. Get a Limelight on two or"
                            + " more tags before enabling.",
                    Level.HIGH);

    private void logAutoSelection(String fullAutoName, boolean autoFileFound, String baseAutoName) {
        Telemetry.log("Auton/SelectedAuto", fullAutoName);
        Telemetry.log("Auton/AutoFileFound", autoFileFound);
        if (autoFileFound) {
            autoFileMissingAlert.set(false);
            Telemetry.print("Auto selected: " + fullAutoName, PrintPriority.HIGH);
        } else {
            autoFileMissingAlert.setText(
                    String.format(
                            "Auto '%s' has NO .auto file named '%s' on the robot (names are"
                                    + " case-sensitive there). It will do nothing. Pick another.",
                            fullAutoName, baseAutoName));
            autoFileMissingAlert.set(true);
        }
    }

    private void clearStartPoseReport() {
        startPoseErrorSinceSeconds = Double.NaN;
        startPoseAlert.set(false);
        startPoseUnverifiedAlert.set(false);
        Telemetry.log("Auton/StartPoseErrorMeters", Double.NaN, "m");
        Telemetry.log("Auton/StartHeadingErrorDeg", Double.NaN, "deg");
    }

    /** Compares the current pose with the selected auto's start and alerts when they disagree. */
    private void checkStartPose() {
        Optional<Pose2d> start =
                selectedAutoPaths.isEmpty()
                        ? Optional.empty()
                        : selectedAutoPaths.get(0).getStartingHolonomicPose();
        if (start.isEmpty() || hasBeenEnabled) {
            clearStartPoseReport();
            return;
        }
        if (!vision.isPoseHeadingSeeded()) {
            clearStartPoseReport();
            startPoseUnverifiedAlert.set(!RuntimeInputs.isSimulation());
            return;
        }
        startPoseUnverifiedAlert.set(false);

        Pose2d pose = swerve.getRobotPose();
        double distanceMeters = pose.getTranslation().getDistance(start.get().getTranslation());
        double headingErrorDeg = pose.getRotation().minus(start.get().getRotation()).getDegrees();
        Telemetry.logDash("Auton/StartPoseErrorMeters", distanceMeters, "m");
        Telemetry.logDash("Auton/StartHeadingErrorDeg", headingErrorDeg, "deg");

        boolean off =
                distanceMeters > START_POSE_ALERT_METERS
                        || Math.abs(headingErrorDeg) > START_HEADING_ALERT_DEG;
        double now = Timer.getTimestamp();
        if (!off) {
            startPoseErrorSinceSeconds = Double.NaN;
            startPoseAlert.set(false);
        } else if (Double.isNaN(startPoseErrorSinceSeconds)) {
            startPoseErrorSinceSeconds = now;
        } else if (now - startPoseErrorSinceSeconds >= START_POSE_ALERT_HOLD_SECONDS
                && !startPoseAlert.get()) {
            startPoseAlert.setText(
                    String.format(
                            "Robot is %.2f m and %.0f deg from the start of %s. Wrong auto, wrong"
                                    + " side, wrong alliance, or a bad vision seed.",
                            distanceMeters, headingErrorDeg, autoName));
            startPoseAlert.set(true);
        }
    }

    @Override
    public void disabledExit() {
        Telemetry.print("### Disabled Exit### ");
    }

    /* AUTONOMOUS MODE (AUTO) */
    @Override
    public void autonomousInit() {
        Telemetry.print("@@@ Auton Init @@@ ");
        String runningAuto = auton.getAutonomousCommand().getName();
        Telemetry.log("Auton/RunningAuto", runningAuto);
        if (robotSim != null) {
            robotSim.getBallSim().clearBalls();
            robotSim.getBallSim().placeFieldBalls();
        }
        try {
            auton.init();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {
        // The Limelight rewind buffer holds 165 s, less than auto plus teleop, so each period is
        // captured as it ends (10183 found the teleop-only capture missed auto at Summer Scorcher).
        if (RobotState.isFMSAttached()) {
            vision.triggerRewindCaptureForAllCameras();
        }
        auton.exit();
        Telemetry.print("@@@ Auton Exit @@@ ");
    }

    @Override
    public void teleopInit() {
        try {
            Telemetry.print("!!! Teleop Init Starting !!! ");
            superStructure.setWantedSuperState(WantedSuperState.IDLE);
            field2d.getObject("Auto Routine").setPoses(new ArrayList<>()); // clears auto visualizer
            Telemetry.print("!!! Teleop Init Complete !!! ");
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {
        if (RobotState.isFMSAttached()) {
            vision.triggerRewindCaptureForAllCameras();
        }
        Telemetry.print("!!! Teleop Exit !!! ");
    }

    /* UTILITY MODE (2026's "test" mode) */
    @Override
    public void utilityInit() {
        Telemetry.print("~~~ Utility Init ~~~ ");
    }

    @Override
    public void utilityPeriodic() {}

    @Override
    public void utilityExit() {
        Telemetry.print("~~~ Utility Exit ~~~ ");
    }

    /* SIMULATION MODE */
    @Override
    public void simulationInit() {
        Telemetry.print("$$$ Simulation Init $$$ ");
    }

    @Override
    public void simulationPeriodic() {
        if (SimScript.requested()) {
            SimScript.periodic();
        }
        if (robotSim == null) {
            return;
        }
        robotSim.getBallSim().tick(); // runs physics, publishes ball positions to NT
        robotSim.updateArticulatedMechanisms();
        Telemetry.log("Sim/Fuel", robotSim.getBallSim().getTotalIntaked());
        Telemetry.log("Sim/Launched", robotSim.getBallSim().getTotalLaunched());
        Telemetry.log("Sim/Scored", robotSim.getBallSim().getTotalScored());
    }
}
