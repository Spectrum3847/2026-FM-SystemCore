package frc.robot.subsystems.vision;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.auton.Auton;
import frc.robot.subsystems.swerve.Swerve;
import frc.spectrumLib.localization.Gate;
import frc.spectrumLib.localization.GateContext;
import frc.spectrumLib.localization.PoseFusion;
import frc.spectrumLib.localization.PoseObservation;
import frc.spectrumLib.localization.PoseObservation.Kind;
import frc.spectrumLib.localization.PoseSource;
import frc.spectrumLib.localization.PoseSourceIO;
import frc.spectrumLib.localization.YawRateHistory;
import frc.spectrumLib.telemetry.Alert;
import frc.spectrumLib.telemetry.Telemetry;
import frc.spectrumLib.telemetry.Telemetry.PrintPriority;
import frc.spectrumLib.util.Util;
import frc.spectrumLib.vision.Limelight;
import frc.spectrumLib.vision.Limelight.LimelightConfig;
import frc.spectrumLib.vision.LimelightHelpers;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.wpilib.command2.Command;
import org.wpilib.command2.Subsystem;
import org.wpilib.driverstation.Alert.Level;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.util.Units;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.system.Timer;
import org.wpilib.vision.apriltag.AprilTagFieldLayout;
import org.wpilib.vision.apriltag.AprilTagFields;

/**
 * FM's multi-source localization testbed: up to three Limelights, three Orin/PhotonVision cameras
 * and a QuestNav, each gated on its own and each logged well enough to compare after the fact.
 *
 * <h2>Every loop</h2>
 *
 * <ol>
 *   <li>Every source reads its device into logged inputs ({@link PoseSource#update()}), whether or
 *       not it is enabled.
 *   <li>Every source gates every observation and logs the verdict and reason.
 *   <li>{@link PoseFusion} applies each source's accepted measurements to that source's shadow
 *       track, and to the robot's fused pose if the source is enabled <em>and</em> its fusion
 *       policy allows it right now (see below).
 *   <li>While disabled, the best Limelight's MegaTag1 seeds the heading and translation of every
 *       track. While enabled, the gross-heading safety net can re-seed once a camera has disagreed
 *       with the gyro badly for a full second.
 * </ol>
 *
 * <h2>Fusion policy (from the 2026 code)</h2>
 *
 * <ul>
 *   <li>Limelights fuse while enabled only in teleop, or in auto while the path asks for pose
 *       updates or the robot is launching (FM's {@code main}).
 *   <li>Every chassis Limelight fuses, not just the best one (offseason: the estimator already
 *       weights each by its std-devs).
 *   <li>MegaTag1 until the disabled seed is confirmed, MegaTag2 after (offseason, switchable at
 *       {@code Vision/ChassisUseMT2}).
 *   <li>The Orin and QuestNav fuse whenever enabled and the robot is enabled. Both start
 *       <b>disabled</b>: they are logged and shadowed, but do not move the robot pose until someone
 *       flips {@code /Localization/Sources/<name>/Enabled} on the dashboard.
 * </ul>
 *
 * <p>Not ported from the offseason branch: the placement-heading vote and the two-camera consensus
 * heading correction (both need the offseason bot's camera geometry to be meaningful), and
 * everything turret-camera specific.
 */
public class Vision implements Subsystem {

    // =========================================================================
    // Configuration
    // =========================================================================

    public static class VisionConfig {
        @Getter final String name = "Vision";

        // -- Limelights (FM, from 2026 main) ----------------------------------

        @Getter
        final LimelightConfig backConfig =
                new LimelightConfig("limelight-back")
                        .withTranslation(-0.3084987734, 0.2134100126, 0.6502249886)
                        .withRotation(0, 0, 180);

        @Getter
        final LimelightConfig leftConfig =
                new LimelightConfig("limelight-left")
                        .withTranslation(0, 0.215, 0.188)
                        .withRotation(0, 0, 90);

        @Getter
        final LimelightConfig rightConfig =
                new LimelightConfig("limelight-right")
                        .withTranslation(-0.04445, 0.3027487722, 0.7137249886)
                        .withRotation(0, 0, -90);

        /**
         * Whether to push the mounts above to the cameras. Off: FM's 2026 code never pushed them,
         * so the values in each camera's flash are the ones that were validated all season and the
         * values above are unverified. Turn on once they have been checked against the web UI.
         */
        @Getter final boolean pushLimelightMounts = false;

        @Getter final int tagPipeline = 0;

        // -- SystemCore's built-in cameras (CALIBRATE: placeholder mounts) ------
        //
        // The SystemCore image runs the Limelight vision stack for cameras plugged into it,
        // publishing them as limelightsc0/1/2 and reading MegaTag2 orientation from
        // limelightshared. It only ever connects to 127.0.0.1:5810 -- which, with FM's code
        // running on the SystemCore, is this program (see Spectrum3847/SystemCoreVision; the
        // NT bridge that repo provides is only needed when a roboRIO is the controller).

        @Getter
        final LimelightConfig[] systemCoreCameras = {
            new LimelightConfig("limelightsc0")
                    .withTranslation(0.25, -0.25, 0.30)
                    .withRotation(0, 15, 0),
            new LimelightConfig("limelightsc1")
                    .withTranslation(-0.25, 0.25, 0.30)
                    .withRotation(0, 15, 180),
        };

        /** Table the SystemCore vision stack reads robot orientation from, for MegaTag2. */
        @Getter final String systemCoreSharedTable = "limelightshared";

        // -- Orin / PhotonVision (CALIBRATE: placeholders until mounted) --------

        /** Camera names in the PhotonVision UI, and their mounts (x fwd, y left, z up). */
        @Getter final String[] orinCameraNames = {"orin-front", "orin-left", "orin-right"};

        @Getter
        final Transform3d[] orinRobotToCamera = {
            new Transform3d(
                    new Translation3d(0.30, 0.0, 0.25),
                    new Rotation3d(0, Units.degreesToRadians(-15), 0)),
            new Transform3d(
                    new Translation3d(0.0, 0.30, 0.25),
                    new Rotation3d(0, Units.degreesToRadians(-15), Units.degreesToRadians(90))),
            new Transform3d(
                    new Translation3d(0.0, -0.30, 0.25),
                    new Rotation3d(0, Units.degreesToRadians(-15), Units.degreesToRadians(-90)))
        };

        // -- QuestNav (CALIBRATE: placeholder until mounted) --------------------

        @Getter
        final Transform3d robotToQuest =
                new Transform3d(new Translation3d(0.0, 0.0, 0.40), Rotation3d.kZero);

        /** Frames arriving within this long of a pose reset may predate it. */
        @Getter final double questResetSettleSeconds = 0.5;

        /** Quest frames older than this are stale (it runs at ~100 Hz). */
        @Getter final double questMaxAgeSeconds = 0.25;

        /**
         * A Quest frame whose motion since the previous frame differs from wheel odometry's by more
         * than this is a tracking jump (re-localisation, re-origin) rather than motion.
         */
        @Getter final double questMaxJumpMeters = 0.25;

        // -- Seeding (offseason) ------------------------------------------------

        /** Translation std-dev while seeding, metres. From the 2026 {@code forceIntegrateXY}. */
        @Getter final double seedXyStdDev = 0.01;

        /** Heading std-dev while seeding, degrees. From the 2026 {@code forceIntegrateXY}. */
        @Getter final double seedThetaStdDevDeg = 0.01;

        /**
         * Consecutive seeding frames in which the best Limelight must have seeded the pose from two
         * or more tags, with its heading holding within {@link #seedConfirmSpreadDeg}, before the
         * seed counts as confirmed. About a second at Limelight frame rates. (2026 counted loops
         * and re-read the same frame each loop; frames are now reported once, so this counts frames
         * and loops without a new frame neither extend nor break the run.)
         */
        @Getter final int seedConfirmLoops = 50;

        /**
         * Peak-to-peak spread (degrees) the MegaTag1 heading may show across the confirmation run.
         * Four-tag heading holds to about a degree; a two-tag run that wanders past this is the
         * geometry noise this exists to wait out, and the run starts over.
         */
        @Getter final double seedConfirmSpreadDeg = 3.0;

        /**
         * Gross heading correction while enabled. 20 deg sits clear of two-tag MegaTag1 heading
         * noise (15 deg tail) while staying far below the 90 and 180 deg boot-heading errors this
         * exists to catch. See the offseason {@code Vision.VisionConfig} for the log evidence.
         */
        @Getter final double grossHeadingErrorDeg = 20.0;

        @Getter final double grossHeadingHoldSeconds = 1.0;
        @Getter final double grossHeadingMaxLinearSpeed = 0.2; // m/s
        @Getter final double grossHeadingMaxOmega = 0.1; // rad/s
    }

    // =========================================================================
    // Fields
    // =========================================================================

    @Getter private static AprilTagFieldLayout tagLayout;

    private final VisionConfig config;
    private final Swerve swerve;
    private final PoseFusion fusion;

    @Getter private final List<Limelight> limelights = new ArrayList<>();
    @Getter private final List<PoseSource> mt1Sources = new ArrayList<>();
    @Getter private final List<PoseSource> mt2Sources = new ArrayList<>();
    @Getter private final List<PoseSource> scMt1Sources = new ArrayList<>();
    @Getter private final List<PoseSource> scMt2Sources = new ArrayList<>();
    @Getter private final List<PoseSource> orinSources = new ArrayList<>();
    @Getter private PoseSource questSource;
    @Getter private final List<PoseSource> allSources = new ArrayList<>();

    private QuestNavControl questControl = QuestNavControl.NONE;
    private SimVision simVision;
    private final List<SimVision.LimelightSimCamera> simLimelights = new ArrayList<>();

    private final YawRateHistory yawRates = new YawRateHistory(64);
    private final LoggedNetworkBoolean chassisUseMt2 =
            new LoggedNetworkBoolean("/Vision/ChassisUseMT2", true);

    @Getter private boolean poseHeadingSeeded = false;
    @Getter private boolean poseSeedConfirmed = false;
    private int seedConfirmStreak = 0;
    private Rotation2d seedConfirmHeadingRef = Rotation2d.kZero;
    private double seedConfirmSpreadLow = 0;
    private double seedConfirmSpreadHigh = 0;
    private double grossHeadingSince = Double.NaN;
    private int grossHeadingCorrections = 0;

    /**
     * One disconnected alert per camera (a Limelight's MT1 and MT2 sources are one camera). FM's
     * Limelights fuse by default, so theirs always applies; every other camera's only while one of
     * its sources has its fuse switch on, so an unplugged testbed camera stays quiet.
     */
    private record CameraAlert(
            String camera,
            List<PoseSource> sources,
            boolean always,
            Alert alert,
            org.wpilib.math.filter.Debouncer debounce) {}

    private final List<CameraAlert> cameraAlerts = new ArrayList<>();

    /** Quest battery below this percentage raises an alert (1425 uses 20). */
    private static final double QUEST_LOW_BATTERY_PERCENT = 20;

    private final Alert questBatteryAlert = new Alert("", Level.MEDIUM);

    private final Alert notSeededAlert =
            new Alert(
                    "Pose heading has not been vision-seeded yet - wait for a Limelight to see tags"
                            + " before enabling",
                    Level.MEDIUM);
    private final Alert notConfirmedAlert =
            new Alert(
                    "Pose seed not confirmed yet - a Limelight needs two or more tags, steady, for"
                            + " about a second before auto starts",
                    Level.MEDIUM);

    // =========================================================================
    // Construction
    // =========================================================================

    public Vision(VisionConfig config, Swerve swerve) {
        this.config = config;
        this.swerve = swerve;
        this.fusion = swerve.getPoseFusion();
        tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

        boolean sim = Constants.currentMode == Constants.Mode.SIM;
        boolean replay = Constants.currentMode == Constants.Mode.REPLAY;
        if (sim) {
            simVision = new SimVision(tagLayout, swerve::getSimTruthPose);
        }

        // Limelights: MT1 and MT2 are separate sources so each gets its own shadow track.
        LimelightConfig[] lls = {config.backConfig, config.leftConfig, config.rightConfig};
        for (LimelightConfig llConfig : lls) {
            addLimelight(llConfig, true, mt1Sources, mt2Sources, sim, replay);
        }
        // SystemCore's own cameras: logged and shadowed, fused only when enabled on the dashboard.
        for (LimelightConfig llConfig : config.systemCoreCameras) {
            addLimelight(llConfig, false, scMt1Sources, scMt2Sources, sim, replay);
        }

        // Orin cameras.
        for (int i = 0; i < config.orinCameraNames.length; i++) {
            String name = config.orinCameraNames[i];
            Transform3d mount = config.orinRobotToCamera[i];
            PoseSourceIO io;
            if (replay) {
                io = PoseSourceIO.NONE;
            } else {
                if (sim) {
                    simVision.addCamera(name, SimVision.thriftiestCam(), mount);
                }
                io = new PhotonIO(name, mount, tagLayout);
            }
            orinSources.add(
                    new PoseSource(
                            shortName(name),
                            io,
                            VisionGates.aprilTagGates(),
                            VisionGates.PHOTON_MODEL,
                            false));
        }

        // QuestNav.
        PoseSourceIO questIo;
        if (replay) {
            questIo = PoseSourceIO.NONE;
        } else if (sim) {
            var q = new SimVision.QuestSimIO(swerve::getSimTruthPose);
            questControl = q;
            questIo = q;
        } else {
            var q = new QuestNavIO(config.robotToQuest);
            questControl = q;
            questIo = q;
        }
        questSource =
                new PoseSource("Quest", questIo, questGates(), VisionGates.QUEST_MODEL, false);

        allSources.addAll(mt1Sources);
        allSources.addAll(mt2Sources);
        allSources.addAll(scMt1Sources);
        allSources.addAll(scMt2Sources);
        allSources.addAll(orinSources);
        allSources.add(questSource);
        for (PoseSource s : allSources) {
            fusion.register(s);
        }
        for (int i = 0; i < mt1Sources.size(); i++) {
            addCameraAlert(List.of(mt1Sources.get(i), mt2Sources.get(i)), true);
        }
        for (int i = 0; i < scMt1Sources.size(); i++) {
            addCameraAlert(List.of(scMt1Sources.get(i), scMt2Sources.get(i)), false);
        }
        for (PoseSource s : orinSources) {
            addCameraAlert(List.of(s), false);
        }
        addCameraAlert(List.of(questSource), false);

        if (!replay) {
            for (Limelight ll : limelights) {
                ll.setLEDMode(false);
            }
            sendCameraSettings(Timer.getTimestamp());
        }

        // Not registered with the scheduler: Robot runs it explicitly, after the drivetrain's
        // odometry and before the scheduler, so this loop's correction lands before any
        // mechanism or command reads the pose (the offseason ordering).
        Telemetry.print(getName() + " Subsystem Initialized");
    }

    /** Creates one Limelight's MT1 and MT2 sources (real, simulated, or replay). */
    private void addLimelight(
            LimelightConfig llConfig,
            boolean fusedByDefault,
            List<PoseSource> mt1List,
            List<PoseSource> mt2List,
            boolean sim,
            boolean replay) {
        Limelight ll = new Limelight(llConfig.getName(), config.tagPipeline, llConfig);
        limelights.add(ll);
        PoseSourceIO mt1Io;
        PoseSourceIO mt2Io;
        if (replay) {
            mt1Io = PoseSourceIO.NONE;
            mt2Io = PoseSourceIO.NONE;
        } else if (sim) {
            Transform3d mount = SimVision.robotToCamera(llConfig);
            var cam =
                    new SimVision.LimelightSimCamera(
                            simVision.addCamera(
                                    "sim-" + llConfig.getName(), SimVision.limelight4(), mount),
                            mount,
                            tagLayout);
            simLimelights.add(cam);
            mt1Io = new SimVision.LimelightSimIO(cam, Kind.LIMELIGHT_MT1, this::headingAt);
            mt2Io = new SimVision.LimelightSimIO(cam, Kind.LIMELIGHT_MT2, this::headingAt);
        } else {
            mt1Io = new LimelightIO(ll, Kind.LIMELIGHT_MT1);
            mt2Io = new LimelightIO(ll, Kind.LIMELIGHT_MT2);
        }
        String base = shortName(llConfig.getName());
        mt1List.add(
                new PoseSource(
                        base + "/MT1",
                        mt1Io,
                        VisionGates.aprilTagGates(),
                        VisionGates.LIMELIGHT_TIERS,
                        fusedByDefault));
        mt2List.add(
                new PoseSource(
                        base + "/MT2",
                        mt2Io,
                        VisionGates.aprilTagGates(),
                        VisionGates.LIMELIGHT_TIERS,
                        fusedByDefault));
    }

    /** "limelight-back" becomes "LL-Back", "limelightsc0" becomes "SC0". */
    private static String shortName(String name) {
        if (name.startsWith("limelightsc")) {
            return "SC" + name.substring("limelightsc".length());
        }
        if (name.startsWith("limelight-")) {
            String side = name.substring("limelight-".length());
            return "LL-" + Character.toUpperCase(side.charAt(0)) + side.substring(1);
        }
        return name;
    }

    @Override
    public String getName() {
        return config.getName();
    }

    /** The fused heading at a capture time: what a Limelight was told the robot's yaw was. */
    private Rotation2d headingAt(double timestampSeconds) {
        return fusion.sampleAt(timestampSeconds).orElse(fusion.getPose()).getRotation();
    }

    // =========================================================================
    // QuestNav gates (VIO failure modes, not AprilTag ones)
    // =========================================================================

    private Pose2d lastQuestPose = null;
    private Pose2d lastQuestOdometry = null;

    private List<Gate> questGates() {
        return List.of(
                Gate.rejectIf("Not Tracking Rejection", (o, c) -> !o.tracking()),
                Gate.rejectIf(
                        "Not Reset Rejection",
                        (o, c) -> Double.isNaN(questHealth(QuestNavIO.H_LAST_RESET_TIME))),
                Gate.rejectIf(
                        "Recent Reset Rejection",
                        (o, c) ->
                                o.timestampSeconds() - questHealth(QuestNavIO.H_LAST_RESET_TIME)
                                        < config.questResetSettleSeconds),
                Gate.rejectIf(
                        "Stale Estimate Rejection",
                        (o, c) ->
                                c.nowSeconds() - o.timestampSeconds() > config.questMaxAgeSeconds),
                Gate.rejectIf(
                        "Out of Field Rejection",
                        (o, c) -> frc.rebuilt.FieldHelpers.poseOutOfField(o.pose2d())),
                this::questJumpGate);
    }

    /**
     * Compares the Quest's motion since its last frame with wheel odometry's over the same loop. A
     * VIO source that re-localises or re-origins jumps; the wheels never do.
     */
    private String questJumpGate(PoseObservation o, GateContext c) {
        Pose2d quest = o.pose2d();
        Pose2d odom = c.odometryPose();
        String result = null;
        if (lastQuestPose != null && lastQuestOdometry != null) {
            double questMove = quest.getTranslation().getDistance(lastQuestPose.getTranslation());
            double odomMove = odom.getTranslation().getDistance(lastQuestOdometry.getTranslation());
            if (Math.abs(questMove - odomMove) > config.questMaxJumpMeters) {
                result = "Jump vs Odometry Rejection";
            }
        }
        lastQuestPose = quest;
        lastQuestOdometry = odom;
        return result;
    }

    private double questHealth(int index) {
        double[] h = questSource.getInputs().health;
        return index < h.length ? h[index] : Double.NaN;
    }

    // =========================================================================
    // Periodic
    // =========================================================================

    @Override
    public void periodic() {
        double now = Timer.getTimestamp();
        boolean disabled = Util.disabled.getAsBoolean();

        if (simVision != null) {
            simVision.update();
            for (var cam : simLimelights) {
                cam.refresh();
            }
        }
        for (Limelight ll : limelights) {
            ll.invalidate();
        }

        // 1. Read every device.
        for (PoseSource s : allSources) {
            s.update();
        }

        // 2. Gate.
        yawRates.record(now, swerve.getYawRateRadPerSec());
        GateContext context =
                new GateContext(
                        now,
                        fusion.getPose(),
                        fusion.getOdometryPose(),
                        swerve.getCurrentRobotChassisSpeeds(),
                        yawRates.peak(now, VisionGates.YAW_RATE_LOOKBACK_SECONDS),
                        disabled,
                        poseHeadingSeeded);
        for (PoseSource s : allSources) {
            s.process(context);
        }

        // 3. Seed (disabled, or enabled with nothing seeded yet) and fuse (enabled).
        if (disabled) {
            seedWhileDisabled();
        } else {
            seedWhileEnabled(context);
        }
        boolean llWindow =
                !disabled
                        && (Util.teleop.getAsBoolean()
                                || Auton.autonPoseUpdate.getAsBoolean()
                                || Auton.autonLaunching.getAsBoolean());
        boolean useMt2 = poseSeedConfirmed && chassisUseMt2.get();
        for (PoseSource s : mt1Sources) {
            applyWithPolicy(s, llWindow && !useMt2);
        }
        for (PoseSource s : mt2Sources) {
            applyWithPolicy(s, llWindow && useMt2);
        }
        for (PoseSource s : scMt1Sources) {
            applyWithPolicy(s, llWindow && !useMt2);
        }
        for (PoseSource s : scMt2Sources) {
            applyWithPolicy(s, llWindow && useMt2);
        }
        for (PoseSource s : orinSources) {
            applyWithPolicy(s, !disabled);
        }
        applyWithPolicy(questSource, !disabled);

        if (!disabled) {
            checkGrossHeadingError(context);
        }

        // 4. Keep the Limelights and the Quest aligned with the fused pose.
        if (Constants.currentMode != Constants.Mode.REPLAY) {
            sendCameraSettings(now);
        }
        pushHeadingToLimelights();
        resetQuestAfterSeed();

        logStatus(disabled, useMt2);
        swerve.afterVision();
    }

    /** When a measurement last moved the fused pose; NaN until one has. */
    private double lastFusedSeconds = Double.NaN;

    /**
     * Applies a source's results. The shadow always gets them; the fused pose only when the source
     * is enabled and {@code policyAllows}. Logs which it was.
     */
    private void applyWithPolicy(PoseSource source, boolean policyAllows) {
        boolean fuse = policyAllows && source.isEnabled();
        source.logPolicy(policyAllows);
        if (fuse) {
            for (PoseSource.Result r : source.getResults()) {
                if (r.accepted()) {
                    lastFusedSeconds = Timer.getTimestamp();
                    break;
                }
            }
        }
        fusion.apply(source, source.getResults(), fuse);
    }

    // =========================================================================
    // Seeding (offseason)
    // =========================================================================

    /** Best Limelight this loop: most MegaTag1 tags, then largest target (offseason ranking). */
    private int bestLimelightIndex() {
        int best = -1;
        double bestScore = 0;
        for (int i = 0; i < mt1Sources.size(); i++) {
            for (PoseSource.Result r : mt1Sources.get(i).getResults()) {
                PoseObservation o = r.observation();
                double score =
                        o.tagCount() * 100.0
                                + (Double.isNaN(o.targetSizePercent()) ? 0 : o.targetSizePercent());
                if (score > bestScore) {
                    bestScore = score;
                    best = i;
                }
            }
        }
        return best;
    }

    /** Whether any Limelight reported a MegaTag1 frame this loop. */
    private boolean anyMt1Frames() {
        for (PoseSource s : mt1Sources) {
            if (!s.getResults().isEmpty()) {
                return true;
            }
        }
        return false;
    }

    /** The best Limelight's latest accepted MegaTag1 result this loop, or {@code null}. */
    private PoseSource.Result bestAcceptedMt1() {
        int best = bestLimelightIndex();
        if (best < 0) {
            return null;
        }
        List<PoseSource.Result> results = mt1Sources.get(best).getResults();
        for (int i = results.size() - 1; i >= 0; i--) {
            if (results.get(i).accepted()) {
                return results.get(i);
            }
        }
        return null;
    }

    /**
     * While disabled, seeds every track (translation and heading) from the best Limelight's
     * MegaTag1 only. MegaTag2 is deliberately not used here: it depends on the heading we push to
     * the camera, which is the very thing seeding is trying to fix.
     */
    private void seedWhileDisabled() {
        PoseSource.Result best = bestAcceptedMt1();
        if (best == null) {
            // A loop with no new frame from any camera is not evidence either way (the cameras run
            // slower than the loop, and each frame is reported once); a loop whose frames were all
            // rejected breaks the run.
            if (anyMt1Frames()) {
                seedConfirmStreak = 0;
                Telemetry.log("Vision/SeedConfirmBreak", "Best frame rejected");
            }
            return;
        }
        PoseObservation o = best.observation();
        fusion.seed(
                o.pose2d(),
                o.timestampSeconds(),
                config.seedXyStdDev,
                Units.degreesToRadians(config.seedThetaStdDevDeg));
        poseHeadingSeeded = true;
        trackSeedConfirmation(o);
    }

    /** Times the pose was seeded while enabled; logged so a match log shows it happened. */
    private int enabledSeedCount = 0;

    /**
     * Seeds the pose during the match when nothing seeded it before enable, then confirms that seed
     * the way a disabled one is confirmed.
     *
     * <p>Offseason Chezy QM4 (2026-09-19, 2e1ea2d): no camera saw a tag before the match, the robot
     * enabled unseeded, and the only in-match recovery was the gross heading correction, which only
     * fires above {@link VisionConfig#grossHeadingErrorDeg}; the robot ran about 7 deg off all
     * match. FM aims with its drivetrain heading, so the same error costs every shot.
     *
     * <p>Seeding takes the first multi-tag MegaTag1 solve from the best Limelight while the robot
     * is slow enough for the gross-heading net ({@link VisionConfig#grossHeadingMaxLinearSpeed},
     * {@link VisionConfig#grossHeadingMaxOmega}). Single-tag headings are where the large MegaTag1
     * heading errors come from, and a wrong seed while enabled aims wrong with confidence.
     * Confirmation (which lets the chassis cameras move on to MegaTag2) is stricter than the
     * disabled version: each counted frame's heading must also agree with the fused heading.
     */
    private void seedWhileEnabled(GateContext c) {
        if (poseSeedConfirmed) {
            return;
        }
        PoseSource.Result best = bestAcceptedMt1();
        if (best == null) {
            if (poseHeadingSeeded && anyMt1Frames()) {
                seedConfirmStreak = 0;
            }
            return;
        }
        PoseObservation o = best.observation();
        if (!poseHeadingSeeded) {
            boolean slow =
                    c.linearSpeed() <= config.grossHeadingMaxLinearSpeed
                            && Math.abs(c.robotVelocity().omega) <= config.grossHeadingMaxOmega;
            if (!slow || !o.multiTag()) {
                return;
            }
            fusion.seed(
                    o.pose2d(),
                    o.timestampSeconds(),
                    config.seedXyStdDev,
                    Units.degreesToRadians(config.seedThetaStdDevDeg));
            poseHeadingSeeded = true;
            enabledSeedCount++;
            resetQuestToRobotPose();
            Telemetry.print(
                    String.format(
                            "Vision: pose seeded WHILE ENABLED from %s (%d tags); nothing had"
                                    + " seeded it before enable.",
                            o.source(), o.tagCount()),
                    PrintPriority.HIGH);
            return;
        }
        double errorDeg =
                o.pose2d().getRotation().minus(headingAt(o.timestampSeconds())).getDegrees();
        if (Math.abs(errorDeg) <= config.seedConfirmSpreadDeg) {
            trackSeedConfirmation(o);
        } else {
            seedConfirmStreak = 0;
            Telemetry.log("Vision/SeedConfirmBreak", "Disagrees with fused heading");
        }
    }

    /**
     * Counts consecutive seeding frames with a multi-tag MegaTag1 heading that holds within {@link
     * VisionConfig#seedConfirmSpreadDeg}; the seed is confirmed after {@link
     * VisionConfig#seedConfirmLoops}. Headings are compared relative to the run's first sample so
     * the wrap at 180 deg cannot split a run.
     */
    private void trackSeedConfirmation(PoseObservation o) {
        if (poseSeedConfirmed) {
            return;
        }
        if (!o.multiTag()) {
            seedConfirmStreak = 0;
            Telemetry.log("Vision/SeedConfirmBreak", "Single tag");
            return;
        }
        Rotation2d heading = o.pose2d().getRotation();
        if (seedConfirmStreak == 0) {
            seedConfirmHeadingRef = heading;
            seedConfirmSpreadLow = 0;
            seedConfirmSpreadHigh = 0;
        }
        double d = heading.minus(seedConfirmHeadingRef).getDegrees();
        double low = Math.min(seedConfirmSpreadLow, d);
        double high = Math.max(seedConfirmSpreadHigh, d);
        if (high - low > config.seedConfirmSpreadDeg) {
            Telemetry.log("Vision/SeedConfirmBreak", "Heading spread");
            seedConfirmHeadingRef = heading;
            seedConfirmStreak = 0;
            low = 0;
            high = 0;
        }
        seedConfirmSpreadLow = low;
        seedConfirmSpreadHigh = high;
        seedConfirmStreak++;

        if (seedConfirmStreak >= config.seedConfirmLoops) {
            poseSeedConfirmed = true;
            Telemetry.print(
                    String.format(
                            "Vision: pose seed confirmed from %s (%d tags, %.1f deg spread);"
                                    + " Limelights will fuse MegaTag2 while enabled.",
                            o.source(), o.tagCount(), high - low));
        }
    }

    /**
     * Safety net for enabling before the cameras have seeded the pose: a stationary robot whose
     * multi-tag MegaTag1 heading disagrees with the fused heading by more than {@link
     * VisionConfig#grossHeadingErrorDeg} for {@link VisionConfig#grossHeadingHoldSeconds} gets
     * re-seeded from that camera.
     */
    private void checkGrossHeadingError(GateContext c) {
        PoseSource.Result best = bestAcceptedMt1();
        boolean still =
                c.linearSpeed() <= config.grossHeadingMaxLinearSpeed
                        && Math.abs(c.robotVelocity().omega) <= config.grossHeadingMaxOmega;
        if (best == null || !best.observation().multiTag() || !still) {
            grossHeadingSince = Double.NaN;
            return;
        }
        PoseObservation o = best.observation();
        double errorDeg =
                o.pose2d().getRotation().minus(headingAt(o.timestampSeconds())).getDegrees();
        Logger.recordOutput("Vision/GrossHeadingErrorDeg", errorDeg);
        if (Math.abs(errorDeg) < config.grossHeadingErrorDeg) {
            grossHeadingSince = Double.NaN;
            return;
        }
        if (Double.isNaN(grossHeadingSince)) {
            grossHeadingSince = c.nowSeconds();
        } else if (c.nowSeconds() - grossHeadingSince >= config.grossHeadingHoldSeconds) {
            fusion.seed(
                    o.pose2d(),
                    o.timestampSeconds(),
                    config.seedXyStdDev,
                    Units.degreesToRadians(config.seedThetaStdDevDeg));
            grossHeadingCorrections++;
            grossHeadingSince = Double.NaN;
            resetQuestToRobotPose();
            Telemetry.print(
                    String.format(
                            "Vision: gross heading correction of %.1f deg from %s",
                            errorDeg, o.source()));
        }
    }

    // =========================================================================
    // Outputs to devices
    // =========================================================================

    /**
     * How often the settings the robot owns (IMU mode, and the mount poses when {@link
     * VisionConfig#pushLimelightMounts} is on) are re-sent. A Limelight that boots after the robot,
     * or reboots mid-match, comes back in IMU mode 0 and on whatever mount was last saved to it; in
     * mode 0 its MegaTag2 poses are garbage, and FM fuses MegaTag2 once the seed is confirmed.
     * Offseason 71dc7a7.
     */
    private static final double SETTINGS_RESEND_PERIOD_SECS = 2.0;

    private double lastSettingsSendSeconds = Double.NEGATIVE_INFINITY;

    /** Sends each Limelight its IMU mode (and mount), at most every resend period. */
    private void sendCameraSettings(double now) {
        if (now - lastSettingsSendSeconds < SETTINGS_RESEND_PERIOD_SECS) {
            return;
        }
        lastSettingsSendSeconds = now;
        for (Limelight ll : limelights) {
            ll.setIMUmode(1);
            if (config.pushLimelightMounts) {
                ll.pushConfiguredCameraPose();
            }
        }
    }

    private long orientationPushes = 0;

    /**
     * MegaTag2 needs the robot's heading every loop: it is written every loop, and one
     * NetworkTables flush for all cameras sends it straight away every other loop, i.e. at 50 Hz --
     * the rate FM's 2026 code flushed at on the roboRIO. A flush makes the NT server send to every
     * client, so at 100 Hz it was twice the traffic for a heading the cameras' own IMUs (IMU mode
     * 1) already bridge between updates. The loops in between still go out at NT's normal update
     * rate.
     */
    private void pushHeadingToLimelights() {
        if (!Constants.hasHardware() || simVision != null) {
            return;
        }
        double yaw = fusion.getPose().getRotation().getDegrees();
        double yawRate = Units.radiansToDegrees(swerve.getYawRateRadPerSec());
        for (Limelight ll : limelights) {
            ll.setRobotOrientation(yaw, yawRate);
        }
        LimelightHelpers.SetRobotOrientation_NoFlush(
                config.systemCoreSharedTable, yaw, yawRate, 0, 0, 0, 0);
        if ((orientationPushes++ & 1) == 0) {
            NetworkTableInstance.getDefault().flush();
        }
    }

    /** Whether the seed was confirmed as of last loop, to catch the moment it becomes confirmed. */
    private boolean wasSeedConfirmed = false;

    /**
     * Re-origins the Quest onto the fused pose whenever the fused pose has just been set by vision:
     * the moment the disabled seed is confirmed, and after a gross heading correction. A Quest
     * re-origined before the seed would carry the robot's pre-seed guess forever. Explicit resets
     * (auto start, reorient, the operator's LB+X) call {@link #resetQuestToRobotPose()}.
     */
    private void resetQuestAfterSeed() {
        if (poseSeedConfirmed && !wasSeedConfirmed) {
            resetQuestToRobotPose();
        }
        wasSeedConfirmed = poseSeedConfirmed;
    }

    /** Sends the Quest the current fused pose. */
    public void resetQuestToRobotPose() {
        questControl.resetPose(fusion.getPose());
        lastQuestPose = null;
        Telemetry.print("Vision: QuestNav re-origined to the robot pose");
    }

    private void addCameraAlert(List<PoseSource> sources, boolean always) {
        String camera = sources.get(0).getName().split("/")[0];
        cameraAlerts.add(
                new CameraAlert(
                        camera,
                        sources,
                        always,
                        new Alert("Camera disconnected: " + camera, Level.MEDIUM),
                        new org.wpilib.math.filter.Debouncer(2.0)));
    }

    /** Disconnected-camera and Quest battery alerts, from logged inputs. */
    private void updateDeviceAlerts() {
        for (CameraAlert c : cameraAlerts) {
            boolean connected = false;
            boolean wanted = c.always();
            for (PoseSource s : c.sources()) {
                connected |= s.isConnected();
                wanted |= s.isEnabled();
            }
            c.alert().set(c.debounce().calculate(wanted && !connected));
        }
        double[] health = questSource.getInputs().health;
        double battery = health.length > 2 ? health[2] : -1;
        questBatteryAlert.setText(String.format("Quest battery low: %.0f%%", battery));
        questBatteryAlert.set(
                questSource.isConnected() && battery >= 0 && battery < QUEST_LOW_BATTERY_PERCENT);
    }

    private void logStatus(boolean disabled, boolean useMt2) {
        updateDeviceAlerts();
        notSeededAlert.set(!poseHeadingSeeded && disabled);
        notConfirmedAlert.set(poseHeadingSeeded && !poseSeedConfirmed && disabled);
        Telemetry.logDash("Vision/PoseHeadingSeeded", poseHeadingSeeded);
        Telemetry.logDash("Vision/PoseSeedConfirmed", poseSeedConfirmed);
        Telemetry.logDash("Vision/SeedConfirmProgress", (long) seedConfirmStreak);
        Telemetry.logDash("Vision/ChassisSource", useMt2 ? "MT2" : "MT1");
        Telemetry.logDash("Vision/GrossHeadingCorrections", (long) grossHeadingCorrections);
        Telemetry.log("Vision/EnabledSeedCount", (long) enabledSeedCount);
        Telemetry.logDash(
                "Vision/SecondsSinceFusedEstimate", Math.min(secondsSinceFusedEstimate(), 999.0));
        if (Telemetry.slowLogThisLoop()) {
            // One connected light per camera for the Pre-Match tab.
            for (int i = 0; i < mt1Sources.size(); i++) {
                Telemetry.logDash(
                        "Vision/" + mt1Sources.get(i).getName().split("/")[0] + "/Connected",
                        mt1Sources.get(i).isConnected());
            }
            for (PoseSource s : orinSources) {
                Telemetry.logDash("Vision/" + s.getName() + "/Connected", s.isConnected());
            }
            Telemetry.logDash("Vision/Quest/Connected", questSource.isConnected());
        }
        // (Each source's Connected flag is already a logged input.)
        if (!Telemetry.slowLogThisLoop()) {
            return; // the Field2d below is for dashboards; 10 Hz is plenty
        }
        // Each Limelight's latest MegaTag1 pose on the Field2d, named like its camera.
        List<PoseSource> cameraMt1 = new ArrayList<>(mt1Sources);
        cameraMt1.addAll(scMt1Sources);
        for (int i = 0; i < limelights.size() && i < cameraMt1.size(); i++) {
            var results = cameraMt1.get(i).getResults();
            if (!results.isEmpty()) {
                Robot.getField2d()
                        .getObject(limelights.get(i).getCameraName())
                        .setPose(results.get(results.size() - 1).observation().pose2d());
            }
        }
    }

    // =========================================================================
    // Queries & commands kept from 2026
    // =========================================================================

    /**
     * Seconds since a vision measurement last moved the fused pose; infinite before the first one.
     * How long the pose has been running on odometry alone. From logged inputs and the replayed
     * clock, so logic may read it.
     */
    public double secondsSinceFusedEstimate() {
        return Double.isNaN(lastFusedSeconds)
                ? Double.POSITIVE_INFINITY
                : Timer.getTimestamp() - lastFusedSeconds;
    }

    /** Whether any Limelight produced an accepted estimate this loop. */
    public boolean hasAccuratePose() {
        for (PoseSource s : mt1Sources) {
            for (var r : s.getResults()) {
                if (r.accepted()) {
                    return true;
                }
            }
        }
        return false;
    }

    /**
     * Saves the last 165 s (the most a Limelight keeps) of each of FM's Limelights' rewind buffer:
     * video plus every frame's results, reviewable frame by frame afterwards. Rewind is a Limelight
     * 4 feature; the camera writes the capture to its own storage and manages that space itself.
     *
     * <p>Not the SystemCore's own cameras: their "camera storage" is the robot controller's
     * internal disk, which {@link frc.spectrumLib.telemetry.LogStorage} is keeping free.
     */
    public void triggerRewindCaptureForAllCameras() {
        if (!Constants.hasHardware() || simVision != null) {
            return;
        }
        for (Limelight limelight : limelights) {
            if (limelight.getName().startsWith("limelightsc")) {
                continue;
            }
            LimelightHelpers.triggerRewindCapture(limelight.getName(), 165);
        }
    }

    /** Sets all Limelights to a pipeline. */
    public void setLimelightPipelines(int pipeline) {
        for (Limelight limelight : limelights) {
            limelight.setLimelightPipeline(pipeline);
        }
    }

    /** Blinks every Limelight's LEDs while held. */
    public Command blinkLimelights() {
        return startEnd(
                        () -> limelights.forEach(Limelight::blinkLEDs),
                        () -> limelights.forEach(ll -> ll.setLEDMode(false)))
                .withName("Vision.blinkLimelights");
    }

    /** Re-origins the QuestNav onto the current robot pose. */
    public Command resetQuestCommand() {
        return runOnce(this::resetQuestToRobotPose)
                .ignoringDisable(true)
                .withName("Vision.resetQuest");
    }

    /** A {@link BooleanSupplier} for whether the pose seed is confirmed. */
    public BooleanSupplier seedConfirmed() {
        return () -> poseSeedConfirmed;
    }
}
