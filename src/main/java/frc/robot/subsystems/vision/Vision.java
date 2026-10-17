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
import frc.spectrumLib.localization.PoseSourceInputs;
import frc.spectrumLib.localization.TagEkf;
import frc.spectrumLib.localization.TagSighting;
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
import java.util.Optional;
import java.util.Set;
import java.util.function.BooleanSupplier;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.wpilib.command2.Command;
import org.wpilib.command2.Subsystem;
import org.wpilib.driverstation.Alert.Level;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
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

        // -- Limelights ---------------------------------------------------------
        //
        // Web UI convention: forward, right, up; pitch positive up; yaw positive clockwise.
        // Left and right positions are from CAD (Camera Pose feature), the back camera's from its
        // flash. Pitch and yaw were solved from TXMAN1 Q13 rewinds against the fused pose.

        @Getter
        final LimelightConfig backConfig =
                new LimelightConfig("limelight-back")
                        .withTranslation(-0.3318, 0.0, 0.4465)
                        .withRotation(0, 13.5, 177);

        @Getter
        final LimelightConfig leftConfig =
                new LimelightConfig("limelight-left")
                        .withTranslation(0.0079, -0.3350, 0.5007)
                        .withRotation(0, 20, -90);

        @Getter
        final LimelightConfig rightConfig =
                new LimelightConfig("limelight-right")
                        .withTranslation(0.0084, 0.3337, 0.5004)
                        .withRotation(0, 20, 84.5);

        /**
         * Whether to push the mounts above to the cameras, so a reflashed or swapped camera still
         * gets the right mount.
         */
        @Getter final boolean pushLimelightMounts = true;

        @Getter final int tagPipeline = 0;

        // -- SystemCore's built-in cameras (CALIBRATE: placeholder mounts) ------
        //
        // The SystemCore image runs the Limelight vision stack for cameras plugged into it,
        // publishing them as limelightsc0/1/2 and reading MegaTag2 orientation from
        // limelightshared. It only ever connects to 127.0.0.1:5810. With FM's code running on the
        // SystemCore, that address is this program (see Spectrum3847/SystemCoreVision; the NT
        // bridge that repo provides is only needed when a roboRIO is the controller).

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

        // -- Orin / PhotonVision on the Jetson (issue #10) ------------------------

        /**
         * Camera names in PhotonVision. FM's Jetson has two cameras; add a name, mount, installed
         * flag and factor to run another.
         */
        @Getter final String[] orinCameraNames = {"orin-left", "orin-right"};

        /** Which cameras are plugged in. Only these raise camera alerts. */
        @Getter final boolean[] orinCameraInstalled = {true, true};

        /**
         * Camera mounts, robot frame (x fwd, y left, z up; negative pitch tilts up). Position from
         * CAD. Pitch, and orin-right's yaw, solved from TXMAN1 logs (Q13: 1318 multi-tag frames);
         * the right bracket is toed 5.5 deg forward, as limelight-right's solve also shows. The
         * Jetson's mount estimate ({@code Vision/Orin/<camera>/Mount/*}) checks height, pitch and
         * roll.
         */
        @Getter
        final Transform3d[] orinRobotToCamera = {
            new Transform3d(
                    new Translation3d(0.0836, 0.3231, 0.4883),
                    new Rotation3d(0, Units.degreesToRadians(-18), Units.degreesToRadians(90))),
            new Transform3d(
                    new Translation3d(0.0836, -0.3231, 0.4883),
                    new Rotation3d(0, Units.degreesToRadians(-18), Units.degreesToRadians(-84.5)))
        };

        /** Per-camera std-dev multiplier (6328's camera factor); 1 until logs say otherwise. */
        @Getter final double[] orinCameraFactors = {1.0, 1.0};

        /** The Jetson's image size, for the image-edge std-dev scaling. */
        @Getter final int orinImageWidth = 1280;

        @Getter final int orinImageHeight = 800;

        /**
         * Tags every Orin camera should ignore at this event (a tag mounted wrong), until changed
         * on the dashboard at {@code /Vision/Orin/ExcludedTags}.
         */
        @Getter final int[] orinExcludedTags = {};

        /** Whether the Jetson is on the robot; its alerts are off without it. */
        @Getter final boolean jetsonInstalled = true;

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
         * seed counts as confirmed. About a second at Limelight frame rates. This counts frames,
         * not loops: a loop with no new frame neither extends nor breaks the run.
         */
        @Getter final int seedConfirmLoops = 50;

        /**
         * Peak-to-peak spread (degrees) the MegaTag1 heading may show across the confirmation run.
         * Four-tag heading holds to about a degree; a two-tag run that wanders past this is the
         * geometry noise this exists to wait out, and the run starts over.
         */
        @Getter final double seedConfirmSpreadDeg = 3.0;

        /**
         * Gross heading correction while enabled. 10 deg sits above the 5 to 8 deg that one
         * camera's mount error can show. A single two-tag frame can read 15 deg off, so every
         * multi-tag frame across the hold must be past the threshold.
         */
        @Getter final double grossHeadingErrorDeg = 10.0;

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
    private final List<OrinPoseSourceIO> orinIos = new ArrayList<>();
    private final List<Transform3d> orinMounts = new ArrayList<>();
    private final List<Transform3d> chassisLimelightMounts = new ArrayList<>();

    /** Orin sources built on the main ones: gyro-held PnP and tx/ty (issue #10, 3 and 8c). */
    @Getter private final List<PoseSource> orinGyroSources = new ArrayList<>();

    @Getter private Jetson jetson;
    private double autoStartSeconds = Double.NaN;
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
        // Must match the Jetson's and Limelights' layout and the event field (#10, sec. 5).
        tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

        boolean sim = Constants.currentMode == Constants.Mode.SIM;
        boolean replay = Constants.currentMode == Constants.Mode.REPLAY;
        if (sim) {
            simVision = new SimVision(tagLayout, swerve::getSimTruthPose);
        }

        // Limelights: MT1 and MT2 are separate sources so each gets its own shadow track.
        LimelightConfig[] lls = {config.backConfig, config.leftConfig, config.rightConfig};
        for (LimelightConfig llConfig : lls) {
            addLimelight(llConfig, true, mt1Sources, mt2Sources, sim, replay);
            chassisLimelightMounts.add(SimVision.robotToCamera(llConfig));
        }
        // SystemCore's own cameras: logged and shadowed, fused only when enabled on the dashboard.
        for (LimelightConfig llConfig : config.systemCoreCameras) {
            addLimelight(llConfig, false, scMt1Sources, scMt2Sources, sim, replay);
        }

        // Orin cameras. The source is derived: in replay the camera IO is NONE and the logged raw
        // results are solved again.
        List<Jetson.Camera> jetsonCameras = new ArrayList<>();
        for (int i = 0; i < config.orinCameraNames.length; i++) {
            String name = config.orinCameraNames[i];
            Transform3d mount = config.orinRobotToCamera[i];
            OrinCameraIO cameraIo;
            if (replay) {
                cameraIo = OrinCameraIO.NONE;
            } else {
                if (sim) {
                    simVision.addCamera(name, SimVision.thriftiestCam(), mount);
                }
                cameraIo = new PhotonOrinIO(name);
            }
            OrinPoseSourceIO io =
                    new OrinPoseSourceIO(
                            name,
                            cameraIo,
                            new OrinSolver(
                                    mount,
                                    tagLayout,
                                    config.orinCameraFactors[i],
                                    config.orinImageWidth,
                                    config.orinImageHeight),
                            () -> jetson == null ? java.util.Set.of() : jetson.getExcludedTags(),
                            this::orinHeadingAt);
            jetsonCameras.add(new Jetson.Camera(name, io, mount, config.orinCameraInstalled[i]));
            orinIos.add(io);
            orinMounts.add(mount);
            orinSources.add(
                    new PoseSource(
                            orinSourceName(name),
                            io,
                            VisionGates.orinGates(this::secondsSinceAutoStart),
                            VisionGates.ORIN_MODEL,
                            true));
            for (OrinGyroSourceIO.Mode mode : OrinGyroSourceIO.Mode.values()) {
                orinGyroSources.add(
                        new PoseSource(
                                orinSourceName(name)
                                        + (mode == OrinGyroSourceIO.Mode.TXTY ? "/TxTy" : "/Gyro"),
                                new OrinGyroSourceIO(
                                        mode,
                                        name,
                                        io,
                                        mount,
                                        tagLayout,
                                        () ->
                                                jetson == null
                                                        ? java.util.Set.of()
                                                        : jetson.getExcludedTags(),
                                        this::orinFusedPoseAt),
                                VisionGates.orinGyroGates(this::secondsSinceAutoStart),
                                VisionGates.ORIN_MODEL,
                                false));
            }
        }
        jetson =
                new Jetson(
                        replay ? JetsonIO.NONE : new JetsonNTIO(),
                        jetsonCameras,
                        config.orinExcludedTags,
                        config.jetsonInstalled && Constants.hasHardware() && !sim);

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
        allSources.addAll(orinGyroSources); // after orinSources: they read this loop's results
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
        // (The Orin cameras' alerts are in Jetson.)
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

    /** "orin-left" becomes "Orin-Left". */
    private static String orinSourceName(String camera) {
        String side = camera.startsWith("orin-") ? camera.substring("orin-".length()) : camera;
        return "Orin-" + Character.toUpperCase(side.charAt(0)) + side.substring(1);
    }

    @Override
    public String getName() {
        return config.getName();
    }

    /** The fused heading at a capture time for the Orin solver, or null before it is seeded. */
    private Rotation2d orinHeadingAt(double timestampSeconds) {
        return poseHeadingSeeded ? headingAt(timestampSeconds) : null;
    }

    /** The fused pose at a capture time for the gyro-based Orin sources, or null before seeding. */
    private Pose2d orinFusedPoseAt(double timestampSeconds) {
        return poseHeadingSeeded
                ? fusion.sampleAt(timestampSeconds).orElse(fusion.getPose())
                : null;
    }

    /** Seconds since auto started, or infinity outside auto. From replayed robot state. */
    private double secondsSinceAutoStart() {
        return Double.isNaN(autoStartSeconds)
                ? Double.POSITIVE_INFINITY
                : Timer.getTimestamp() - autoStartSeconds;
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
                VisionGates.FUTURE_TIMESTAMP_GATE,
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

        boolean auto = Util.autoMode.getAsBoolean() && !disabled;
        if (auto && Double.isNaN(autoStartSeconds)) {
            autoStartSeconds = now;
        } else if (!auto) {
            autoStartSeconds = Double.NaN;
        }

        // 1. Read every device (the Jetson first: the Orin solves use its excluded-tag list).
        jetson.updateInputs();
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
                        poseHeadingSeeded,
                        swerve.getTilt().tiltDeg());
        for (PoseSource s : allSources) {
            s.process(context);
        }

        // 3. Seed (disabled, or enabled with nothing seeded yet; the gross-heading net while
        // enabled) and fuse (enabled). Seeding comes first so the frame it used can be left out of
        // the fusing below.
        seededThisLoop = null;
        if (disabled) {
            seedWhileDisabled(context);
        } else {
            seedWhileEnabled(context);
            // One seed per loop: only one frame can be kept out of the fusing.
            if (seededThisLoop == null) {
                checkGrossHeadingError(context);
            }
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
        for (PoseSource s : orinGyroSources) {
            applyWithPolicy(s, !disabled);
        }
        applyWithPolicy(questSource, !disabled);

        feedTagEkf();
        // 4. Keep the Limelights and the Quest aligned with the fused pose.
        if (Constants.currentMode != Constants.Mode.REPLAY) {
            sendCameraSettings(now);
        }
        pushHeadingToLimelights();
        resetQuestAfterSeed();
        jetson.periodic(
                context.linearSpeed() < 0.05 && Math.abs(context.robotVelocity().omega) < 0.05);

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
        List<PoseSource.Result> results = withoutSeedFrame(source.getResults());
        if (fuse) {
            for (PoseSource.Result r : results) {
                if (r.accepted()) {
                    lastFusedSeconds = Timer.getTimestamp();
                    break;
                }
            }
        }
        fusion.apply(source, results, fuse);
    }

    /**
     * The frame a seed used this loop, or {@code null}. It has already gone into every track, as
     * the seed, so fusing it again as an ordinary measurement would count its translation twice.
     */
    private PoseObservation seededThisLoop = null;

    /** Seeds every track from {@code o} and remembers it as this loop's seed frame. */
    private void seedFrom(PoseObservation o) {
        fusion.seed(
                o.pose2d(),
                o.timestampSeconds(),
                config.seedXyStdDev,
                Units.degreesToRadians(config.seedThetaStdDevDeg));
        seededThisLoop = o;
    }

    /**
     * {@code results} with this loop's seed frame, if it is among them, marked as not fused ("Used
     * as Seed"). It stays in the list so its disagreement with the fused pose is still logged with
     * the rest.
     */
    private List<PoseSource.Result> withoutSeedFrame(List<PoseSource.Result> results) {
        if (seededThisLoop == null) {
            return results;
        }
        for (int i = 0; i < results.size(); i++) {
            PoseSource.Result r = results.get(i);
            if (r.accepted() && r.observation() == seededThisLoop) {
                List<PoseSource.Result> out = new ArrayList<>(results);
                out.set(i, new PoseSource.Result(r.observation(), "Used as Seed", r.stdDevs()));
                return out;
            }
        }
        return results;
    }

    // =========================================================================
    // Tag EKF (logged only; never moves the robot pose)
    // =========================================================================

    /** Every Orin and chassis-Limelight tag this loop, frame by frame, into the tag EKF. */
    private void feedTagEkf() {
        TagEkf ekf = fusion.getTagEkf();
        Set<Integer> excluded = jetson.getExcludedTags();
        // The 2-D model assumes a level robot.
        boolean tilted = swerve.isTilted();
        long start = System.nanoTime();
        for (int i = 0; i < orinIos.size(); i++) {
            OrinPoseSourceIO io = orinIos.get(i);
            String camera = orinSources.get(i).getName();
            for (PhotonPipelineResult r : io.getSolver().decoded) {
                List<TagSighting> frame = new ArrayList<>();
                for (PhotonTrackedTarget t : r.getTargets()) {
                    if (t.fiducialId < 0 || excluded.contains(t.fiducialId)) {
                        continue;
                    }
                    Optional<Pose3d> tag = tagLayout.getTagPose(t.fiducialId);
                    if (tag.isEmpty()) {
                        continue;
                    }
                    frame.add(
                            new TagSighting(
                                    camera,
                                    r.getTimestampSeconds(),
                                    t.fiducialId,
                                    tag.get().getTranslation(),
                                    orinMounts.get(i),
                                    t.bestCameraToTarget.getTranslation(),
                                    io.getSolver().cameraFactor()));
                }
                addTagFrame(ekf, frame, tilted);
            }
        }
        int stride = PoseSourceInputs.FIDUCIAL_STRIDE;
        for (int i = 0; i < mt1Sources.size(); i++) {
            PoseSourceInputs in = mt1Sources.get(i).getInputs();
            if (in.count() == 0 || in.fiducials.length == 0) {
                continue;
            }
            String camera = mt1Sources.get(i).getName().split("/")[0];
            double timestamp = in.timestamps[in.count() - 1];
            List<TagSighting> frame = new ArrayList<>();
            for (int f = 0; f + stride <= in.fiducials.length; f += stride) {
                int id = (int) in.fiducials[f];
                if (excluded.contains(id)) {
                    continue;
                }
                Optional<Pose3d> tag = tagLayout.getTagPose(id);
                if (tag.isEmpty()) {
                    continue;
                }
                frame.add(
                        new TagSighting(
                                camera,
                                timestamp,
                                id,
                                tag.get().getTranslation(),
                                chassisLimelightMounts.get(i),
                                TagSighting.fromAngles(
                                        in.fiducials[f + 1],
                                        in.fiducials[f + 2],
                                        in.fiducials[f + 3]),
                                1.0));
            }
            addTagFrame(ekf, frame, tilted);
        }
        Logger.recordOutput("Localization/EKF/FeedMicros", (System.nanoTime() - start) / 1000);
    }

    private static void addTagFrame(TagEkf ekf, List<TagSighting> frame, boolean tilted) {
        if (tilted) {
            for (TagSighting t : frame) {
                ekf.skip(t.camera(), "Tilted");
            }
        } else {
            ekf.addFrame(frame);
        }
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
    private void seedWhileDisabled(GateContext c) {
        PoseSource.Result best = bestAcceptedMt1();
        if (best == null) {
            return;
        }
        PoseObservation o = best.observation();
        // A robot rolling after auto, or a single tag, can seed a heading many degrees off.
        boolean still =
                c.linearSpeed() <= config.grossHeadingMaxLinearSpeed
                        && Math.abs(c.robotVelocity().omega) <= config.grossHeadingMaxOmega;
        if (!still || !o.multiTag()) {
            return;
        }
        seedFrom(o);
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
     * heading errors come from, and a wrong seed taken while enabled gives confident, wrong aims.
     * Confirmation (which lets the chassis cameras move on to MegaTag2) is stricter than the
     * disabled version: each counted frame's heading must also agree with the fused heading.
     */
    private void seedWhileEnabled(GateContext c) {
        if (poseSeedConfirmed) {
            return;
        }
        PoseSource.Result best = bestAcceptedMt1();
        if (best == null) {
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
            seedFrom(o);
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
        if (!o.multiTag()) {
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
        // Loops without a new multi-tag frame are common and do not break the hold.
        if (best == null || !best.observation().multiTag()) {
            return;
        }
        if (!still) {
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
            seedFrom(o);
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
     * VisionConfig#pushLimelightMounts} is on) are re-sent. Offseason 71dc7a7.
     *
     * <p>FM uses IMU mode 1: MegaTag2 takes its yaw from {@code SetRobotOrientation} (as in mode 0,
     * "external yaw only", which is standard MegaTag2 and works fine), and the Limelight 4 also
     * seeds its internal IMU from that yaw. A camera left in mode 0 would still produce usable
     * MegaTag2 poses from the pushed heading; mode 1 only matters for the internal IMU.
     *
     * <p>The resend is belt-and-braces and most likely a no-op: NetworkTables does not transmit a
     * write of an unchanged value, and a Limelight that boots after the robot or reboots mid-match
     * gets the current values when it resubscribes anyway. It is kept only because it costs one NT
     * write per camera every two seconds.
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
     * Yaw rate (rad/s) above which the heading is flushed every loop rather than every other loop.
     */
    private static final double HEADING_FLUSH_EVERY_LOOP_RAD_PER_SEC = 0.5;

    /**
     * MegaTag2 solves each frame with the last external yaw the camera received -- the cameras' own
     * IMUs do not fill in between updates -- so a stale heading is heading error, and while turning
     * it becomes MegaTag2 translation error. The heading is written every loop, and a NetworkTables
     * flush sends it straight away: every loop (100 Hz) while the robot turns faster than {@link
     * #HEADING_FLUSH_EVERY_LOOP_RAD_PER_SEC}, where a 10 ms older heading matters, and every other
     * loop (50 Hz, the rate FM's 2026 code flushed at) otherwise, where the heading barely changes
     * and a flush -- which makes the NT server send to every client -- would mostly be traffic.
     * Unflushed loops still go out at NT's normal update rate.
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
        boolean turning =
                Math.abs(swerve.getYawRateRadPerSec()) > HEADING_FLUSH_EVERY_LOOP_RAD_PER_SEC;
        if (turning || (orientationPushes & 1) == 0) {
            NetworkTableInstance.getDefault().flush();
        }
        orientationPushes++;
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

    /** End of auto: auto's 20 s plus a few seconds either side. */
    public static final double AUTO_REWIND_SECONDS = 25;

    /**
     * End of teleop: the most a Limelight keeps. Teleop's 140 s plus the gap after auto, so it also
     * reaches back into the end of auto.
     */
    public static final double TELEOP_REWIND_SECONDS = 165;

    /**
     * Saves the last {@code seconds} of the rewind buffer of each FM Limelight: video plus every
     * frame's results, reviewable frame by frame afterwards. Rewind is a Limelight 4 feature; the
     * camera writes the capture to its own storage, which survives a power cycle (checked
     * 2026-10-16), so the robot can be powered off after a match.
     *
     * <p>Not the SystemCore's own cameras: their "camera storage" is the robot controller's
     * internal disk, which {@link frc.spectrumLib.telemetry.LogStorage} is keeping free.
     *
     * @param seconds how much to save, at most 165
     */
    public void triggerRewindCaptureForAllCameras(double seconds) {
        if (!Constants.hasHardware() || simVision != null) {
            return;
        }
        for (Limelight limelight : limelights) {
            if (limelight.getName().startsWith("limelightsc")) {
                continue;
            }
            LimelightHelpers.triggerRewindCapture(limelight.getName(), seconds);
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
