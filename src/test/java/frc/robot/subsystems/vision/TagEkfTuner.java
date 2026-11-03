package frc.robot.subsystems.vision;

import frc.robot.Event;
import frc.robot.subsystems.swerve.SwerveConfig;
import frc.robot.subsystems.swerve.TiltState;
import frc.spectrumLib.localization.CornerSighting;
import frc.spectrumLib.localization.MountEstimator;
import frc.spectrumLib.localization.OdometryGuard;
import frc.spectrumLib.localization.SlipDetector;
import frc.spectrumLib.localization.TagEkf;
import frc.spectrumLib.localization.TagEkf.Noise;
import frc.spectrumLib.localization.TagSighting;
import frc.spectrumLib.vision.Limelight.LimelightConfig;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HexFormat;
import java.util.List;
import java.util.Map;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.wpilib.fields.Field;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.kinematics.SwerveModulePosition;

/**
 * Replays logged odometry and tags (Limelight MT1 fiducials and Orin PhotonLib results) through
 * {@link TagEkf} with different noise models and scores each on tags it never saw: every {@link
 * #HOLDOUT}th frame is held out, and its tags' innovations against the filter at their capture time
 * are the score. Odometry goes through {@link OdometryGuard}, the tilt scale and the bump noise
 * scale as in {@code Swerve}, so a CAN drop does what it did on the robot.
 *
 * <p>Scored: held-out position error (p50, p90), NIS (2 is honest: the innovation is two numbers),
 * share of fed tags accepted, and share of multi-tag MT1 poses the estimate was over {@link
 * #LOST_M} from ("lost").
 *
 * <pre>
 * python tools/export_tagekf_events.py robotlogs/X.wpilog ...      # once per log
 * ./gradlew tagEkfTune -Plogs=robotlogs/TXMAN1-8515 [-Ptrace=8515_Test_r19] [-Psweep=false]
 * </pre>
 *
 * Desktop tool, not a test: it needs the exported logs, which are not in git.
 */
public final class TagEkfTuner {

    static final int HOLDOUT = 4;
    static final double LOST_M = 1.0;
    static final String TRACE = System.getProperty("tuner.trace", "");

    /**
     * Simulated camera outage: no tags are fed for {@code BLACKOUT_ON} s of every {@code
     * BLACKOUT_PERIOD} s (held-out tags are still scored, so the score is the drift). 0 = off.
     */
    static final double BLACKOUT_ON = Double.parseDouble(System.getProperty("tuner.blackout", "0"));

    static final double BLACKOUT_PERIOD = 20;

    /**
     * Logs reported but not tuned on. Q30: LL-Back's MT1 pose sits 172 deg and 0.7 m from both
     * LL-Left and LL-Right (they agree within 0.1 m in every other log), so that camera was mounted
     * or configured differently and today's mounts can't replay it.
     */
    static final java.util.Set<String> EXCLUDE = java.util.Set.of("8515_Q30");

    sealed interface Ev permits Odom, Daq, Gyro, Tilt, Frame, Ref {
        double t();
    }

    record Odom(double t, double yaw, double[] dist, double[] ang) implements Ev {}

    record Daq(double t) implements Ev {}

    record Gyro(double t, boolean ok) implements Ev {}

    record Tilt(
            double t,
            double pitch,
            double roll,
            double pitchRate,
            double rollRate,
            double ax,
            double ay,
            double az)
            implements Ev {}

    /** A frame, ordered by capture time; {@code orin} for the per-source split. */
    record Frame(double t, boolean orin, List<TagSighting> tags, List<CornerSighting> corners)
            implements Ev {}

    static final double TAG_SIZE_M = Vision.TAG_SIZE_M;

    /**
     * Orin frames fed as corner pixels instead of PhotonVision's per-tag bearing and range, at this
     * corner std (pixels); 0 = bearing and range.
     */
    double orinCornerPx = 0;

    /** Online mount estimation time constant, s; 0 = off. */
    double mountTau = Double.parseDouble(System.getProperty("tuner.mounttau", "0"));

    static final double MOUNT_MAX_RMS_PX = 40;

    static final double MOUNT_KNOCK_DEG = 2.5;

    /** Knock injection: this camera's mount turns this much, this long after its first frame. */
    static final String KNOCK_CAMERA = "Orin-Left";

    static final double KNOCK_DEG = Double.parseDouble(System.getProperty("tuner.knock", "0"));

    static final double KNOCK_AFTER_SECONDS = 60;

    /** Orin sighting camera name -> configured mount (with -Pmountfix). */
    private final Map<String, Transform3d> orinBase = new HashMap<>();

    /** The frame's tags and corners with a different mount. */
    static Frame remount(Frame f, Transform3d m) {
        List<TagSighting> tags = new ArrayList<>(f.tags().size());
        for (TagSighting t : f.tags()) {
            tags.add(
                    new TagSighting(
                            t.camera(),
                            t.timestampSeconds(),
                            t.tagId(),
                            t.tagField(),
                            m,
                            t.cameraToTag(),
                            t.noiseScale()));
        }
        List<CornerSighting> corners = new ArrayList<>(f.corners().size());
        for (CornerSighting c : f.corners()) {
            corners.add(
                    new CornerSighting(
                            c.camera(),
                            c.timestampSeconds(),
                            c.tagId(),
                            c.fieldCorners(),
                            c.pixels(),
                            m,
                            c.intrinsics(),
                            c.distortion(),
                            c.pixelStd()));
        }
        return new Frame(f.t(), f.orin(), tags, corners);
    }

    /** Odometry: each module's angle as the middle of its swing over the sample. */
    static final boolean MID_ANGLE = Boolean.getBoolean("tuner.midangle");

    /** Odometry noise x STEER_NOISE while any module steers faster than this, rad/s; 0 = off. */
    static final double STEER_RATE = Double.parseDouble(System.getProperty("tuner.steerrate", "0"));

    static final double STEER_NOISE =
            Double.parseDouble(System.getProperty("tuner.steernoise", "10"));

    /** Self-checking uncertainty (TagEkf.setAdaptive); 0 = off. */
    double adaptiveNis = Double.parseDouble(System.getProperty("tuner.adaptive", "0"));

    /** Gain 1, as PoseFusion sets it. */
    double adaptiveGain = adaptiveNis > 0 ? 1 : 0;

    /** Held-out tags are scored against the smoothed pose this long after their capture. */
    static final double SMOOTH_LAG_SECONDS = 0.5;

    /**
     * Orin mount corrections, degrees, camera-local: left yaw, left pitch, right yaw, right pitch
     * (-Pmountfix=a,b,c,d). For testing what calibrated mounts would buy.
     */
    static final double[] MOUNT_FIX =
            Arrays.stream(System.getProperty("tuner.mountfix", "0,0,0,0").split(","))
                    .mapToDouble(Double::parseDouble)
                    .toArray();

    /** The mount with a camera-local yaw (about z) and pitch (about y) added. */
    static Transform3d withFix(Transform3d mount, double[] fix, int at) {
        var delta =
                new org.wpilib.math.geometry.Rotation3d(
                        0, Math.toRadians(fix[at + 1]), Math.toRadians(fix[at]));
        var q = mount.getRotation().getQuaternion().times(delta.getQuaternion());
        return new Transform3d(mount.getTranslation(), new org.wpilib.math.geometry.Rotation3d(q));
    }

    static final boolean CORNER_CHECK = System.getProperty("tuner.mode", "").equals("cornercheck");
    private int cornerChecks = 0;
    private final Map<String, double[]> cornerResid =
            new java.util.concurrent.ConcurrentHashMap<>();

    /** A multi-tag MT1 pose: seeds the filter, then checks it for lost. */
    record Ref(double t, Pose2d pose) implements Ev {}

    record Log(String name, List<Ev> events) {}

    /**
     * Optional inputs beyond tags and odometry, each off at 0.
     *
     * @param slipThreshold SlipDetector acceleration gap, m/s²
     * @param slipHold SlipDetector hold, s
     * @param slipNoise odometry noise multiplier while slipping
     * @param bumpStd FieldPriors bump-crossing std, m
     * @param constrainStd FieldPriors wall and hub std, m
     * @param halfSize FieldPriors robot half-size, m
     */
    record Priors(
            double slipThreshold,
            double slipHold,
            double slipNoise,
            double bumpStd,
            double constrainStd,
            double halfSize) {
        static final Priors NONE = new Priors(0, 0, 1, 0, 0, 0.4);

        Priors slip(double threshold, double hold, double noise) {
            return new Priors(threshold, hold, noise, bumpStd, constrainStd, halfSize);
        }

        Priors bump(double std) {
            return new Priors(slipThreshold, slipHold, slipNoise, std, constrainStd, halfSize);
        }

        Priors constrain(double std, double half) {
            return new Priors(slipThreshold, slipHold, slipNoise, bumpStd, std, half);
        }
    }

    /**
     * The Pigeon's X/Y acceleration axes sit ~175 deg from the robot's (fit to the wheels on all 10
     * TXMAN1 logs) although MountPoseYaw is 0: it's mounted turned around. Yaw doesn't care.
     */
    static final double PIGEON_ACCEL_YAW_DEG = 180;

    /** What one noise model scored on one or more logs. */
    static final class Score {
        final List<Double> err = new ArrayList<>(); // held-out tag position error, m
        final List<Double> errSmooth = new ArrayList<>(); // same, against the smoothed pose
        double nisSmoothSum;
        int nisSmoothN;
        double nisSum;
        int nisN;
        int refs;
        int lost;
        long fedAccepted;
        long fedRejected;
        long slipLoops;
        long steerSamples;
        long knockedFrames;
        double knockDetectedAfter = Double.NaN;
        final Map<String, String> mounts = new java.util.TreeMap<>();
        long bumpFixes;
        long constraints;

        void add(Score o) {
            err.addAll(o.err);
            errSmooth.addAll(o.errSmooth);
            nisSmoothSum += o.nisSmoothSum;
            nisSmoothN += o.nisSmoothN;
            nisSum += o.nisSum;
            nisN += o.nisN;
            fedAccepted += o.fedAccepted;
            fedRejected += o.fedRejected;
            refs += o.refs;
            lost += o.lost;
            slipLoops += o.slipLoops;
            bumpFixes += o.bumpFixes;
            constraints += o.constraints;
        }

        double lostShare() {
            return (double) lost / Math.max(1, refs);
        }

        double pct(double q) {
            return pct(err, q);
        }

        static double pct(List<Double> list, double q) {
            if (list.isEmpty()) {
                return Double.NaN;
            }
            double[] e = list.stream().mapToDouble(Double::doubleValue).sorted().toArray();
            return e[(int) Math.min(e.length - 1, q * e.length)];
        }

        String smoothed() {
            return String.format(
                    "smoothed p50 %5.3f p90 %6.3f m  NIS %5.2f",
                    pct(errSmooth, 0.5),
                    pct(errSmooth, 0.9),
                    nisSmoothSum / Math.max(1, nisSmoothN));
        }

        @Override
        public String toString() {
            return String.format(
                    "held-out p50 %5.3f p90 %6.3f m  NIS %5.2f  fed accept %4.1f%%  lost %4.1f%%  n %d"
                            + "  slip loops %d  bump fixes %d  constraints %d",
                    pct(0.5),
                    pct(0.9),
                    nisSum / Math.max(1, nisN),
                    100.0 * fedAccepted / Math.max(1, fedAccepted + fedRejected),
                    100 * lostShare(),
                    err.size(),
                    slipLoops,
                    bumpFixes,
                    constraints);
        }
    }

    private final Translation2d[] modules;
    private final OdometryGuard.Settings guardSettings;
    private final Map<String, Transform3d> mounts = new HashMap<>();
    private final Map<String, OrinSolver> orins = new HashMap<>();
    private final Map<String, String> orinNames = new HashMap<>();
    private final Field layout = Field.loadField(Event.APRILTAG_FIELD);

    TagEkfTuner() {
        SwerveConfig sc = new SwerveConfig();
        modules =
                new Translation2d[] {
                    new Translation2d(sc.getFrontLeftXPos(), sc.getFrontLeftYPos()),
                    new Translation2d(sc.getFrontRightXPos(), sc.getFrontRightYPos()),
                    new Translation2d(sc.getBackLeftXPos(), sc.getBackLeftYPos()),
                    new Translation2d(sc.getBackRightXPos(), sc.getBackRightYPos())
                };
        // As Swerve builds it.
        guardSettings =
                new OdometryGuard.Settings(
                        1.5 * sc.getLinearSpeedAt12Volts().baseUnitMagnitude(),
                        1.5 * sc.getAngularSpeedAt12Volts().baseUnitMagnitude(),
                        modules);
        Vision.VisionConfig vc = new Vision.VisionConfig();
        for (LimelightConfig c : List.of(vc.backConfig, vc.leftConfig, vc.rightConfig)) {
            // Log names: limelight-back -> LL-Back.
            String n = c.getName().replace("limelight-", "");
            mounts.put(
                    "LL-" + Character.toUpperCase(n.charAt(0)) + n.substring(1),
                    SimVision.robotToCamera(c));
        }
        for (int i = 0; i < vc.orinCameraNames.length; i++) {
            String name = vc.orinCameraNames[i];
            Transform3d mount = withFix(vc.orinRobotToCamera[i], MOUNT_FIX, 2 * i);
            mounts.put(name, mount);
            orins.put(
                    name,
                    new OrinSolver(
                            mount,
                            layout,
                            vc.orinCameraFactors[i],
                            vc.orinImageWidth,
                            vc.orinImageHeight));
            String side = name.substring("orin-".length());
            orinNames.put(
                    name, "Orin-" + Character.toUpperCase(side.charAt(0)) + side.substring(1));
            orinBase.put(orinNames.get(name), mount);
        }
    }

    Log load(Path p) throws IOException {
        List<Ev> ev = new ArrayList<>();
        HexFormat hex = HexFormat.of();
        List<String> lines = Files.readAllLines(p);
        Map<String, double[]> intrinsics = new HashMap<>();
        for (String line : lines) {
            if (line.startsWith("K ")) {
                String[] f = line.split(" ");
                intrinsics.put(f[1], parse(f, 2));
            }
        }
        for (String line : lines) {
            String[] f = line.split(" ");
            switch (f[0]) {
                case "O" -> {
                    double[] v = parse(f, 1);
                    ev.add(
                            new Odom(
                                    v[0],
                                    v[1],
                                    Arrays.copyOfRange(v, 2, 6),
                                    Arrays.copyOfRange(v, 6, 10)));
                }
                case "D" -> ev.add(new Daq(Double.parseDouble(f[1])));
                case "G" -> ev.add(new Gyro(Double.parseDouble(f[1]), f[2].equals("1")));
                case "T" -> {
                    double[] v = parse(f, 1);
                    ev.add(new Tilt(v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7]));
                }
                case "P" -> {
                    double[] v = parse(f, 2);
                    ev.add(new Ref(v[0], new Pose2d(v[1], v[2], Rotation2d.fromRadians(v[3]))));
                }
                case "F" -> {
                    Transform3d mount = mounts.get(f[1]);
                    double t = Double.parseDouble(f[2]);
                    int n = Integer.parseInt(f[3]);
                    List<TagSighting> tags = new ArrayList<>();
                    for (int i = 0; i < n && mount != null; i++) {
                        int id = Integer.parseInt(f[4 + 4 * i]);
                        var tag = layout.getTagPose(id).map(Pose3d::getTranslation);
                        if (tag.isPresent()) {
                            tags.add(
                                    new TagSighting(
                                            f[1],
                                            t,
                                            id,
                                            tag.get(),
                                            mount,
                                            TagSighting.fromAngles(
                                                    Double.parseDouble(f[5 + 4 * i]),
                                                    Double.parseDouble(f[6 + 4 * i]),
                                                    Double.parseDouble(f[7 + 4 * i])),
                                            1.0));
                        }
                    }
                    if (!tags.isEmpty()) {
                        ev.add(new Frame(t, false, tags, List.of()));
                    }
                }
                case "R" -> {
                    OrinSolver solver = orins.get(f[1]);
                    if (solver == null) {
                        break;
                    }
                    var maybe = solver.decode(hex.parseHex(f[3]));
                    if (maybe.isEmpty()) {
                        break;
                    }
                    // As Vision.feedTagEkf builds Orin sightings.
                    PhotonPipelineResult r = maybe.get();
                    // TXMAN1's PhotonVision sent microseconds in the field this PhotonLib calls
                    // captureTimestampNanos (it matches loop time read as micros, not as nanos).
                    double capture = r.metadata.captureTimestampNanos / 1e6;
                    List<TagSighting> tags = new ArrayList<>();
                    List<CornerSighting> corners = new ArrayList<>();
                    double[] k = intrinsics.get(f[1]);
                    for (PhotonTrackedTarget tt : r.getTargets()) {
                        var tag = layout.getTagPose(tt.fiducialId).map(Pose3d::getTranslation);
                        var tagPose = layout.getTagPose(tt.fiducialId);
                        if (k != null
                                && tt.fiducialId >= 0
                                && tagPose.isPresent()
                                && tt.getDetectedCorners().size() == 4) {
                            double[][] px = new double[4][];
                            for (int i = 0; i < 4; i++) {
                                px[i] =
                                        new double[] {
                                            tt.getDetectedCorners().get(i).x,
                                            tt.getDetectedCorners().get(i).y
                                        };
                            }
                            corners.add(
                                    new CornerSighting(
                                            orinNames.get(f[1]),
                                            capture,
                                            tt.fiducialId,
                                            CornerSighting.tagCorners(tagPose.get(), TAG_SIZE_M),
                                            px,
                                            mounts.get(f[1]),
                                            Arrays.copyOfRange(k, 0, 4),
                                            Arrays.copyOfRange(k, 4, k.length),
                                            1.0));
                        }
                        if (tt.fiducialId >= 0 && tag.isPresent()) {
                            tags.add(
                                    new TagSighting(
                                            orinNames.get(f[1]),
                                            capture,
                                            tt.fiducialId,
                                            tag.get(),
                                            mounts.get(f[1]),
                                            tt.bestCameraToTarget.getTranslation(),
                                            solver.cameraFactor()));
                        }
                    }
                    if (CORNER_CHECK && k != null && cornerChecks < 5) {
                        for (PhotonTrackedTarget tt : r.getTargets()) {
                            if (tt.getDetectedCorners().size() != 4 || cornerChecks >= 5) {
                                continue;
                            }
                            cornerChecks++;
                            CornerSighting probe =
                                    new CornerSighting(
                                            "",
                                            0,
                                            0,
                                            null,
                                            null,
                                            null,
                                            Arrays.copyOfRange(k, 0, 4),
                                            Arrays.copyOfRange(k, 4, k.length),
                                            1);
                            Transform3d ct = tt.bestCameraToTarget;
                            StringBuilder b = new StringBuilder();
                            double h = TAG_SIZE_M / 2;
                            double[][] yz = {{-h, -h}, {h, -h}, {h, h}, {-h, h}};
                            for (double[] c : yz) {
                                Translation3d inCam =
                                        new Translation3d(0, c[0], c[1])
                                                .rotateBy(ct.getRotation())
                                                .plus(ct.getTranslation());
                                double[] q = probe.projectCamera(inCam);
                                b.append(
                                        String.format(
                                                " (%.0f,%.0f)",
                                                q == null ? -1 : q[0], q == null ? -1 : q[1]));
                            }
                            b.append("  detected");
                            for (var c : tt.getDetectedCorners()) {
                                b.append(String.format(" (%.0f,%.0f)", c.x, c.y));
                            }
                            System.out.println(
                                    "  " + f[1] + " tag " + tt.fiducialId + " predicted" + b);
                        }
                    }
                    if (!tags.isEmpty()) {
                        ev.add(new Frame(capture, true, tags, corners));
                    }
                }
                default -> {}
            }
        }
        // Frames by capture time, so held-out tags meet the filter at the moment they saw.
        // TagEkf inserts by capture time and replays, so this is the order it ends up in anyway.
        ev.sort(Comparator.comparingDouble(Ev::t));
        return new Log(p.getFileName().toString().split("_akit")[0], ev);
    }

    private static double[] parse(String[] f, int from) {
        double[] v = new double[f.length - from];
        for (int i = from; i < f.length; i++) {
            v[i - from] = Double.parseDouble(f[i]);
        }
        return v;
    }

    /**
     * Runs one log through a fresh filter. {@code wheelScale} multiplies module travel; {@code
     * sources} is "all", "ll" or "orin" (which frames are fed; held-out frames are always all).
     */
    Score run(Log log, Noise noise, double wheelScale, String sources) {
        return run(log, noise, wheelScale, sources, Vision.EKF_SKIP_TILT_DEG);
    }

    /** {@code skipDeg}: Vision.addTagFrame's tilt policy. */
    Score run(Log log, Noise noise, double wheelScale, String sources, double skipDeg) {
        return run(log, noise, wheelScale, sources, skipDeg, Priors.NONE);
    }

    /** {@code priors}: the optional slip, bump and wall/hub inputs. */
    Score run(
            Log log,
            Noise noise,
            double wheelScale,
            String sources,
            double skipDeg,
            Priors priors) {
        SlipDetector slip =
                priors.slipThreshold() > 0
                        ? new SlipDetector(priors.slipThreshold(), priors.slipHold())
                        : null;
        FieldPriors field =
                new FieldPriors(priors.bumpStd(), priors.constrainStd(), priors.halfSize());
        double yaw = Math.toRadians(PIGEON_ACCEL_YAW_DEG);
        double wheelVx = 0;
        double wheelVy = 0;
        double lastOdomT = Double.NaN;
        double[] lastGuarded = null;
        TagEkf ekf = new TagEkf(modules, noise);
        ekf.setAdaptive(adaptiveNis, adaptiveGain);
        java.util.ArrayDeque<TagSighting> pending = new java.util.ArrayDeque<>();
        Rotation2d[] prevAngle = new Rotation2d[4];
        double prevAngleT = Double.NaN;
        double[] loopNoiseScale = {1.0};
        MountEstimator mountEst =
                mountTau > 0
                        ? new MountEstimator(mountTau, MOUNT_MAX_RMS_PX, MOUNT_KNOCK_DEG, 20)
                        : null;
        double knockAt = Double.NaN;
        if (KNOCK_DEG != 0) {
            for (Ev e : log.events()) {
                if (e instanceof Frame fr && fr.orin()) {
                    knockAt = fr.t() + KNOCK_AFTER_SECONDS;
                    break;
                }
            }
        }
        OdometryGuard guard = new OdometryGuard(guardSettings);
        TiltState tilt = new TiltState();
        Score sc = new Score();
        boolean gyroOk = true;
        double daqAt = Double.NaN;
        boolean haveDistances = false;
        double[] scaled = new double[4];
        double[] lastRaw = new double[4];
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        int frameIndex = 0;
        double nextTrace = 0;
        for (Ev e : log.events()) {
            switch (e) {
                case Gyro g -> gyroOk = g.ok();
                case Daq d -> daqAt = d.t();
                case Tilt v -> {
                    tilt.update(
                            v.t(),
                            v.pitch(),
                            v.roll(),
                            v.pitchRate(),
                            v.rollRate(),
                            v.ax(),
                            v.ay(),
                            v.az());
                    double scale = tilt.tilted() || tilt.bumpedRecently(v.t()) ? 10.0 : 1.0;
                    if (slip != null) {
                        double ax = 9.81 * (v.ax() * Math.cos(yaw) - v.ay() * Math.sin(yaw));
                        double ay = 9.81 * (v.ax() * Math.sin(yaw) + v.ay() * Math.cos(yaw));
                        slip.update(v.t(), wheelVx, wheelVy, ax, ay);
                        if (slip.slipping(v.t())) {
                            scale *= priors.slipNoise();
                            sc.slipLoops++;
                        }
                    }
                    loopNoiseScale[0] = scale;
                    ekf.setOdometryNoiseScale(scale);
                    field.update(ekf, v.t(), tilt.tiltDeg());
                }
                case Odom o -> {
                    // Swerve: a loop whose DAQ failed may carry a dead 0. Samples share their
                    // loop's DAQ flag; the exporter puts D on the loop's own time, just before.
                    boolean daqFailed = !Double.isNaN(daqAt) && o.t() - daqAt < 0.02;
                    boolean[] ok = new boolean[4];
                    for (int m = 0; m < 4; m++) {
                        ok[m] = !(daqFailed && (o.dist()[m] == 0 || o.ang()[m] == 0));
                    }
                    var out =
                            guard.update(
                                    new OdometryGuard.Sample(
                                            o.t(),
                                            o.yaw(),
                                            gyroOk && !(daqFailed && o.yaw() == 0),
                                            0,
                                            false,
                                            o.dist(),
                                            o.ang(),
                                            ok));
                    if (out.isEmpty()) {
                        continue;
                    }
                    SwerveModulePosition[] p = out.get().positions();
                    if (lastGuarded != null && o.t() > lastOdomT) {
                        // Robot-frame chassis velocity: mean module displacement over dt.
                        double dt = o.t() - lastOdomT;
                        double vx = 0;
                        double vy = 0;
                        for (int m = 0; m < 4; m++) {
                            double dd = p[m].distance - lastGuarded[m];
                            vx += dd * p[m].angle.getCos();
                            vy += dd * p[m].angle.getSin();
                        }
                        wheelVx = vx / 4 / dt;
                        wheelVy = vy / 4 / dt;
                    }
                    lastGuarded = new double[4];
                    for (int m = 0; m < 4; m++) {
                        lastGuarded[m] = p[m].distance;
                    }
                    lastOdomT = o.t();
                    double s = tilt.odometryScale() * wheelScale;
                    double steerRate = 0;
                    for (int m = 0; m < 4; m++) {
                        double raw = p[m].distance;
                        scaled[m] = haveDistances ? scaled[m] + (raw - lastRaw[m]) * s : raw;
                        lastRaw[m] = raw;
                        Rotation2d angle = p[m].angle;
                        if (prevAngle[m] != null && o.t() > prevAngleT) {
                            steerRate =
                                    Math.max(
                                            steerRate,
                                            Math.abs(angle.minus(prevAngle[m]).getRadians())
                                                    / (o.t() - prevAngleT));
                            if (MID_ANGLE) {
                                // The wheel rolled across the whole swing: use the middle.
                                angle = prevAngle[m].interpolate(p[m].angle, 0.5);
                            }
                        }
                        prevAngle[m] = p[m].angle;
                        positions[m] = new SwerveModulePosition(scaled[m], angle);
                    }
                    prevAngleT = o.t();
                    if (STEER_RATE > 0) {
                        boolean steering = steerRate > STEER_RATE;
                        if (steering) {
                            sc.steerSamples++;
                        }
                        ekf.setOdometryNoiseScale(loopNoiseScale[0] * (steering ? STEER_NOISE : 1));
                    }
                    haveDistances = true;
                    ekf.addOdometry(o.t(), out.get().heading(), positions.clone());
                    while (!pending.isEmpty()
                            && pending.peekFirst().timestampSeconds() + SMOOTH_LAG_SECONDS
                                    <= o.t()) {
                        TagSighting t = pending.pollFirst();
                        double[] in = ekf.innovationSmoothed(t);
                        if (in != null) {
                            sc.errSmooth.add(Math.hypot(in[0] * t.cameraToTag().getNorm(), in[1]));
                            sc.nisSmoothSum += Math.min(in[2], 50);
                            sc.nisSmoothN++;
                        }
                    }
                }
                case Ref r -> {
                    if (!haveDistances) {
                        continue;
                    }
                    if (!ekf.isSeeded()) {
                        ekf.addPose(r.pose(), r.t(), 0.1, 0.05);
                        continue;
                    }
                    Pose2d est = ekf.getPose();
                    double d = est.getTranslation().getDistance(r.pose().getTranslation());
                    sc.refs++;
                    if (d > LOST_M) {
                        sc.lost++;
                    }
                    if (TRACE.equals(log.name()) && r.t() >= nextTrace) {
                        nextTrace = r.t() + 2;
                        System.out.printf(
                                "  t %7.2f  off %6.2f m  std %.2f m  relocalizations %d  accepted %d"
                                        + "  rejected %d%n",
                                r.t(),
                                d,
                                ekf.getStdDevs()[0],
                                ekf.getRelocalizations(),
                                ekf.getAcceptedCount(),
                                ekf.getRejectedCount());
                    }
                }
                case Frame f0 -> {
                    if (!haveDistances || !ekf.isSeeded()) {
                        continue;
                    }
                    Frame f = f0;
                    boolean heldOut = frameIndex % HOLDOUT == 0;
                    if (f0.orin() && (mountEst != null || !Double.isNaN(knockAt))) {
                        // The camera as it is now: knocked after knockAt (simulated by turning the
                        // configured mount the other way), then the online correction on top.
                        String cam = f0.tags().get(0).camera();
                        Transform3d base = orinBase.get(cam);
                        if (cam.equals(KNOCK_CAMERA) && f0.t() >= knockAt) {
                            base = MountEstimator.withFix(base, -KNOCK_DEG, 0);
                            sc.knockedFrames++;
                        }
                        Transform3d m = mountEst == null ? base : mountEst.corrected(cam, base);
                        f = remount(f0, m);
                        if (mountEst != null && !heldOut) {
                            mountEst.update(ekf.getPose(), f.corners());
                            if (cam.equals(KNOCK_CAMERA)
                                    && f0.t() >= knockAt
                                    && Double.isNaN(sc.knockDetectedAfter)
                                    && mountEst.cameras().get(cam).knocked()) {
                                sc.knockDetectedAfter = f0.t() - knockAt;
                            }
                        }
                    }
                    boolean skipped = tilt.tiltDeg() > skipDeg;
                    if (frameIndex++ % HOLDOUT == 0 && !skipped) {
                        for (TagSighting t : f.tags()) {
                            double[] in = ekf.innovation(t);
                            if (in != null) {
                                double range = t.cameraToTag().getNorm();
                                sc.err.add(Math.hypot(in[0] * range, in[1]));
                                // ponytail: capped so one flipped tag doesn't own the mean
                                sc.nisSum += Math.min(in[2], 50);
                                sc.nisN++;
                            }
                            pending.addLast(t);
                        }
                    } else if (BLACKOUT_ON > 0 && f.t() % BLACKOUT_PERIOD < BLACKOUT_ON) {
                        continue;
                    } else if (sources.equals("all") || f.orin() == sources.equals("orin")) {
                        if (TRACE.equals(log.name()) && f.orin()) {
                            Pose2d est = ekf.getPose();
                            for (CornerSighting c : f.corners()) {
                                for (int i = 0; i < 4; i++) {
                                    double[] q =
                                            c.project(
                                                    est.getX(),
                                                    est.getY(),
                                                    est.getRotation().getRadians(),
                                                    c.fieldCorners()[i]);
                                    if (q == null) {
                                        continue;
                                    }
                                    // Nearest detected corner.
                                    double[] best = null;
                                    for (double[] d : c.pixels()) {
                                        if (best == null
                                                || Math.hypot(d[0] - q[0], d[1] - q[1])
                                                        < Math.hypot(
                                                                best[0] - q[0], best[1] - q[1])) {
                                            best = d;
                                        }
                                    }
                                    double[] acc =
                                            cornerResid.computeIfAbsent(
                                                    c.camera(), x -> new double[5]);
                                    acc[0] += best[0] - q[0];
                                    acc[1] += best[1] - q[1];
                                    acc[2] += Math.pow(best[0] - q[0], 2);
                                    acc[3] += Math.pow(best[1] - q[1], 2);
                                    acc[4]++;
                                }
                            }
                        }
                        long a = ekf.getAcceptedCount();
                        long rj = ekf.getRejectedCount();
                        if (f.orin() && orinCornerPx > 0) {
                            if (tilt.tiltDeg() > skipDeg) {
                                continue;
                            }
                            double k = Math.max(1, tilt.tiltDeg() / TiltState.TILTED_DEG);
                            List<CornerSighting> cs = new ArrayList<>();
                            for (CornerSighting c : f.corners()) {
                                cs.add(
                                        new CornerSighting(
                                                c.camera(),
                                                c.timestampSeconds(),
                                                c.tagId(),
                                                c.fieldCorners(),
                                                c.pixels(),
                                                c.robotToCamera(),
                                                c.intrinsics(),
                                                c.distortion(),
                                                orinCornerPx * k));
                            }
                            ekf.addCornerFrame(cs);
                        } else {
                            Vision.addTagFrame(ekf, f.tags(), tilt.tiltDeg(), skipDeg);
                        }
                        sc.fedAccepted += ekf.getAcceptedCount() - a;
                        sc.fedRejected += ekf.getRejectedCount() - rj;
                    }
                }
            }
        }
        if (TRACE.equals(log.name())) {
            cornerResid.forEach(
                    (cam, a) ->
                            System.out.printf(
                                    "  %s corner residual vs estimate: u mean %+.1f rms %.1f px, v mean %+.1f rms %.1f px (n %.0f)%n",
                                    cam,
                                    a[0] / a[4],
                                    Math.sqrt(a[2] / a[4]),
                                    a[1] / a[4],
                                    Math.sqrt(a[3] / a[4]),
                                    a[4]));
            cornerResid.clear();
            System.out.println("  rejections " + sources + ": " + ekf.getRejectionCounts());
            double firstOrin = Double.NaN;
            double firstOdom = Double.NaN;
            for (Ev e : log.events()) {
                if (e instanceof Frame f && f.orin() && Double.isNaN(firstOrin)) {
                    firstOrin = f.t();
                }
                if (e instanceof Odom o && Double.isNaN(firstOdom)) {
                    firstOdom = o.t();
                }
            }
            System.out.printf(
                    "  first odometry %.3f  first Orin frame %.3f%n", firstOdom, firstOrin);
        }
        sc.bumpFixes = field.bumpFixes();
        if (mountEst != null) {
            mountEst.cameras()
                    .forEach(
                            (cam, c) ->
                                    sc.mounts.put(
                                            cam,
                                            String.format(
                                                    "yaw %+.2f pitch %+.2f deg%s",
                                                    c.yawDeg(),
                                                    c.pitchDeg(),
                                                    c.knocked() ? " KNOCKED" : "")));
        }
        sc.constraints = field.constraints();
        return sc;
    }

    Score runAll(List<Log> logs, Noise noise, double wheelScale) {
        return runAll(logs, noise, wheelScale, skipDeg);
    }

    double skipDeg = Vision.EKF_SKIP_TILT_DEG;

    Score runAll(List<Log> logs, Priors priors) {
        Score total = new Score();
        logs.parallelStream()
                .map(l -> run(l, Noise.DEFAULT, 1.0, "all", skipDeg, priors))
                .toList()
                .forEach(total::add);
        return total;
    }

    /** Scores each optional input alone over a small grid, then the best of each together. */
    static void comparePriors(TagEkfTuner tuner, List<Log> logs) {
        Score base = tuner.runAll(logs, Priors.NONE);
        System.out.println(
                "\nPriors (as-built noise), cost " + fmt(cost(base)) + ": none   " + base);
        Priors bestSlip = Priors.NONE;
        Score bestSlipScore = base;
        for (double th : new double[] {2, 3, 4, 6}) {
            for (double hold : new double[] {0.25, 0.5}) {
                for (double k : new double[] {5, 20}) {
                    Priors pr = Priors.NONE.slip(th, hold, k);
                    Score s = tuner.runAll(logs, pr);
                    System.out.printf(
                            "  slip %.0f m/s² hold %.2f s x%-3.0f cost %s  %s%n",
                            th, hold, k, fmt(cost(s)), s);
                    if (cost(s) < cost(bestSlipScore)) {
                        bestSlip = pr;
                        bestSlipScore = s;
                    }
                }
            }
        }
        Priors bestBump = Priors.NONE;
        Score bestBumpScore = base;
        for (double std : new double[] {0.2, 0.35, 0.5, 0.8}) {
            Priors pr = Priors.NONE.bump(std);
            Score s = tuner.runAll(logs, pr);
            System.out.printf("  bump std %.2f m          cost %s  %s%n", std, fmt(cost(s)), s);
            if (cost(s) < cost(bestBumpScore)) {
                bestBump = pr;
                bestBumpScore = s;
            }
        }
        Priors bestWall = Priors.NONE;
        Score bestWallScore = base;
        for (double std : new double[] {0.02, 0.05, 0.1, 0.2}) {
            for (double half : new double[] {0.3, 0.4, 0.45}) {
                Priors pr = Priors.NONE.constrain(std, half);
                Score s = tuner.runAll(logs, pr);
                System.out.printf(
                        "  wall/hub std %.2f half %.2f cost %s  %s%n", std, half, fmt(cost(s)), s);
                if (cost(s) < cost(bestWallScore)) {
                    bestWall = pr;
                    bestWallScore = s;
                }
            }
        }
        Priors all =
                new Priors(
                        bestSlip.slipThreshold(),
                        bestSlip.slipHold(),
                        bestSlip.slipNoise(),
                        bestBump.bumpStd(),
                        bestWall.constrainStd(),
                        bestWall.halfSize());
        Score allScore = tuner.runAll(logs, all);
        System.out.println("\nBest slip:     " + bestSlip + "\n  " + bestSlipScore);
        System.out.println("Best bump:     " + bestBump + "\n  " + bestBumpScore);
        System.out.println("Best wall/hub: " + bestWall + "\n  " + bestWallScore);
        System.out.println(
                "All three:     " + all + "\n  cost " + fmt(cost(allScore)) + "  " + allScore);
        System.out.println("None:          cost " + fmt(cost(base)) + "  " + base);
        System.out.println("Per log, all three vs none:");
        for (Log l : logs) {
            System.out.printf(
                    "  %-14s all  %s%n  %-14s none %s%n",
                    l.name(),
                    tuner.run(l, Noise.DEFAULT, 1.0, "all", tuner.skipDeg, all),
                    "",
                    tuner.run(l, Noise.DEFAULT, 1.0, "all", tuner.skipDeg, Priors.NONE));
        }
    }

    /** Orin frames as bearing and range vs as corner pixels, fed alone and with the Limelights. */
    static void compareCorners(TagEkfTuner tuner, List<Log> logs) {
        for (String src : new String[] {"orin", "all"}) {
            System.out.println("\nFed: " + src);
            tuner.orinCornerPx = 0;
            Score base = tuner.runAllSources(logs, src);
            System.out.printf("  bearing/range      cost %s  %s%n", fmt(cost(base)), base);
            double bestPx = 0;
            Score best = base;
            for (double px : new double[] {2, 8, 16, 32}) {
                tuner.orinCornerPx = px;
                Score s = tuner.runAllSources(logs, src);
                System.out.printf("  corners %4.1f px    cost %s  %s%n", px, fmt(cost(s)), s);
                if (cost(s) < cost(best)) {
                    best = s;
                    bestPx = px;
                }
            }
            System.out.println("  per log, bearing/range vs corners at " + bestPx + " px:");
            for (Log l : logs) {
                tuner.orinCornerPx = 0;
                Score a = tuner.run(l, Noise.DEFAULT, 1.0, src);
                tuner.orinCornerPx = bestPx;
                Score b = tuner.run(l, Noise.DEFAULT, 1.0, src);
                System.out.printf("    %-14s br  %s%n    %-14s cor %s%n", l.name(), a, "", b);
            }
        }
        tuner.orinCornerPx = 0;
    }

    Score runAllSources(List<Log> logs, String src) {
        Score total = new Score();
        logs.parallelStream()
                .map(l -> run(l, Noise.DEFAULT, 1.0, src))
                .toList()
                .forEach(total::add);
        return total;
    }

    /** Online mount estimation off and on, fed as bearing/range and as corners. */
    static void compareMount(TagEkfTuner tuner, List<Log> logs) {
        boolean quick = Boolean.getBoolean("tuner.quick");
        for (double px : quick ? new double[0] : new double[] {0, 4, 16}) {
            tuner.orinCornerPx = px;
            for (double tau : new double[] {0, 1, 2, 5}) {
                tuner.mountTau = tau;
                Score sc = tuner.runAll(logs, Noise.DEFAULT, 1.0);
                System.out.printf(
                        "  %-15s mount %-9s cost %s  %s%n",
                        px == 0 ? "bearing/range" : "corners " + px + " px",
                        tau == 0 ? "off" : "tau " + tau + " s",
                        fmt(cost(sc)),
                        sc);
            }
        }
        tuner.orinCornerPx = 0;
        tuner.mountTau = 2;
        System.out.println(
                "Per log, bearing/range, mount tau 2 s: final corrections"
                        + (KNOCK_DEG != 0
                                ? ", knock " + KNOCK_DEG + " deg on " + KNOCK_CAMERA
                                : ""));
        for (Log l : logs) {
            Score sc = tuner.run(l, Noise.DEFAULT, 1.0, "all");
            String knock = "";
            if (KNOCK_DEG != 0) {
                knock =
                        String.format(
                                "  knocked frames %d, detected after %s",
                                sc.knockedFrames,
                                Double.isNaN(sc.knockDetectedAfter)
                                        ? "never"
                                        : String.format("%.1f s", sc.knockDetectedAfter));
            }
            System.out.printf("  %-14s %s%s%n", l.name(), sc.mounts, knock);
        }
        tuner.mountTau = 0;
    }

    /** Real-time vs smoothed scores, with self-checking uncertainty off and on. */
    static void compareSmooth(TagEkfTuner tuner, List<Log> logs) {
        double[][] settings = {{0, 0}, {4, 0.5}, {4, 2}, {6, 0.5}, {6, 2}, {10, 1}, {10, 4}};
        for (double[] st : settings) {
            tuner.adaptiveNis = st[0];
            tuner.adaptiveGain = st[1];
            Score sc = tuner.runAll(logs, Noise.DEFAULT, 1.0);
            System.out.printf(
                    "  adaptive %s  cost %s  real-time %s%n%49s%s%n",
                    st[0] == 0
                            ? "off          "
                            : String.format("NIS>%-3.0f x%-4.1f", st[0], st[1]),
                    fmt(cost(sc)),
                    sc,
                    "",
                    sc.smoothed());
        }
        tuner.adaptiveNis = 0;
        tuner.adaptiveGain = 0;
    }

    static String fmt(double v) {
        return String.format("%.4f", v);
    }

    Score runAll(List<Log> logs, Noise noise, double wheelScale, double sd) {
        Score total = new Score();
        logs.parallelStream()
                .map(l -> run(l, noise, wheelScale, "all", sd))
                .toList()
                .forEach(total::add);
        return total;
    }

    static final List<String> FIELDS =
            List.of(
                    "odomVarPerMeter",
                    "slipVarPerMeter",
                    "slipYawWeight",
                    "headingVarPerSecond",
                    "headingVarPerRadian",
                    "bearingStdRad",
                    "rangeStdBaseM",
                    "rangeStdPerM2",
                    "timingStdSeconds",
                    "gateChi2");

    static double[] values(Noise n) {
        return new double[] {
            n.odomVarPerMeter(),
            n.slipVarPerMeter(),
            n.slipYawWeight(),
            n.headingVarPerSecond(),
            n.headingVarPerRadian(),
            n.bearingStdRad(),
            n.rangeStdBaseM(),
            n.rangeStdPerM2(),
            n.timingStdSeconds(),
            n.gateChi2()
        };
    }

    static Noise with(Noise n, int field, double v) {
        double[] a = values(n);
        a[field] = v;
        return new Noise(a[0], a[1], a[2], a[3], a[4], a[5], a[6], a[7], a[8], a[9]);
    }

    /** Multipliers tried on each field per pass; slipYawWeight is absolute. */
    static final double[] FACTORS = {0.25, 0.5, 2, 4};

    static final double[] WHEEL_SCALES = {0.78, 0.82, 0.86, 0.90, 0.94, 0.98, 1.0, 1.02};

    public static void main(String[] args) throws IOException {
        Path dir = Path.of(args.length > 0 ? args[0] : "robotlogs/TXMAN1-8515");
        TagEkfTuner tuner = new TagEkfTuner();
        List<Log> logs = new ArrayList<>();
        try (var s = Files.list(dir)) {
            for (Path p : s.filter(p -> p.toString().endsWith(".tagekf.txt")).sorted().toList()) {
                logs.add(tuner.load(p));
            }
        }
        if (logs.isEmpty()) {
            System.out.println(
                    "No .tagekf.txt in " + dir + "; run tools/export_tagekf_events.py first.");
            return;
        }
        for (Log l : logs) {
            if (EXCLUDE.contains(l.name())) {
                System.out.printf(
                        "Excluded %s, as built: %s%n",
                        l.name(), tuner.run(l, Noise.DEFAULT, 1.0, "all"));
            }
        }
        logs = logs.stream().filter(l -> !EXCLUDE.contains(l.name())).toList();
        System.out.println(
                logs.size() + " logs. As built, per log (all / Limelight only / Orin only fed):");
        for (Log l : logs) {
            System.out.printf(
                    "  %-14s all   %s%n", l.name(), tuner.run(l, Noise.DEFAULT, 1.0, "all"));
            if (TRACE.isEmpty() || TRACE.equals(l.name())) {
                System.out.printf("  %-14s ll    %s%n", "", tuner.run(l, Noise.DEFAULT, 1.0, "ll"));
                System.out.printf(
                        "  %-14s orin  %s%n", "", tuner.run(l, Noise.DEFAULT, 1.0, "orin"));
            }
        }
        Score before = tuner.runAll(logs, Noise.DEFAULT, 1.0, TiltState.TILTED_DEG);
        System.out.println("All logs, as built before the tilt change (skip > 5 deg): " + before);
        Score asBuilt = tuner.runAll(logs, Noise.DEFAULT, 1.0);
        System.out.println("All logs, as built: " + asBuilt);
        if (CORNER_CHECK) {
            return;
        }
        if (System.getProperty("tuner.mode", "").equals("mount")) {
            compareMount(tuner, logs);
            return;
        }
        if (System.getProperty("tuner.mode", "").equals("smooth")) {
            compareSmooth(tuner, logs);
            return;
        }
        if (System.getProperty("tuner.mode", "").equals("corners")) {
            compareCorners(tuner, logs);
            return;
        }
        if (System.getProperty("tuner.mode", "").equals("priors")) {
            comparePriors(tuner, logs);
            return;
        }
        if (!Boolean.parseBoolean(System.getProperty("tuner.sweep", "true"))) {
            return;
        }

        // Coordinate descent over the wheel scale and every noise field, three passes.
        Noise best = Noise.DEFAULT;
        double wheel = 1.0;
        Score bestScore = asBuilt;
        for (int pass = 0; pass < 3; pass++) {
            for (double ws : WHEEL_SCALES) {
                Score s = tuner.runAll(logs, best, ws);
                if (cost(s) < cost(bestScore)) {
                    wheel = ws;
                    bestScore = s;
                    System.out.printf("  pass %d %-20s -> %-10.4g %s%n", pass, "wheelScale", ws, s);
                }
            }
            for (double sd : new double[] {5, 8, 10, 15, 20, 90}) {
                Score s = tuner.runAll(logs, best, wheel, sd);
                if (cost(s) < cost(bestScore)) {
                    tuner.skipDeg = sd;
                    bestScore = s;
                    System.out.printf(
                            "  pass %d %-20s -> %-10.4g %s%n", pass, "tiltSkipDeg", sd, s);
                }
            }
            for (int f = 0; f < FIELDS.size(); f++) {
                double base = values(best)[f];
                double[] tries =
                        f == FIELDS.indexOf("slipYawWeight")
                                ? new double[] {0, 0.25, 0.5, 1}
                                : Arrays.stream(FACTORS).map(x -> x * base).toArray();
                for (double v : tries) {
                    Noise n = with(best, f, v);
                    Score s = tuner.runAll(logs, n, wheel);
                    if (cost(s) < cost(bestScore)) {
                        best = n;
                        bestScore = s;
                        System.out.printf(
                                "  pass %d %-20s -> %-10.4g %s%n", pass, FIELDS.get(f), v, s);
                    }
                }
            }
        }
        System.out.printf(
                "%nBest: wheel scale %.2f, tilt skip %.0f deg%n  %s%n  %s%n",
                wheel, tuner.skipDeg, best, bestScore);
        System.out.println("Before:    " + before);
        System.out.println("As built:  " + asBuilt);
        System.out.println("Per log, best:");
        for (Log l : logs) {
            System.out.printf(
                    "  %-14s %s%n", l.name(), tuner.run(l, best, wheel, "all", tuner.skipDeg));
        }
    }

    /**
     * Lower is better: held-out p50 and p90 position error, a penalty when NIS strays from 2 (an
     * over- or under-confident filter that scores well here won't generalize), and lost time, which
     * dominates: right 99% of the time and 20 m off the rest is no use.
     */
    static double cost(Score s) {
        double nis = s.nisSum / Math.max(1, s.nisN);
        return s.pct(0.5)
                + 0.5 * s.pct(0.9)
                + 0.02 * Math.abs(Math.log(nis / 2))
                + 2 * s.lostShare();
    }
}
