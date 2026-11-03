package frc.robot.subsystems.vision;

import frc.robot.Event;
import frc.robot.subsystems.swerve.SwerveConfig;
import frc.robot.subsystems.swerve.TiltState;
import frc.spectrumLib.localization.OdometryGuard;
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
    record Frame(double t, boolean orin, List<TagSighting> tags) implements Ev {}

    /** A multi-tag MT1 pose: seeds the filter, then checks it for lost. */
    record Ref(double t, Pose2d pose) implements Ev {}

    record Log(String name, List<Ev> events) {}

    /** What one noise model scored on one or more logs. */
    static final class Score {
        final List<Double> err = new ArrayList<>(); // held-out tag position error, m
        double nisSum;
        int nisN;
        int refs;
        int lost;
        long fedAccepted;
        long fedRejected;

        void add(Score o) {
            err.addAll(o.err);
            nisSum += o.nisSum;
            nisN += o.nisN;
            fedAccepted += o.fedAccepted;
            fedRejected += o.fedRejected;
            refs += o.refs;
            lost += o.lost;
        }

        double lostShare() {
            return (double) lost / Math.max(1, refs);
        }

        double pct(double q) {
            if (err.isEmpty()) {
                return Double.NaN;
            }
            double[] e = err.stream().mapToDouble(Double::doubleValue).sorted().toArray();
            return e[(int) Math.min(e.length - 1, q * e.length)];
        }

        @Override
        public String toString() {
            return String.format(
                    "held-out p50 %5.3f p90 %6.3f m  NIS %5.2f  fed accept %4.1f%%  lost %4.1f%%  n %d",
                    pct(0.5),
                    pct(0.9),
                    nisSum / Math.max(1, nisN),
                    100.0 * fedAccepted / Math.max(1, fedAccepted + fedRejected),
                    100 * lostShare(),
                    err.size());
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
            Transform3d mount = vc.orinRobotToCamera[i];
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
        }
    }

    Log load(Path p) throws IOException {
        List<Ev> ev = new ArrayList<>();
        HexFormat hex = HexFormat.of();
        for (String line : Files.readAllLines(p)) {
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
                        ev.add(new Frame(t, false, tags));
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
                    for (PhotonTrackedTarget tt : r.getTargets()) {
                        var tag = layout.getTagPose(tt.fiducialId).map(Pose3d::getTranslation);
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
                    if (!tags.isEmpty()) {
                        ev.add(new Frame(capture, true, tags));
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
        TagEkf ekf = new TagEkf(modules, noise);
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
                    ekf.setOdometryNoiseScale(
                            tilt.tilted() || tilt.bumpedRecently(v.t()) ? 10.0 : 1.0);
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
                    double s = tilt.odometryScale() * wheelScale;
                    for (int m = 0; m < 4; m++) {
                        double raw = p[m].distance;
                        scaled[m] = haveDistances ? scaled[m] + (raw - lastRaw[m]) * s : raw;
                        lastRaw[m] = raw;
                        positions[m] = new SwerveModulePosition(scaled[m], p[m].angle);
                    }
                    haveDistances = true;
                    ekf.addOdometry(o.t(), out.get().heading(), positions.clone());
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
                case Frame f -> {
                    if (!haveDistances || !ekf.isSeeded()) {
                        continue;
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
                        }
                    } else if (sources.equals("all") || f.orin() == sources.equals("orin")) {
                        long a = ekf.getAcceptedCount();
                        long rj = ekf.getRejectedCount();
                        Vision.addTagFrame(ekf, f.tags(), tilt.tiltDeg(), skipDeg);
                        sc.fedAccepted += ekf.getAcceptedCount() - a;
                        sc.fedRejected += ekf.getRejectedCount() - rj;
                    }
                }
            }
        }
        if (TRACE.equals(log.name())) {
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
        return sc;
    }

    Score runAll(List<Log> logs, Noise noise, double wheelScale) {
        return runAll(logs, noise, wheelScale, skipDeg);
    }

    double skipDeg = Vision.EKF_SKIP_TILT_DEG;

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
