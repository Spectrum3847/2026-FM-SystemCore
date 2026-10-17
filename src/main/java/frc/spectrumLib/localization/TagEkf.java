package frc.spectrumLib.localization;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.util.MathUtil;

/**
 * An extended Kalman filter over (x, y, heading) that takes every camera's tags one at a time, in
 * the style of 971's AOS localizer: each tag is a bearing and a distance from its camera, so a
 * single or far tag still corrects the pose at a low weight instead of being thrown away, and every
 * camera feeds the same estimate.
 *
 * <ul>
 *   <li><b>Predict</b> on every odometry sample: wheel translation, gyro heading change. Variance
 *       grows with distance driven, and more when the modules disagree with each other or with the
 *       gyro (slip). Heading gets very little.
 *   <li><b>Correct</b> per tag, at the frame's capture time. About {@link #HISTORY_SECONDS} of
 *       states are kept; a frame is inserted at its time and odometry and later frames are replayed
 *       over it, so the result does not depend on arrival order.
 *   <li><b>Reject</b> a tag whose innovation fails a chi-squared test. If tags keep failing for
 *       {@link #RELOCALIZE_SECONDS}, the covariance is widened so the cameras can pull it back.
 * </ul>
 *
 * <p>Noise values are starting points to tune against logs, not measured.
 */
public class TagEkf {

    static final double HISTORY_SECONDS = 1.5;
    private static final int CAPACITY = 1024;

    /** Translation variance per metre driven (m²/m): 3 cm after 1 m. */
    static final double ODOM_VAR_PER_METER = 0.03 * 0.03;

    /** Translation variance per metre of slip (m²/m). */
    static final double SLIP_VAR_PER_METER = 0.05;

    /** Heading random walk (rad²/s): the Pigeon's ~0.4 deg/min. */
    static final double HEADING_VAR_PER_SECOND = sq(Math.toRadians(0.4)) / 60.0;

    /** Heading variance per radian turned (rad²/rad): ~0.3% gyro scale error. */
    static final double HEADING_VAR_PER_RADIAN = 6e-5;

    /** Bearing std-dev on the optical axis; mostly mount-yaw error. */
    static final double BEARING_STD_RAD = Math.toRadians(1.0);

    static final double RANGE_STD_BASE_M = 0.02;
    static final double RANGE_STD_PER_M2 = 0.01;

    /** Std-devs scale by 1 + (angle off the optical axis / this)². */
    static final double OFF_AXIS_REF_RAD = Math.toRadians(35);

    /** Capture-time error, turned into noise by the robot's speed. */
    static final double TIMING_STD_SECONDS = 0.005;

    static final double MIN_RANGE_M = 0.2;
    static final double MAX_RANGE_M = 8.0;

    /** Chi-squared, 2 degrees of freedom, 99.9%. */
    static final double GATE_CHI2 = 13.8;

    static final double RELOCALIZE_SECONDS = 1.0;
    static final int RELOCALIZE_MIN_REJECTS = 20;
    static final double RELOCALIZE_XY_STD_M = 0.5;
    static final double RELOCALIZE_THETA_STD_RAD = Math.toRadians(5);

    static final double RESET_XY_STD_M = 0.05;
    static final double RESET_THETA_STD_RAD = Math.toRadians(1);

    private static final double UNSEEDED_VAR = 1e4;

    /** Per-camera tag counts, and this loop's residuals of the tags that passed. */
    public static final class CameraStats {
        public long accepted;
        public long rejected;

        /** Whether the residual arrays were last logged non-empty, so they're cleared once. */
        boolean loggedResiduals;

        private double[] bearingResidualsDeg = new double[8];
        private double[] rangeResidualsM = new double[8];
        private int loopCount;

        private void add(double bearingDeg, double rangeM) {
            if (loopCount == bearingResidualsDeg.length) {
                bearingResidualsDeg = java.util.Arrays.copyOf(bearingResidualsDeg, loopCount * 2);
                rangeResidualsM = java.util.Arrays.copyOf(rangeResidualsM, loopCount * 2);
            }
            bearingResidualsDeg[loopCount] = bearingDeg;
            rangeResidualsM[loopCount] = rangeM;
            loopCount++;
        }

        public double[] bearingResidualsDeg() {
            return java.util.Arrays.copyOf(bearingResidualsDeg, loopCount);
        }

        public double[] rangeResidualsM() {
            return java.util.Arrays.copyOf(rangeResidualsM, loopCount);
        }

        public int loopCount() {
            return loopCount;
        }
    }

    /** A tag reduced to what the update needs. Robot frame, camera at (cx, cy). */
    private record Measured(
            String camera,
            double bearing,
            double range,
            double cx,
            double cy,
            double tagX,
            double tagY,
            double noiseScale) {}

    private static final class Correction {
        final double time;
        final Measured[] tags;
        final double px;
        final double py;
        final double ptheta;
        final double varXy;
        final double varTheta;
        boolean inflate;

        Correction(double time, Measured[] tags) {
            this(time, tags, 0, 0, 0, 0, 0);
        }

        Correction(
                double time,
                Measured[] tags,
                double px,
                double py,
                double ptheta,
                double varXy,
                double varTheta) {
            this.time = time;
            this.tags = tags;
            this.px = px;
            this.py = py;
            this.ptheta = ptheta;
            this.varXy = varXy;
            this.varTheta = varTheta;
        }
    }

    private final int modules;
    private final double[] moduleX;
    private final double[] moduleY;
    private final double meanModuleRadius;

    /** (AᵀA)⁻¹ for the least-squares chassis twist from module displacements. */
    private final double[] twistNormalInv;

    private final double[] sampleTime = new double[CAPACITY];
    private final double[] sampleDx = new double[CAPACITY];
    private final double[] sampleDy = new double[CAPACITY];
    private final double[] sampleDtheta = new double[CAPACITY];
    private final double[] sampleDt = new double[CAPACITY];
    private final double[] sampleVarXy = new double[CAPACITY];
    private final double[] sampleVarTheta = new double[CAPACITY];

    /** State and covariance after each sample's prediction, before its corrections. */
    private final double[] preState = new double[CAPACITY * 3];

    private final double[] preCov = new double[CAPACITY * 9];
    private int oldest = 0;
    private int size = 0;

    /** Corrections still inside the history, in time order. */
    private final List<Correction> corrections = new ArrayList<>();

    private final double[] s = new double[3];
    private final double[] p = new double[9];

    private boolean haveOdometry = false;
    private double lastGyro;
    private final double[] lastDistances;
    private double odometryNoiseScale = 1.0;

    private boolean seeded = false;

    /** A good frame had at least half its tags pass the gate. */
    private double lastGoodFrameTime = Double.NaN;

    private int rejectsSinceGoodFrame = 0;
    private Correction recording;

    private long acceptedCount = 0;
    private long rejectedCount = 0;
    private long relocalizations = 0;
    private final Map<String, Long> rejectionCounts = new LinkedHashMap<>();
    private final Map<String, CameraStats> cameraStats = new LinkedHashMap<>();

    private final double[] moduleDx;
    private final double[] moduleDy;

    /**
     * @param moduleLocations module positions from the robot centre, robot frame
     */
    public TagEkf(Translation2d[] moduleLocations) {
        modules = moduleLocations.length;
        moduleX = new double[modules];
        moduleY = new double[modules];
        lastDistances = new double[modules];
        moduleDx = new double[modules];
        moduleDy = new double[modules];
        double sx = 0;
        double sy = 0;
        double sr2 = 0;
        double radius = 0;
        for (int i = 0; i < modules; i++) {
            moduleX[i] = moduleLocations[i].getX();
            moduleY[i] = moduleLocations[i].getY();
            sx += moduleX[i];
            sy += moduleY[i];
            sr2 += sq(moduleX[i]) + sq(moduleY[i]);
            radius += moduleLocations[i].getNorm();
        }
        meanModuleRadius = radius / modules;
        twistNormalInv = inverse3(new double[] {modules, 0, -sy, 0, modules, sx, -sy, sx, sr2});
        p[0] = UNSEEDED_VAR;
        p[4] = UNSEEDED_VAR;
        p[8] = UNSEEDED_VAR;
    }

    /**
     * Multiplies odometry translation variance, e.g. while the robot is tilted on a bump.
     *
     * @param scale 1 for normal driving
     */
    public void setOdometryNoiseScale(double scale) {
        odometryNoiseScale = scale;
    }

    /**
     * Predicts with one odometry sample.
     *
     * @param timestampSeconds sample time
     * @param gyro gyro yaw
     * @param positions module positions, same order as the locations
     */
    public void addOdometry(
            double timestampSeconds, Rotation2d gyro, SwerveModulePosition[] positions) {
        double g = gyro.getRadians();
        if (!haveOdometry) {
            haveOdometry = true;
            lastGyro = g;
            for (int i = 0; i < modules; i++) {
                lastDistances[i] = positions[i].distance;
            }
            size = 0;
            pushSample(timestampSeconds, 0, 0, 0, 0, 0, 0);
            return;
        }
        double bx = 0;
        double by = 0;
        double bw = 0;
        double[] dxs = moduleDx;
        double[] dys = moduleDy;
        for (int i = 0; i < modules; i++) {
            double d = positions[i].distance - lastDistances[i];
            lastDistances[i] = positions[i].distance;
            double a = positions[i].angle.getRadians();
            dxs[i] = d * Math.cos(a);
            dys[i] = d * Math.sin(a);
            bx += dxs[i];
            by += dys[i];
            bw += -moduleY[i] * dxs[i] + moduleX[i] * dys[i];
        }
        double[] n = twistNormalInv;
        double vx = n[0] * bx + n[1] * by + n[2] * bw;
        double vy = n[3] * bx + n[4] * by + n[5] * bw;
        double w = n[6] * bx + n[7] * by + n[8] * bw;
        double residual = 0;
        for (int i = 0; i < modules; i++) {
            residual += sq(dxs[i] - (vx - w * moduleY[i])) + sq(dys[i] - (vy + w * moduleX[i]));
        }
        double dGyro = MathUtil.angleModulus(g - lastGyro);
        lastGyro = g;
        double slip = Math.sqrt(residual / modules) + Math.abs(w - dGyro) * meanModuleRadius;

        // Pose exponential with the gyro's heading change, as WPILib's odometry does.
        double sinTerm;
        double cosTerm;
        if (Math.abs(dGyro) < 1e-9) {
            sinTerm = 1.0 - dGyro * dGyro / 6.0;
            cosTerm = 0.5 * dGyro;
        } else {
            sinTerm = Math.sin(dGyro) / dGyro;
            cosTerm = (1 - Math.cos(dGyro)) / dGyro;
        }
        double lx = vx * sinTerm - vy * cosTerm;
        double ly = vx * cosTerm + vy * sinTerm;

        double dt = Math.max(0, timestampSeconds - newestTime());
        double varXy =
                (ODOM_VAR_PER_METER * Math.hypot(lx, ly) + SLIP_VAR_PER_METER * slip)
                        * odometryNoiseScale;
        double varTheta = HEADING_VAR_PER_SECOND * dt + HEADING_VAR_PER_RADIAN * Math.abs(dGyro);
        predict(s, p, lx, ly, dGyro, varXy, varTheta);
        while (size > 0 && sampleTime[oldest] < timestampSeconds - HISTORY_SECONDS) {
            dropOldest();
        }
        if (size == CAPACITY) {
            dropOldest();
        }
        pushSample(timestampSeconds, lx, ly, dGyro, dt, varXy, varTheta);
    }

    /**
     * Hard-resets the pose (auto start, driver reorient). Frames captured before it are dropped.
     *
     * @param pose the new pose
     */
    public void reset(Pose2d pose) {
        s[0] = pose.getX();
        s[1] = pose.getY();
        s[2] = pose.getRotation().getRadians();
        java.util.Arrays.fill(p, 0);
        p[0] = sq(RESET_XY_STD_M);
        p[4] = sq(RESET_XY_STD_M);
        p[8] = sq(RESET_THETA_STD_RAD);
        corrections.clear();
        if (size > 0) {
            int newest = index(size - 1);
            oldest = newest;
            size = 1;
            storePre(newest);
            lastGoodFrameTime = sampleTime[newest];
        }
        seeded = true;
        rejectsSinceGoodFrame = 0;
    }

    /**
     * Applies a whole-pose measurement at its capture time (the seed every track shares).
     *
     * @param pose measured pose
     * @param timestampSeconds capture time
     * @param xyStd translation std-dev, metres
     * @param thetaStd heading std-dev, radians
     */
    public void addPose(Pose2d pose, double timestampSeconds, double xyStd, double thetaStd) {
        Correction c =
                new Correction(
                        clampTime(timestampSeconds),
                        null,
                        pose.getX(),
                        pose.getY(),
                        pose.getRotation().getRadians(),
                        sq(xyStd),
                        sq(thetaStd));
        if (insert(c, "Pose")) {
            if (!seeded) {
                lastGoodFrameTime = c.time;
            }
            seeded = true;
        }
    }

    /**
     * Applies one camera frame's tags. They must share one capture time.
     *
     * @param frame the frame's tags
     */
    public void addFrame(List<TagSighting> frame) {
        if (frame.isEmpty()) {
            return;
        }
        if (!seeded) {
            for (TagSighting t : frame) {
                reject(t.camera(), "Not Seeded");
            }
            return;
        }
        List<Measured> kept = new ArrayList<>(frame.size());
        for (TagSighting t : frame) {
            Measured m = measure(t);
            if (m == null) {
                reject(t.camera(), "Range");
            } else {
                kept.add(m);
            }
        }
        if (kept.isEmpty()) {
            return;
        }
        Correction c =
                new Correction(
                        clampTime(frame.get(0).timestampSeconds()), kept.toArray(new Measured[0]));
        if (rejectsSinceGoodFrame >= RELOCALIZE_MIN_REJECTS
                && c.time - lastGoodFrameTime >= RELOCALIZE_SECONDS) {
            c.inflate = true;
            relocalizations++;
            rejectsSinceGoodFrame = 0;
            lastGoodFrameTime = c.time;
        }
        if (!insert(c, null)) {
            for (Measured m : c.tags) {
                reject(m.camera(), size == 0 ? "No Odometry" : "Too Old");
            }
        }
    }

    private static Measured measure(TagSighting t) {
        Translation3d inCamera = t.cameraToTag();
        double norm = inCamera.getNorm();
        if (!(norm > 0)) {
            return null;
        }
        Translation3d inRobot = inCamera.rotateBy(t.robotToCamera().getRotation());
        double range = Math.hypot(inRobot.getX(), inRobot.getY());
        if (range < MIN_RANGE_M || range > MAX_RANGE_M) {
            return null;
        }
        double offAxis = Math.acos(Math.clamp(inCamera.getX() / norm, -1.0, 1.0));
        return new Measured(
                t.camera(),
                Math.atan2(inRobot.getY(), inRobot.getX()),
                range,
                t.robotToCamera().getX(),
                t.robotToCamera().getY(),
                t.tagField().getX(),
                t.tagField().getY(),
                t.noiseScale() * (1 + sq(offAxis / OFF_AXIS_REF_RAD)));
    }

    /** Inserts a correction in time order and replays from it; false if outside the history. */
    private boolean insert(Correction c, String rejectAs) {
        if (size == 0 || c.time < sampleTime[oldest]) {
            if (rejectAs != null) {
                countRejection(rejectAs + (size == 0 ? " (No Odometry)" : " (Too Old)"));
            }
            return false;
        }
        int at = corrections.size();
        while (at > 0 && corrections.get(at - 1).time > c.time) {
            at--;
        }
        corrections.add(at, c);
        int k = size - 1;
        while (k > 0 && sampleTime[index(k)] > c.time) {
            k--;
        }
        recording = c;
        replayFrom(k);
        recording = null;
        return true;
    }

    /** Restores sample {@code k}'s pre-correction state and replays every later event. */
    private void replayFrom(int k) {
        int ik = index(k);
        System.arraycopy(preState, ik * 3, s, 0, 3);
        System.arraycopy(preCov, ik * 9, p, 0, 9);
        int ci = 0;
        while (ci < corrections.size() && corrections.get(ci).time < sampleTime[ik]) {
            ci++;
        }
        for (int j = k; j < size; j++) {
            int ij = index(j);
            if (j > k) {
                predict(
                        s,
                        p,
                        sampleDx[ij],
                        sampleDy[ij],
                        sampleDtheta[ij],
                        sampleVarXy[ij],
                        sampleVarTheta[ij]);
                storePre(ij);
            }
            double next = j + 1 < size ? sampleTime[index(j + 1)] : Double.POSITIVE_INFINITY;
            while (ci < corrections.size() && corrections.get(ci).time < next) {
                apply(corrections.get(ci), ij);
                ci++;
            }
        }
    }

    private void apply(Correction c, int sample) {
        if (c.inflate) {
            p[0] += sq(RELOCALIZE_XY_STD_M);
            p[4] += sq(RELOCALIZE_XY_STD_M);
            p[8] += sq(RELOCALIZE_THETA_STD_RAD);
        }
        if (c.tags == null) {
            correctPose(c);
            return;
        }
        double dt = sampleDt[sample];
        double speed = dt > 0 ? Math.hypot(sampleDx[sample], sampleDy[sample]) / dt : 0;
        double omega = dt > 0 ? Math.abs(sampleDtheta[sample]) / dt : 0;
        boolean record = c == recording;
        int accepted = 0;
        for (Measured m : c.tags) {
            if (correctTag(m, speed, omega, record)) {
                accepted++;
            }
        }
        if (record) {
            if (2 * accepted >= c.tags.length) {
                rejectsSinceGoodFrame = 0;
                lastGoodFrameTime = c.time;
            } else {
                rejectsSinceGoodFrame += c.tags.length - accepted;
            }
        }
    }

    private void correctPose(Correction c) {
        double[] r = {c.varXy, 0, 0, 0, c.varXy, 0, 0, 0, c.varTheta};
        double[] sInv = inverse3(add3(p, r));
        double[] k = mul3(p, sInv);
        double v0 = c.px - s[0];
        double v1 = c.py - s[1];
        double v2 = MathUtil.angleModulus(c.ptheta - s[2]);
        s[0] += k[0] * v0 + k[1] * v1 + k[2] * v2;
        s[1] += k[3] * v0 + k[4] * v1 + k[5] * v2;
        s[2] = MathUtil.angleModulus(s[2] + k[6] * v0 + k[7] * v1 + k[8] * v2);
        // Joseph form: (I-K)P(I-K)ᵀ + KRKᵀ.
        double[] ik = {1 - k[0], -k[1], -k[2], -k[3], 1 - k[4], -k[5], -k[6], -k[7], 1 - k[8]};
        double[] a = mul3(mul3(ik, p), transpose3(ik));
        double[] b = mul3(mul3(k, r), transpose3(k));
        for (int i = 0; i < 9; i++) {
            p[i] = a[i] + b[i];
        }
    }

    /** One tag's bearing-and-distance update; false if it failed the gate. */
    private boolean correctTag(Measured m, double speed, double omega, boolean record) {
        double c = Math.cos(s[2]);
        double sn = Math.sin(s[2]);
        double wx = m.tagX() - s[0];
        double wy = m.tagY() - s[1];
        double a = c * wx + sn * wy;
        double b = -sn * wx + c * wy;
        double ux = a - m.cx();
        double uy = b - m.cy();
        double rho2 = ux * ux + uy * uy;
        double rho = Math.sqrt(rho2);
        if (rho < 1e-6) {
            return false;
        }
        // du/d(x, y, theta): columns (-c, sn), (-sn, -c), (b, -a).
        double[] dux = {-c, -sn, b};
        double[] duy = {sn, -c, -a};
        double[] hb = new double[3];
        double[] hr = new double[3];
        for (int i = 0; i < 3; i++) {
            hb[i] = (-uy * dux[i] + ux * duy[i]) / rho2;
            hr[i] = (ux * dux[i] + uy * duy[i]) / rho;
        }
        double nb = MathUtil.angleModulus(m.bearing() - Math.atan2(uy, ux));
        double nr = m.range() - rho;

        double stdB =
                Math.hypot(
                        BEARING_STD_RAD * m.noiseScale(),
                        (omega + speed / m.range()) * TIMING_STD_SECONDS);
        double stdR =
                Math.hypot(
                        (RANGE_STD_BASE_M + RANGE_STD_PER_M2 * m.range() * m.range())
                                * m.noiseScale(),
                        speed * TIMING_STD_SECONDS);
        double rb = stdB * stdB;
        double rr = stdR * stdR;

        double[] phb = mulVec3(p, hb);
        double[] phr = mulVec3(p, hr);
        double s00 = dot3(hb, phb) + rb;
        double s01 = dot3(hb, phr);
        double s11 = dot3(hr, phr) + rr;
        double det = s00 * s11 - s01 * s01;
        if (!(det > 0)) {
            return false;
        }
        double i00 = s11 / det;
        double i01 = -s01 / det;
        double i11 = s00 / det;
        double d2 = nb * (i00 * nb + i01 * nr) + nr * (i01 * nb + i11 * nr);
        if (d2 > GATE_CHI2) {
            if (record) {
                reject(m.camera(), "Innovation");
            }
            return false;
        }
        // K = P Hᵀ S⁻¹, 3x2.
        double[] kb = new double[3];
        double[] kr = new double[3];
        for (int i = 0; i < 3; i++) {
            kb[i] = phb[i] * i00 + phr[i] * i01;
            kr[i] = phb[i] * i01 + phr[i] * i11;
        }
        s[0] += kb[0] * nb + kr[0] * nr;
        s[1] += kb[1] * nb + kr[1] * nr;
        s[2] = MathUtil.angleModulus(s[2] + kb[2] * nb + kr[2] * nr);
        double[] ikh = new double[9];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                ikh[i * 3 + j] = (i == j ? 1 : 0) - kb[i] * hb[j] - kr[i] * hr[j];
            }
        }
        double[] joseph = mul3(mul3(ikh, p), transpose3(ikh));
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                p[i * 3 + j] = joseph[i * 3 + j] + kb[i] * rb * kb[j] + kr[i] * rr * kr[j];
            }
        }
        if (record) {
            acceptedCount++;
            CameraStats st = statsFor(m.camera());
            st.accepted++;
            st.add(Math.toDegrees(nb), nr);
        }
        return true;
    }

    private static void predict(
            double[] s,
            double[] p,
            double lx,
            double ly,
            double dtheta,
            double varXy,
            double varTheta) {
        double c = Math.cos(s[2]);
        double sn = Math.sin(s[2]);
        double gx = c * lx - sn * ly;
        double gy = sn * lx + c * ly;
        s[0] += gx;
        s[1] += gy;
        s[2] = MathUtil.angleModulus(s[2] + dtheta);
        // F = [[1, 0, -gy], [0, 1, gx], [0, 0, 1]]; P = F P Fᵀ + Q.
        double p02 = p[2] - gy * p[8];
        double p12 = p[5] + gx * p[8];
        double p00 = p[0] - 2 * gy * p[2] + gy * gy * p[8];
        double p11 = p[4] + 2 * gx * p[5] + gx * gx * p[8];
        double p01 = p[1] + gx * p[2] - gy * p[5] - gx * gy * p[8];
        p[0] = p00 + varXy;
        p[1] = p01;
        p[2] = p02;
        p[3] = p01;
        p[4] = p11 + varXy;
        p[5] = p12;
        p[6] = p02;
        p[7] = p12;
        p[8] += varTheta;
    }

    /**
     * Counts a tag the caller chose not to apply.
     *
     * @param camera the tag's camera
     * @param reason why, for {@code RejectionCounts}
     */
    public void skip(String camera, String reason) {
        reject(camera, reason);
    }

    private void reject(String camera, String reason) {
        rejectedCount++;
        statsFor(camera).rejected++;
        countRejection(reason);
    }

    private void countRejection(String reason) {
        rejectionCounts.merge(reason, 1L, Long::sum);
    }

    private CameraStats statsFor(String camera) {
        return cameraStats.computeIfAbsent(camera, n -> new CameraStats());
    }

    private double clampTime(double t) {
        return size == 0 ? t : Math.min(t, newestTime());
    }

    private double newestTime() {
        return size == 0 ? Double.NEGATIVE_INFINITY : sampleTime[index(size - 1)];
    }

    private int index(int logical) {
        return (oldest + logical) % CAPACITY;
    }

    private void pushSample(
            double t,
            double dx,
            double dy,
            double dtheta,
            double dt,
            double varXy,
            double varTheta) {
        int i = index(size);
        size++;
        sampleTime[i] = t;
        sampleDx[i] = dx;
        sampleDy[i] = dy;
        sampleDtheta[i] = dtheta;
        sampleDt[i] = dt;
        sampleVarXy[i] = varXy;
        sampleVarTheta[i] = varTheta;
        storePre(i);
    }

    private void storePre(int i) {
        System.arraycopy(s, 0, preState, i * 3, 3);
        System.arraycopy(p, 0, preCov, i * 9, 9);
    }

    /** Drops the oldest sample; corrections in its interval are already in later states. */
    private void dropOldest() {
        oldest = (oldest + 1) % CAPACITY;
        size--;
        double first = size > 0 ? sampleTime[oldest] : Double.POSITIVE_INFINITY;
        while (!corrections.isEmpty() && corrections.get(0).time < first) {
            corrections.remove(0);
        }
    }

    /** Clears this loop's residuals; call after logging. */
    public void clearLoopStats() {
        for (CameraStats st : cameraStats.values()) {
            st.loopCount = 0;
        }
    }

    public Pose2d getPose() {
        return new Pose2d(s[0], s[1], Rotation2d.fromRadians(s[2]));
    }

    /** Std-devs [x m, y m, heading rad]. */
    public double[] getStdDevs() {
        return new double[] {Math.sqrt(p[0]), Math.sqrt(p[4]), Math.sqrt(p[8])};
    }

    public boolean isSeeded() {
        return seeded;
    }

    public long getAcceptedCount() {
        return acceptedCount;
    }

    public long getRejectedCount() {
        return rejectedCount;
    }

    public long getRelocalizations() {
        return relocalizations;
    }

    public Map<String, Long> getRejectionCounts() {
        return rejectionCounts;
    }

    public Map<String, CameraStats> getCameraStats() {
        return cameraStats;
    }

    // -- 3x3 helpers, row-major --

    private static double sq(double v) {
        return v * v;
    }

    private static double dot3(double[] a, double[] b) {
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    }

    private static double[] mulVec3(double[] m, double[] v) {
        return new double[] {
            m[0] * v[0] + m[1] * v[1] + m[2] * v[2],
            m[3] * v[0] + m[4] * v[1] + m[5] * v[2],
            m[6] * v[0] + m[7] * v[1] + m[8] * v[2]
        };
    }

    private static double[] mul3(double[] a, double[] b) {
        double[] out = new double[9];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                out[i * 3 + j] =
                        a[i * 3] * b[j] + a[i * 3 + 1] * b[3 + j] + a[i * 3 + 2] * b[6 + j];
            }
        }
        return out;
    }

    private static double[] add3(double[] a, double[] b) {
        double[] out = new double[9];
        for (int i = 0; i < 9; i++) {
            out[i] = a[i] + b[i];
        }
        return out;
    }

    private static double[] transpose3(double[] a) {
        return new double[] {a[0], a[3], a[6], a[1], a[4], a[7], a[2], a[5], a[8]};
    }

    private static double[] inverse3(double[] m) {
        double c00 = m[4] * m[8] - m[5] * m[7];
        double c01 = m[5] * m[6] - m[3] * m[8];
        double c02 = m[3] * m[7] - m[4] * m[6];
        double det = m[0] * c00 + m[1] * c01 + m[2] * c02;
        return new double[] {
            c00 / det,
            (m[2] * m[7] - m[1] * m[8]) / det,
            (m[1] * m[5] - m[2] * m[4]) / det,
            c01 / det,
            (m[0] * m[8] - m[2] * m[6]) / det,
            (m[2] * m[3] - m[0] * m[5]) / det,
            c02 / det,
            (m[1] * m[6] - m[0] * m[7]) / det,
            (m[0] * m[4] - m[1] * m[3]) / det
        };
    }
}
