package frc.spectrumLib.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.spectrumLib.localization.PoseObservation.Kind;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveModulePosition;

class PoseFusionTest {

    private static SwerveModulePosition[] still() {
        SwerveModulePosition[] p = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            p[i] = new SwerveModulePosition();
        }
        return p;
    }

    /** Builds the estimators and registers {@code sources} before any odometry, as Vision does. */
    private static PoseFusion fusion(PoseSource... sources) {
        var k =
                new SwerveDriveKinematics(
                        new Translation2d(0.3, 0.3),
                        new Translation2d(0.3, -0.3),
                        new Translation2d(-0.3, 0.3),
                        new Translation2d(-0.3, -0.3));
        PoseFusion f = new PoseFusion(k, Rotation2d.kZero, still(), new double[] {0.1, 0.1, 0.1});
        for (PoseSource s : sources) {
            f.register(s);
        }
        for (int i = 0; i < 20; i++) {
            f.addOdometry(i * 0.02, Rotation2d.kZero, still());
        }
        return f;
    }

    /** A source whose device reports one observation 1 m from where the robot thinks it is. */
    private static PoseSource source(String name, boolean enabled) {
        PoseSourceIO io =
                inputs -> {
                    inputs.connected = true;
                    inputs.resize(1);
                    inputs.set(
                            0,
                            new PoseObservation(
                                    name,
                                    Kind.PHOTON,
                                    0.3,
                                    new Pose3d(new Pose2d(1, 0, Rotation2d.kZero)),
                                    3,
                                    2.0,
                                    Double.NaN,
                                    0.1,
                                    true));
                };
        return new PoseSource(
                name,
                io,
                List.of(Gate.rejectIf("Never", (o, c) -> false)),
                (o, c) -> new StdDevModel.StdDevs(0.1, 1e6, "Test tier"),
                enabled);
    }

    private static GateContext ctx(PoseFusion f) {
        return new GateContext(
                0.38, f.getPose(), f.getOdometryPose(), new ChassisVelocities(), 0, false, true);
    }

    @Test
    void disabledSourceMovesOnlyItsShadow() {
        PoseSource s = source("Test-Disabled", false);
        PoseFusion f = fusion(s);
        s.update();
        f.apply(s, s.process(ctx(f)), s.isEnabled());

        assertEquals(0.0, f.getPose().getX(), 1e-9, "fused pose must not move");
        assertTrue(f.getShadowPose("Test-Disabled").orElseThrow().getX() > 0.4, "shadow must move");
        assertEquals(0.0, f.getOdometryPose().getX(), 1e-9);
    }

    @Test
    void enabledSourceMovesFusedAndShadow() {
        PoseSource s = source("Test-Enabled", true);
        PoseFusion f = fusion(s);
        s.update();
        f.apply(s, s.process(ctx(f)), s.isEnabled());

        assertTrue(f.getPose().getX() > 0.4, "fused x = " + f.getPose().getX());
        assertTrue(f.getShadowPose("Test-Enabled").orElseThrow().getX() > 0.4);
    }

    @Test
    void rejectionIsNamed() {
        PoseFusion f = fusion();
        PoseSource s =
                new PoseSource(
                        "Test-Gated",
                        inputs -> {
                            inputs.resize(1);
                            inputs.set(
                                    0,
                                    new PoseObservation(
                                            "",
                                            Kind.LIMELIGHT_MT1,
                                            0.3,
                                            Pose3d.kZero,
                                            1,
                                            3.0,
                                            0.5,
                                            0.95,
                                            true));
                        },
                        List.of(
                                Gate.rejectIf(
                                        "High Ambiguity Rejection",
                                        (o, c) -> o.maxAmbiguity() > 0.9)),
                        (o, c) -> new StdDevModel.StdDevs(0.1, 1e6, "x"),
                        true);
        s.update();
        var results = s.process(ctx(f));
        assertEquals("High Ambiguity Rejection", results.get(0).rejection());
        assertEquals(1, s.getRejectedCount());
    }

    /**
     * A source reporting one frame at {@code t} with heading {@code headingDeg}, heading ignored.
     */
    private static PoseSource frameAt(String name, double t, double x, double headingDeg) {
        PoseSourceIO io =
                inputs -> {
                    inputs.connected = true;
                    inputs.resize(1);
                    inputs.set(
                            0,
                            new PoseObservation(
                                    name,
                                    Kind.LIMELIGHT_MT1,
                                    t,
                                    new Pose3d(
                                            new Pose2d(x, 0, Rotation2d.fromDegrees(headingDeg))),
                                    2,
                                    2.0,
                                    Double.NaN,
                                    0.1,
                                    true));
                };
        return new PoseSource(
                name,
                io,
                List.of(Gate.rejectIf("Never", (o, c) -> false)),
                (o, c) -> new StdDevModel.StdDevs(0.1, 1e6, "Test tier"),
                true);
    }

    @Test
    void seedHeadingSurvivesOutOfOrderFrame() {
        // Seed at t from a frame, fuse that same frame at t (heading ignored), then an older frame
        // from another camera at t - 0.05 arrives: the redo must keep the seeded heading, in the
        // fused pose and in the shadows, which get the seed too.
        PoseSource cam = frameAt("Test-Cam", 0.30, 1.0, 40);
        PoseSource older = frameAt("Test-Older", 0.25, 1.0, 0);
        PoseFusion f = fusion(cam, older);
        cam.update();
        older.update();
        // Create the older camera's shadow first, so the seed reaches it as it does on the robot.
        f.apply(older, older.process(ctx(f)), false);

        f.seed(new Pose2d(1.0, 0, Rotation2d.fromDegrees(40)), 0.30, 0.01, 1e-4);
        f.apply(cam, cam.process(ctx(f)), true);
        assertEquals(40, f.getPose().getRotation().getDegrees(), 0.1);

        f.apply(older, older.process(ctx(f)), true);
        assertEquals(40, f.getPose().getRotation().getDegrees(), 0.1, "fused heading");
        assertEquals(
                40,
                f.getShadowPose("Test-Older").orElseThrow().getRotation().getDegrees(),
                0.1,
                "shadow heading");
    }
}
