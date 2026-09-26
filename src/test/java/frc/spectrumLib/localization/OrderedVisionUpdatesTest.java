package frc.spectrumLib.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.wpilib.math.estimator.SwerveDrivePoseEstimator;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.linalg.VecBuilder;

class OrderedVisionUpdatesTest {
    private static final org.wpilib.math.linalg.Matrix<
                    org.wpilib.math.numbers.N3, org.wpilib.math.numbers.N1>
            STD = VecBuilder.fill(0.5, 0.5, 0.5);

    /** A robot driving +x at 1 m/s, sampled at 250 Hz for one second. */
    private static SwerveDrivePoseEstimator driving() {
        var k =
                new SwerveDriveKinematics(
                        new Translation2d(0.3, 0.3),
                        new Translation2d(0.3, -0.3),
                        new Translation2d(-0.3, 0.3),
                        new Translation2d(-0.3, -0.3));
        SwerveModulePosition[] p = positions(0);
        var e =
                new SwerveDrivePoseEstimator(
                        k,
                        Rotation2d.kZero,
                        p,
                        Pose2d.kZero,
                        VecBuilder.fill(0.1, 0.1, 0.1),
                        VecBuilder.fill(0.9, 0.9, 0.9));
        for (int i = 0; i <= 250; i++) {
            e.updateWithTime(i * 0.004, Rotation2d.kZero, positions(i * 0.004));
        }
        return e;
    }

    private static SwerveModulePosition[] positions(double metres) {
        SwerveModulePosition[] p = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            p[i] = new SwerveModulePosition(metres, Rotation2d.kZero);
        }
        return p;
    }

    private static final Pose2d EARLY = new Pose2d(0.5, 0.2, Rotation2d.kZero); // t = 0.4
    private static final Pose2d LATE = new Pose2d(0.9, -0.1, Rotation2d.kZero); // t = 0.8

    @Test
    void outOfOrderMatchesInOrder() {
        var inOrder = driving();
        var inOrderV = new OrderedVisionUpdates(inOrder);
        inOrderV.add(EARLY, 0.4, STD, 1.0);
        inOrderV.add(LATE, 0.8, STD, 1.0);

        var reversed = driving();
        var reversedV = new OrderedVisionUpdates(reversed);
        reversedV.add(LATE, 0.8, STD, 1.0);
        reversedV.add(EARLY, 0.4, STD, 1.0);

        Pose2d a = inOrder.getEstimatedPosition();
        Pose2d b = reversed.getEstimatedPosition();
        assertEquals(a.getX(), b.getX(), 1e-9);
        assertEquals(a.getY(), b.getY(), 1e-9);
        assertEquals(1, reversedV.reapplied());
        assertEquals(0, inOrderV.reapplied());
    }

    @Test
    void plainEstimatorDropsTheNewerMeasurement() {
        // The behavior being worked around: adding the older frame second discards the newer one.
        var inOrder = driving();
        inOrder.addVisionMeasurement(EARLY, 0.4, STD);
        inOrder.addVisionMeasurement(LATE, 0.8, STD);
        var reversed = driving();
        reversed.addVisionMeasurement(LATE, 0.8, STD);
        reversed.addVisionMeasurement(EARLY, 0.4, STD);
        assertNotEquals(
                inOrder.getEstimatedPosition().getY(),
                reversed.getEstimatedPosition().getY(),
                1e-6);
    }

    @Test
    void forgetsMeasurementsOlderThanTheBuffer() {
        var e = driving();
        var v = new OrderedVisionUpdates(e);
        v.add(LATE, 0.8, STD, 1.0);
        // Newest odometry now 2.5 s: the 0.8 s frame is outside the 1.5 s buffer and not redone.
        v.add(EARLY, 0.4, STD, 2.5);
        assertEquals(0, v.reapplied());
    }

    private static final double TINY = 1e-4;
    private static final double HUGE = 999999.0;

    @Test
    void seedSurvivesAnOutOfOrderRedo() {
        // Vision seeds with a MegaTag1 frame, and the same frame at the same time is fused as an
        // ordinary camera measurement whose heading is ignored. An older frame arriving afterwards
        // forces a redo, which must re-apply the seed too, not just the camera frame.
        Pose2d seeded = new Pose2d(0.8, 0.0, Rotation2d.fromDegrees(30));
        var e = driving();
        var v = new OrderedVisionUpdates(e);
        v.add(seeded, 0.8, VecBuilder.fill(TINY, TINY, TINY), 1.0);
        v.add(seeded, 0.8, VecBuilder.fill(0.5, 0.5, HUGE), 1.0);
        double before = e.getEstimatedPosition().getRotation().getDegrees();
        assertEquals(30, before, 0.1);

        v.add(new Pose2d(0.75, 0.0, Rotation2d.kZero), 0.75, VecBuilder.fill(0.5, 0.5, HUGE), 1.0);
        assertEquals(2, v.reapplied());
        assertEquals(before, e.getEstimatedPosition().getRotation().getDegrees(), 0.01);
    }

    @Test
    void sameTimestampRedoMatchesInOrder() {
        // Two measurements sharing a timestamp are re-applied in the order they were added.
        Pose2d a = new Pose2d(0.9, 0.2, Rotation2d.fromDegrees(10));
        Pose2d b = new Pose2d(0.7, -0.2, Rotation2d.fromDegrees(-5));
        var inOrder = driving();
        var inOrderV = new OrderedVisionUpdates(inOrder);
        inOrderV.add(EARLY, 0.4, STD, 1.0);
        inOrderV.add(a, 0.8, STD, 1.0);
        inOrderV.add(b, 0.8, STD, 1.0);

        var late = driving();
        var lateV = new OrderedVisionUpdates(late);
        lateV.add(a, 0.8, STD, 1.0);
        lateV.add(b, 0.8, STD, 1.0);
        lateV.add(EARLY, 0.4, STD, 1.0);

        Pose2d x = inOrder.getEstimatedPosition();
        Pose2d y = late.getEstimatedPosition();
        assertEquals(x.getX(), y.getX(), 1e-9);
        assertEquals(x.getY(), y.getY(), 1e-9);
        assertEquals(x.getRotation().getRadians(), y.getRotation().getRadians(), 1e-9);
    }

    @Test
    void boundedWhenOdometryStops() {
        var e = driving();
        var v = new OrderedVisionUpdates(e);
        // Odometry stuck at 1.0 s while frames keep arriving: nothing ages out by time.
        for (int i = 0; i < OrderedVisionUpdates.MAX_TIMESTAMPS + 500; i++) {
            v.add(LATE, 1.0 + i * 0.01, STD, 1.0);
        }
        assertTrue(v.size() <= OrderedVisionUpdates.MAX_TIMESTAMPS);
    }
}
