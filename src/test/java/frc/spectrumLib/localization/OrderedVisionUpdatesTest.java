package frc.spectrumLib.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;

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
}
