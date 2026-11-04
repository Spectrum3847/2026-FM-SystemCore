package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.spectrumLib.localization.SlipDetector;
import frc.spectrumLib.localization.TagEkf;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.SwerveModulePosition;

/** FieldPriors' hub and wall constraints, and SlipDetector. */
class FieldPriorsTest {

    private static TagEkf seededAt(double x, double y) {
        TagEkf e =
                new TagEkf(
                        new Translation2d[] {
                            new Translation2d(0.3, 0.3),
                            new Translation2d(0.3, -0.3),
                            new Translation2d(-0.3, 0.3),
                            new Translation2d(-0.3, -0.3)
                        });
        SwerveModulePosition[] still = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            still[i] = new SwerveModulePosition(0, new Rotation2d());
        }
        e.addOdometry(0, new Rotation2d(), still);
        e.addOdometry(0.01, new Rotation2d(), still);
        e.reset(new Pose2d(x, y, new Rotation2d()));
        return e;
    }

    @Test
    void pushesOutOfHub() {
        double[] hub = FieldPriors.HUBS[0];
        // Just inside the hub's near (low-x) face: out the near side.
        TagEkf e = seededAt(hub[0] + 0.05, (hub[2] + hub[3]) / 2);
        new FieldPriors(0, 0.01, 0.4).update(e, 0.01, 0);
        assertTrue(e.getPose().getX() < hub[0] - 0.3, "x " + e.getPose().getX());
    }

    @Test
    void legalPoseIsLeftAlone() {
        TagEkf e = seededAt(8.0, 2.0);
        FieldPriors f = new FieldPriors(0, 0.01, 0.4);
        f.update(e, 0.01, 0);
        assertEquals(0, f.constraints());
        assertEquals(8.0, e.getPose().getX(), 1e-9);
    }

    @Test
    void slipWhenWheelsAccelerateAndImuDoesnt() {
        SlipDetector d = new SlipDetector(3, 0.25);
        for (int i = 0; i <= 50; i++) {
            double t = i * 0.01;
            d.update(t, 0, 0, 0, 0);
        }
        assertTrue(!d.slipping(0.5));
        for (int i = 1; i <= 30; i++) {
            double t = 0.5 + i * 0.01;
            d.update(t, 8 * i * 0.01, 0, 0, 0); // wheels at 8 m/s², IMU still
        }
        assertTrue(d.slipping(0.8), "gap " + d.gap());
    }
}
