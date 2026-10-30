package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;

import frc.spectrumLib.localization.Gate;
import frc.spectrumLib.localization.GateContext;
import frc.spectrumLib.localization.PoseObservation;
import frc.spectrumLib.localization.PoseObservation.Kind;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;

class VisionGatesTest {
    private static final double NOW = 100.0;

    private static final GateContext CONTEXT =
            new GateContext(
                    NOW, Pose2d.ZERO, Pose2d.ZERO, new ChassisVelocities(), 0, false, true);

    /** A two-tag MegaTag1 frame mid-field that every other gate passes. */
    private static PoseObservation frameAt(double timestampSeconds) {
        return new PoseObservation(
                "Test",
                Kind.LIMELIGHT_MT1,
                timestampSeconds,
                new Pose3d(new Pose2d(5, 4, Rotation2d.ZERO)),
                2,
                2.0,
                1.0,
                0.1,
                true);
    }

    /** The first rejection in {@code gates}, or null if the frame passes them all. */
    private static String firstRejection(List<Gate> gates, PoseObservation o) {
        for (Gate g : gates) {
            String reason = g.check(o, CONTEXT);
            if (reason != null) {
                return reason;
            }
        }
        return null;
    }

    @Test
    void futureFrameIsRejected() {
        assertEquals(
                "Future Timestamp Rejection",
                firstRejection(VisionGates.aprilTagGates(), frameAt(NOW + 333)));
        assertEquals(
                "Future Timestamp Rejection",
                firstRejection(VisionGates.aprilTagGates(), frameAt(NOW + 0.06)));
    }

    @Test
    void frameWithinTheMarginPasses() {
        assertNull(firstRejection(VisionGates.aprilTagGates(), frameAt(NOW + 0.04)));
        assertNull(firstRejection(VisionGates.aprilTagGates(), frameAt(NOW - 0.1)));
    }
}
