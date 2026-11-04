package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;

import frc.spectrumLib.localization.TagEkf;
import frc.spectrumLib.localization.TagSighting;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;

/** Vision.addTagFrame: tags dropped only above the skip tilt, fed (noisier) below it. */
class TagEkfTiltPolicyTest {

    private static TagEkf ekf() {
        return new TagEkf(
                new Translation2d[] {
                    new Translation2d(0.3, 0.3),
                    new Translation2d(0.3, -0.3),
                    new Translation2d(-0.3, 0.3),
                    new Translation2d(-0.3, -0.3)
                });
    }

    private static final List<TagSighting> FRAME =
            List.of(
                    new TagSighting(
                            "cam",
                            0,
                            1,
                            new Translation3d(5, 0, 1),
                            new Transform3d(),
                            new Translation3d(3, 0, 0.5),
                            1.0));

    @Test
    void steadyTiltIsFedNotDropped() {
        TagEkf e = ekf();
        Vision.addTagFrame(e, FRAME, 7, Vision.EKF_SKIP_TILT_DEG);
        assertNull(e.getRejectionCounts().get("Tilted"));
    }

    @Test
    void steepTiltIsDropped() {
        TagEkf e = ekf();
        Vision.addTagFrame(e, FRAME, Vision.EKF_SKIP_TILT_DEG + 1, Vision.EKF_SKIP_TILT_DEG);
        assertEquals(1L, e.getRejectionCounts().get("Tilted"));
    }
}
