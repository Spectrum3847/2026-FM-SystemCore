package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;

import java.util.ArrayList;
import org.junit.jupiter.api.Test;
import org.wpilib.vision.apriltag.AprilTagFieldLayout;
import org.wpilib.vision.apriltag.AprilTagFields;

class PreMatchFieldTest {

    private static final AprilTagFields ANDYMARK = AprilTagFields.k2026RebuiltAndymark;
    private static final AprilTagFields WELDED = AprilTagFields.k2026RebuiltWelded;

    @Test
    void theEventsLayoutPasses() {
        assertNull(
                PreMatch.fieldMismatch(
                        AprilTagFieldLayout.loadField(ANDYMARK), ANDYMARK, ANDYMARK));
    }

    @Test
    void theOtherFieldWarns() {
        assertEquals(
                "AprilTag layout k2026RebuiltAndymark, but this event's field is"
                        + " k2026RebuiltWelded (Event.APRILTAG_FIELD)",
                PreMatch.fieldMismatch(AprilTagFieldLayout.loadField(ANDYMARK), ANDYMARK, WELDED));
    }

    @Test
    void aModifiedLayoutWarns() {
        AprilTagFieldLayout full = AprilTagFieldLayout.loadField(ANDYMARK);
        var tags = new ArrayList<>(full.getTags());
        tags.remove(0); // a tag dropped, as a hand-edited layout might
        AprilTagFieldLayout edited =
                new AprilTagFieldLayout(tags, full.getFieldLength(), full.getFieldWidth());
        assertEquals(
                "AprilTag layout differs from WPILib's k2026RebuiltAndymark, this event's",
                PreMatch.fieldMismatch(edited, ANDYMARK, ANDYMARK));
    }
}
