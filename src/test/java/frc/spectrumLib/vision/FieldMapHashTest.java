package frc.spectrumLib.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;

import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.vision.apriltag.AprilTag;
import org.wpilib.vision.apriltag.AprilTagFieldLayout;
import org.wpilib.vision.apriltag.AprilTagFields;

class FieldMapHashTest {

    // The SpectrumJetson side checks itself against these two.
    @Test
    void andymarkHash() {
        assertEquals(
                "2792cd4e868967b8",
                FieldMapHash.of(
                        AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark)));
    }

    @Test
    void weldedHash() {
        assertEquals(
                "ca28a7c1139ed099",
                FieldMapHash.of(AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded)));
    }

    @Test
    void canonicalTextIsSortedFourDecimalsNoTrailingNewline() {
        AprilTagFieldLayout layout =
                new AprilTagFieldLayout(
                        List.of(
                                new AprilTag(7, new Pose3d(1.0, 2.0, 0.5, Rotation3d.ZERO)),
                                new AprilTag(
                                        2,
                                        new Pose3d(
                                                11.863959,
                                                -0.00001,
                                                1.12395,
                                                new Rotation3d(0, 0, Math.PI)))),
                        16.518,
                        8.043);
        assertEquals(
                "2,11.8640,-0.0000,1.1240,0.0000,0.0000,0.0000,1.0000\n"
                        + "7,1.0000,2.0000,0.5000,1.0000,0.0000,0.0000,0.0000\n"
                        + "field,16.5180,8.0430",
                FieldMapHash.canonicalText(layout));
    }

    @Test
    void roundsTheShortestDecimalHalfUp() {
        // 0.55245 is just below the tie in binary; Python's "%.4f" gives 0.5524. The 2026 fields'
        // hub tags are at this height, so another implementation must round like Java.
        AprilTagFieldLayout layout =
                new AprilTagFieldLayout(
                        List.of(new AprilTag(1, new Pose3d(0, 0, 0.55245, Rotation3d.ZERO))),
                        1,
                        1);
        assertEquals(
                "1,0.0000,0.0000,0.5525,1.0000,0.0000,0.0000,0.0000\nfield,1.0000,1.0000",
                FieldMapHash.canonicalText(layout));
    }

    @Test
    void hashIsSha256PrefixOfTheText() {
        // SHA-256("abc") = ba7816bf8f01cfea414140de5dae2223...
        assertEquals("ba7816bf8f01cfea", FieldMapHash.hash("abc"));
    }

    @Test
    void movingOneTagAMillimeterChangesIt() {
        AprilTagFieldLayout full =
                AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
        List<AprilTag> tags = new ArrayList<>(full.getTags());
        AprilTag first = tags.get(0);
        tags.set(
                0,
                new AprilTag(
                        first.ID,
                        new Pose3d(
                                first.pose.getX() + 0.001,
                                first.pose.getY(),
                                first.pose.getZ(),
                                first.pose.getRotation())));
        assertNotEquals(
                FieldMapHash.of(full),
                FieldMapHash.of(
                        new AprilTagFieldLayout(
                                tags, full.getFieldLength(), full.getFieldWidth())));
    }

    @Test
    void tagOrderDoesNotMatter() {
        AprilTagFieldLayout full = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
        List<AprilTag> reversed = new ArrayList<>(full.getTags()).reversed();
        assertEquals(
                FieldMapHash.of(full),
                FieldMapHash.of(
                        new AprilTagFieldLayout(
                                reversed, full.getFieldLength(), full.getFieldWidth())));
    }
}
