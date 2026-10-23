package frc.spectrumLib.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import org.junit.jupiter.api.Test;
import org.wpilib.vision.apriltag.AprilTagFieldLayout;
import org.wpilib.vision.apriltag.AprilTagFields;

class LimelightFieldMapTest {
    private static final double EPS = 1e-9;

    private static JsonNode andymark() throws Exception {
        return new ObjectMapper()
                .readTree(
                        LimelightFieldMap.of(
                                AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark),
                                165.1));
    }

    @Test
    void topLevelMatchesLimelightsOwn2026Map() throws Exception {
        JsonNode root = andymark();
        assertEquals("frc", root.get("type").asText());
        assertEquals(16.518, root.get("fieldlength").asDouble(), EPS);
        assertEquals(8.043, root.get("fieldwidth").asDouble(), EPS);
        assertEquals(32, root.get("fiducials").size());
        for (JsonNode f : root.get("fiducials")) {
            assertEquals("apriltag3_36h11_classic", f.get("family").asText());
            assertEquals(165.1, f.get("size").asDouble(), EPS);
            assertTrue(f.get("unique").asBoolean());
            assertEquals(16, f.get("transform").size());
        }
    }

    /** Tag 1 as Limelight's FRC2026_ANDYMARK.fmap has it: field-center origin, facing -x. */
    @Test
    void tagOneIsLimelightsTransform() throws Exception {
        JsonNode tag = andymark().get("fiducials").get(0);
        assertEquals(1, tag.get("id").asInt());
        double[] expected = {
            -1, 0, 0, 3.604959, 0, -1, 0, 3.3899914, 0, 0, 1, 0.889, 0, 0, 0, 1,
        };
        for (int i = 0; i < 16; i++) {
            assertEquals(expected[i], tag.get("transform").get(i).asDouble(), 1e-6, "element " + i);
        }
    }

    /** Tag 2 faces +y (yaw 90 degrees): the rotation block is [[0,-1],[1,0]]. */
    @Test
    void rotationIsRowMajor() throws Exception {
        JsonNode t = andymark().get("fiducials").get(1).get("transform");
        assertEquals(0, t.get(0).asDouble(), 1e-9);
        assertEquals(-1, t.get(1).asDouble(), 1e-9);
        assertEquals(1, t.get(4).asDouble(), 1e-9);
        assertEquals(0, t.get(5).asDouble(), 1e-9);
        assertEquals(3.6423986, t.get(3).asDouble(), 1e-6);
        assertEquals(0.6032558, t.get(7).asDouble(), 1e-6);
    }
}
