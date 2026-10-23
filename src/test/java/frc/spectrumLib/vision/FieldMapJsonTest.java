package frc.spectrumLib.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import io.avaje.jsonb.Jsonb;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;
import org.wpilib.vision.apriltag.AprilTagFieldLayout;
import org.wpilib.vision.apriltag.AprilTagFields;

class FieldMapJsonTest {

    @ParameterizedTest
    @EnumSource(
            value = AprilTagFields.class,
            names = {"k2026RebuiltAndymark", "k2026RebuiltWelded"})
    void wpilibReadsItBackAsTheSameMap(AprilTagFields field) {
        AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(field);
        AprilTagFieldLayout back =
                Jsonb.instance().type(AprilTagFieldLayout.class).fromJson(FieldMapJson.of(layout));
        assertEquals(layout, back);
        assertEquals(FieldMapHash.of(layout), FieldMapHash.of(back));
    }

    /** What a coprocessor does: hash the numbers in the JSON it was sent. */
    @ParameterizedTest
    @EnumSource(
            value = AprilTagFields.class,
            names = {"k2026RebuiltAndymark", "k2026RebuiltWelded"})
    void hashingTheJsonsNumbersGivesTheRobotsHash(AprilTagFields field) throws Exception {
        AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(field);
        JsonNode root = new ObjectMapper().readTree(FieldMapJson.of(layout));
        List<String> lines = new ArrayList<>();
        for (JsonNode tag : root.get("tags")) {
            JsonNode t = tag.get("pose").get("translation");
            JsonNode q = tag.get("pose").get("rotation").get("quaternion");
            lines.add(
                    String.join(
                            ",",
                            tag.get("ID").asText(),
                            num(t, "x"),
                            num(t, "y"),
                            num(t, "z"),
                            num(q, "W"),
                            num(q, "X"),
                            num(q, "Y"),
                            num(q, "Z")));
        }
        JsonNode f = root.get("field");
        lines.add("field," + num(f, "length") + "," + num(f, "width"));
        assertEquals(FieldMapHash.of(layout), FieldMapHash.hash(String.join("\n", lines)));
    }

    private static String num(JsonNode node, String key) {
        return String.format(Locale.ROOT, "%.4f", node.get(key).asDouble());
    }
}
