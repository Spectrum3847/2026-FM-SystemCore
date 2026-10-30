package frc.spectrumLib.vision;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import org.wpilib.fields.Field;
import org.wpilib.fields.FieldTag;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Quaternion;

/**
 * A layout as WPILib Field JSON, for a coprocessor to load.
 *
 * <p>Written here rather than by WPILib's serializer, which re-derives each quaternion: it writes
 * the 2026 fields' {@code -0.0} components as {@code 0.0}. {@link FieldMapHash} formats those as
 * {@code -0.0000} and {@code 0.0000}, so a coprocessor that hashed WPILib's output would never
 * match the robot. This writes the numbers the robot hashes.
 */
public final class FieldMapJson {
    private FieldMapJson() {}

    private static final ObjectMapper MAPPER = new ObjectMapper();

    /**
     * The layout as JSON: {@code {"tags":[{"ID","pose":{"translation":{x,y,z},"rotation":
     * {"quaternion":{W,X,Y,Z}}}}],"field":{"length","width"}}}, tags sorted by ID.
     *
     * @param layout the field map
     * @return the JSON text
     */
    public static String of(Field layout) {
        ObjectNode root = MAPPER.createObjectNode();
        ArrayNode tagsNode = root.putArray("tags");
        List<FieldTag> tags = new ArrayList<>(layout.getTags());
        tags.sort(Comparator.comparingInt(t -> t.getID()));
        for (FieldTag t : tags) {
            Pose3d p = t.getPose();
            Quaternion q = p.getRotation().getQuaternion();
            ObjectNode tag = tagsNode.addObject();
            tag.put("ID", t.getID());
            ObjectNode pose = tag.putObject("pose");
            ObjectNode translation = pose.putObject("translation");
            translation.put("x", p.getX());
            translation.put("y", p.getY());
            translation.put("z", p.getZ());
            ObjectNode quaternion = pose.putObject("rotation").putObject("quaternion");
            quaternion.put("W", q.getW());
            quaternion.put("X", q.getX());
            quaternion.put("Y", q.getY());
            quaternion.put("Z", q.getZ());
        }
        ObjectNode field = root.putObject("field");
        field.put("length", layout.getFieldLength());
        field.put("width", layout.getFieldWidth());
        return root.toString();
    }
}
