package frc.spectrumLib.vision;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.numbers.N3;
import org.wpilib.vision.apriltag.AprilTag;
import org.wpilib.vision.apriltag.AprilTagFieldLayout;

/**
 * A WPILib layout as a Limelight field map (.fmap), the body of Limelight's {@code POST
 * /upload-fieldmap}.
 *
 * <p>Format from Limelight's map specification and its own FRC2026_ANDYMARK/WELDED.fmap downloads:
 *
 * <ul>
 *   <li>Top level: {@code type} ("frc"), {@code fieldlength} and {@code fieldwidth} (meters), and
 *       {@code fiducials}.
 *   <li>Each fiducial: {@code family}, {@code id}, {@code size} (the tag's black square, mm),
 *       {@code transform} and {@code unique}.
 *   <li>{@code transform} is the tag's pose as a row-major 4x4 matrix in meters, with the origin at
 *       the field's center. WPILib's origin is the blue corner, so x and y shift by half the field.
 *       The rotation is WPILib's tag rotation unchanged: Limelight's file for tag 1 of the AndyMark
 *       field is WPILib's pose shifted by (-8.259, -4.0215).
 * </ul>
 */
public final class LimelightFieldMap {
    private LimelightFieldMap() {}

    /** The FRC tag family since 2024, as Limelight names it. */
    public static final String FAMILY_36H11 = "apriltag3_36h11_classic";

    private static final ObjectMapper MAPPER = new ObjectMapper();

    /**
     * The .fmap JSON for a layout.
     *
     * @param layout the field map
     * @param tagSizeMm the tags' size in millimeters
     * @return the JSON text
     */
    public static String of(AprilTagFieldLayout layout, double tagSizeMm) {
        double halfLength = layout.getFieldLength() / 2;
        double halfWidth = layout.getFieldWidth() / 2;
        ObjectNode root = MAPPER.createObjectNode();
        root.put("type", "frc");
        root.put("fieldlength", layout.getFieldLength());
        root.put("fieldwidth", layout.getFieldWidth());
        ArrayNode fiducials = root.putArray("fiducials");
        List<AprilTag> tags = new ArrayList<>(layout.getTags());
        tags.sort(Comparator.comparingInt(t -> t.ID));
        for (AprilTag t : tags) {
            Pose3d p = t.pose;
            Matrix<N3, N3> r = p.getRotation().toMatrix();
            double[] translation = {p.getX() - halfLength, p.getY() - halfWidth, p.getZ()};
            ObjectNode f = fiducials.addObject();
            f.put("family", FAMILY_36H11);
            f.put("id", t.ID);
            f.put("size", tagSizeMm);
            ArrayNode transform = f.putArray("transform");
            for (int row = 0; row < 3; row++) {
                for (int col = 0; col < 3; col++) {
                    transform.add(r.get(row, col));
                }
                transform.add(translation[row]);
            }
            transform.add(0.0).add(0.0).add(0.0).add(1.0);
            f.put("unique", true);
        }
        return root.toString();
    }
}
