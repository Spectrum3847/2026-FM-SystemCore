package frc.spectrumLib.vision;

import java.nio.charset.StandardCharsets;
import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HexFormat;
import java.util.List;
import java.util.Locale;
import org.wpilib.fields.Field;
import org.wpilib.fields.FieldTag;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Quaternion;

/**
 * A short fingerprint of an AprilTag field map, so the robot and each coprocessor can tell whether
 * they solve with the same tags. Every side computes it the same way (the SpectrumJetson code
 * implements this exact text), so change nothing here without changing them too:
 *
 * <ol>
 *   <li>Tags sorted by ID, one line each: {@code id,x,y,z,qw,qx,qy,qz} (meters, and the quaternion
 *       of the tag's pose).
 *   <li>A last line {@code field,<length m>,<width m>}.
 *   <li>Every number with four decimals, {@code String.format(Locale.ROOT, "%.4f")}; lines joined
 *       with {@code "\n"}, no trailing newline. Java rounds the shortest decimal form half-up
 *       (0.55245 gives 0.5525), where C and Python's {@code "%.4f"} round the exact binary value
 *       (0.5524): another implementation must round like Java, e.g. Python's {@code
 *       Decimal(repr(x)).quantize(Decimal("0.0001"), ROUND_HALF_UP)}.
 *   <li>SHA-256 of the UTF-8 bytes, lowercase hex, the first {@link #LENGTH} characters.
 * </ol>
 *
 * <p>Four decimals is a tenth of a millimeter: well under what any solve can see, and coarse enough
 * that the same map read by two JSON parsers hashes the same.
 */
public final class FieldMapHash {
    private FieldMapHash() {}

    /** Hex characters kept. */
    public static final int LENGTH = 16;

    /**
     * The canonical hash of a layout.
     *
     * @param layout the field map
     * @return {@link #LENGTH} lowercase hex characters
     */
    public static String of(Field layout) {
        return hash(canonicalText(layout));
    }

    /**
     * The text that is hashed, for tests and for comparing with another implementation.
     *
     * @param layout the field map
     * @return the canonical lines, joined with {@code "\n"}
     */
    public static String canonicalText(Field layout) {
        List<FieldTag> tags = new ArrayList<>(layout.getTags());
        tags.sort(Comparator.comparingInt(t -> t.getID()));
        List<String> lines = new ArrayList<>(tags.size() + 1);
        for (FieldTag t : tags) {
            Pose3d p = t.getPose();
            Quaternion q = p.getRotation().getQuaternion();
            lines.add(
                    t.getID()
                            + ","
                            + num(p.getX())
                            + ","
                            + num(p.getY())
                            + ","
                            + num(p.getZ())
                            + ","
                            + num(q.getW())
                            + ","
                            + num(q.getX())
                            + ","
                            + num(q.getY())
                            + ","
                            + num(q.getZ()));
        }
        lines.add("field," + num(layout.getFieldLength()) + "," + num(layout.getFieldWidth()));
        return String.join("\n", lines);
    }

    static String hash(String text) {
        try {
            byte[] digest =
                    MessageDigest.getInstance("SHA-256")
                            .digest(text.getBytes(StandardCharsets.UTF_8));
            return HexFormat.of().formatHex(digest).substring(0, LENGTH);
        } catch (NoSuchAlgorithmException e) {
            throw new IllegalStateException("every JVM has SHA-256", e);
        }
    }

    private static String num(double v) {
        return String.format(Locale.ROOT, "%.4f", v);
    }
}
