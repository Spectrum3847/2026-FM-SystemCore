package frc.robot.subsystems.vision;

import java.io.ByteArrayOutputStream;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Quaternion;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;

/**
 * The few QuestNav protobuf messages FM uses, encoded and decoded by hand.
 *
 * <p>QuestNavLib's vendordep is built for WPILib 2026 and does not load on 2027. Its wire format is
 * small and stable (QuestNav {@code protos/data.proto}, {@code commands.proto} and WPILib's {@code
 * geometry3d.proto}), so rather than generate quickbuf classes this reads and writes the protobuf
 * wire format directly:
 *
 * <pre>
 * ProtobufQuestNavFrameData  { int32 frame_count = 1; double timestamp = 2;
 *                              ProtobufPose3d pose3d = 3; bool isTracking = 4; }
 * ProtobufQuestNavDeviceData { int32 tracking_lost_counter = 1; int32 battery_percent = 3; }
 * ProtobufQuestNavCommand    { QuestNavCommandType type = 1; uint32 command_id = 2;
 *                              ProtobufQuestNavPoseResetPayload pose_reset_payload = 10; }
 * ProtobufQuestNavPoseResetPayload { ProtobufPose3d target_pose = 1; }
 * ProtobufPose3d { ProtobufTranslation3d translation = 1; ProtobufRotation3d rotation = 2; }
 * ProtobufTranslation3d { double x = 1; double y = 2; double z = 3; }
 * ProtobufRotation3d { ProtobufQuaternion q = 1; }
 * ProtobufQuaternion { double w = 1; double x = 2; double y = 3; double z = 4; }
 * </pre>
 *
 * <p>Check this against the QuestNav app version on the headset before the event.
 */
public final class QuestNavProtocol {
    private QuestNavProtocol() {}

    public static final String FRAME_DATA_TYPE =
            "proto:questnav.protos.data.ProtobufQuestNavFrameData";
    public static final String DEVICE_DATA_TYPE =
            "proto:questnav.protos.data.ProtobufQuestNavDeviceData";
    public static final String COMMAND_TYPE =
            "proto:questnav.protos.commands.ProtobufQuestNavCommand";

    /** {@code QuestNavCommandType.POSE_RESET}. */
    private static final int POSE_RESET = 1;

    /** A decoded frame. */
    public record Frame(int frameCount, double appTimestamp, Pose3d pose, boolean tracking) {}

    /** A decoded device status. */
    public record DeviceData(int trackingLostCounter, int batteryPercent) {}

    // ── Decoding ────────────────────────────────────────────────────────────

    private static final class Reader {
        private final ByteBuffer buf;

        Reader(byte[] bytes, int offset, int length) {
            buf = ByteBuffer.wrap(bytes, offset, length).order(ByteOrder.LITTLE_ENDIAN);
        }

        boolean more() {
            return buf.hasRemaining();
        }

        long varint() {
            long result = 0;
            int shift = 0;
            while (true) {
                byte b = buf.get();
                result |= (long) (b & 0x7F) << shift;
                if ((b & 0x80) == 0) {
                    return result;
                }
                shift += 7;
            }
        }

        double fixed64() {
            return buf.getDouble();
        }

        /** Returns the sub-message as {offset, length} into the backing array and skips it. */
        int[] lengthDelimited() {
            int len = (int) varint();
            int start = buf.arrayOffset() + buf.position();
            buf.position(buf.position() + len);
            return new int[] {start, len};
        }

        void skip(int wireType) {
            switch (wireType) {
                case 0 -> varint();
                case 1 -> buf.position(buf.position() + 8);
                case 2 -> lengthDelimited();
                case 5 -> buf.position(buf.position() + 4);
                default -> throw new IllegalArgumentException("wire type " + wireType);
            }
        }
    }

    /** Decodes {@code ProtobufQuestNavFrameData}; {@code null} if the bytes are malformed. */
    public static Frame decodeFrame(byte[] bytes) {
        try {
            Reader r = new Reader(bytes, 0, bytes.length);
            int frameCount = 0;
            double timestamp = 0;
            Pose3d pose = Pose3d.kZero;
            boolean tracking = false;
            while (r.more()) {
                long tag = r.varint();
                int field = (int) (tag >>> 3);
                int wire = (int) (tag & 7);
                if (field == 1 && wire == 0) {
                    frameCount = (int) r.varint();
                } else if (field == 2 && wire == 1) {
                    timestamp = r.fixed64();
                } else if (field == 3 && wire == 2) {
                    int[] sub = r.lengthDelimited();
                    pose = decodePose(bytes, sub[0], sub[1]);
                } else if (field == 4 && wire == 0) {
                    tracking = r.varint() != 0;
                } else {
                    r.skip(wire);
                }
            }
            return new Frame(frameCount, timestamp, pose, tracking);
        } catch (RuntimeException e) {
            return null;
        }
    }

    /** Decodes {@code ProtobufQuestNavDeviceData}; {@code null} if malformed. */
    public static DeviceData decodeDeviceData(byte[] bytes) {
        try {
            Reader r = new Reader(bytes, 0, bytes.length);
            int lost = 0;
            int battery = -1;
            while (r.more()) {
                long tag = r.varint();
                int field = (int) (tag >>> 3);
                int wire = (int) (tag & 7);
                if (field == 1 && wire == 0) {
                    lost = (int) r.varint();
                } else if (field == 3 && wire == 0) {
                    battery = (int) r.varint();
                } else {
                    r.skip(wire);
                }
            }
            return new DeviceData(lost, battery);
        } catch (RuntimeException e) {
            return null;
        }
    }

    private static Pose3d decodePose(byte[] bytes, int offset, int length) {
        Reader r = new Reader(bytes, offset, length);
        Translation3d t = Translation3d.kZero;
        Rotation3d rot = Rotation3d.kZero;
        while (r.more()) {
            long tag = r.varint();
            int field = (int) (tag >>> 3);
            int wire = (int) (tag & 7);
            if (field == 1 && wire == 2) {
                int[] sub = r.lengthDelimited();
                double[] xyz = decodeDoubles(bytes, sub[0], sub[1], 3);
                t = new Translation3d(xyz[0], xyz[1], xyz[2]);
            } else if (field == 2 && wire == 2) {
                int[] sub = r.lengthDelimited();
                Reader rr = new Reader(bytes, sub[0], sub[1]);
                while (rr.more()) {
                    long tag2 = rr.varint();
                    if ((tag2 >>> 3) == 1 && (tag2 & 7) == 2) {
                        int[] q = rr.lengthDelimited();
                        double[] wxyz = decodeDoubles(bytes, q[0], q[1], 4);
                        rot = new Rotation3d(new Quaternion(wxyz[0], wxyz[1], wxyz[2], wxyz[3]));
                    } else {
                        rr.skip((int) (tag2 & 7));
                    }
                }
            } else {
                r.skip(wire);
            }
        }
        return new Pose3d(t, rot);
    }

    /** Decodes a message of consecutive {@code double} fields 1..n (proto3 omits zeros). */
    private static double[] decodeDoubles(byte[] bytes, int offset, int length, int n) {
        double[] out = new double[n];
        if (n == 4) {
            out[0] = 0; // w defaults to 0 on the wire; a real quaternion always sends it
        }
        Reader r = new Reader(bytes, offset, length);
        while (r.more()) {
            long tag = r.varint();
            int field = (int) (tag >>> 3);
            int wire = (int) (tag & 7);
            if (wire == 1 && field >= 1 && field <= n) {
                out[field - 1] = r.fixed64();
            } else {
                r.skip(wire);
            }
        }
        return out;
    }

    // ── Encoding ────────────────────────────────────────────────────────────

    private static void varint(ByteArrayOutputStream out, long v) {
        while ((v & ~0x7FL) != 0) {
            out.write((int) ((v & 0x7F) | 0x80));
            v >>>= 7;
        }
        out.write((int) v);
    }

    private static void doubleField(ByteArrayOutputStream out, int field, double v) {
        varint(out, (field << 3) | 1);
        long bits = Double.doubleToLongBits(v);
        for (int i = 0; i < 8; i++) {
            out.write((int) (bits >>> (8 * i)) & 0xFF);
        }
    }

    private static void messageField(ByteArrayOutputStream out, int field, byte[] body) {
        varint(out, (field << 3) | 2);
        varint(out, body.length);
        out.writeBytes(body);
    }

    private static byte[] encodePose(Pose3d pose) {
        ByteArrayOutputStream t = new ByteArrayOutputStream();
        doubleField(t, 1, pose.getX());
        doubleField(t, 2, pose.getY());
        doubleField(t, 3, pose.getZ());
        Quaternion q = pose.getRotation().getQuaternion();
        ByteArrayOutputStream qb = new ByteArrayOutputStream();
        doubleField(qb, 1, q.getW());
        doubleField(qb, 2, q.getX());
        doubleField(qb, 3, q.getY());
        doubleField(qb, 4, q.getZ());
        ByteArrayOutputStream rot = new ByteArrayOutputStream();
        messageField(rot, 1, qb.toByteArray());
        ByteArrayOutputStream p = new ByteArrayOutputStream();
        messageField(p, 1, t.toByteArray());
        messageField(p, 2, rot.toByteArray());
        return p.toByteArray();
    }

    /**
     * Encodes a pose-reset command: the Quest re-origins so that its current pose reads {@code
     * questPose} on the field.
     */
    public static byte[] encodePoseReset(int commandId, Pose3d questPose) {
        ByteArrayOutputStream payload = new ByteArrayOutputStream();
        messageField(payload, 1, encodePose(questPose));
        ByteArrayOutputStream cmd = new ByteArrayOutputStream();
        varint(cmd, (1 << 3));
        varint(cmd, POSE_RESET);
        varint(cmd, (2 << 3));
        varint(cmd, commandId);
        messageField(cmd, 10, payload.toByteArray());
        return cmd.toByteArray();
    }
}
