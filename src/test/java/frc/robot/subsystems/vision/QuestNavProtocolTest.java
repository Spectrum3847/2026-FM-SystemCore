package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.ByteArrayOutputStream;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;

class QuestNavProtocolTest {

    /** Builds a ProtobufQuestNavFrameData the way the headset would, by hand. */
    private static byte[] frame(int count, double timestamp, byte[] pose, boolean tracking) {
        ByteArrayOutputStream out = new ByteArrayOutputStream();
        out.write(0x08); // field 1 varint
        out.write(count);
        out.write(0x11); // field 2 fixed64
        long bits = Double.doubleToLongBits(timestamp);
        for (int i = 0; i < 8; i++) {
            out.write((int) (bits >>> (8 * i)) & 0xFF);
        }
        out.write(0x1A); // field 3 length-delimited
        out.write(pose.length);
        out.writeBytes(pose);
        out.write(0x20); // field 4 varint
        out.write(tracking ? 1 : 0);
        return out.toByteArray();
    }

    @Test
    void poseResetRoundTripsThroughTheFrameDecoder() {
        Pose3d pose = new Pose3d(3.25, -1.5, 0.42, new Rotation3d(0.01, -0.02, 1.2));
        // A pose-reset command carries the same ProtobufPose3d a frame does: cut it out.
        byte[] cmd = QuestNavProtocol.encodePoseReset(7, pose);
        int payloadStart = 4; // type(2 bytes) + command_id(2 bytes)
        assertEquals((byte) 0x52, cmd[payloadStart]); // field 10, length-delimited
        int payloadLen = cmd[payloadStart + 1];
        byte[] payload =
                java.util.Arrays.copyOfRange(cmd, payloadStart + 2, payloadStart + 2 + payloadLen);
        assertEquals((byte) 0x0A, payload[0]); // target_pose
        byte[] poseBytes = java.util.Arrays.copyOfRange(payload, 2, 2 + payload[1]);

        QuestNavProtocol.Frame f = QuestNavProtocol.decodeFrame(frame(42, 12.5, poseBytes, true));
        assertNotNull(f);
        assertEquals(42, f.frameCount());
        assertEquals(12.5, f.appTimestamp(), 1e-12);
        assertTrue(f.tracking());
        assertEquals(pose.getX(), f.pose().getX(), 1e-9);
        assertEquals(pose.getY(), f.pose().getY(), 1e-9);
        assertEquals(pose.getZ(), f.pose().getZ(), 1e-9);
        assertEquals(pose.getRotation().getZ(), f.pose().getRotation().getZ(), 1e-9);
    }

    @Test
    void malformedFrameIsRejectedNotThrown() {
        assertEquals(null, QuestNavProtocol.decodeFrame(new byte[] {0x1A, 0x7F}));
    }
}
