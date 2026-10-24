package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.spectrumLib.vision.FieldMapHash;
import frc.spectrumLib.vision.FieldMapJson;
import io.avaje.jsonb.Jsonb;
import org.junit.jupiter.api.Test;
import org.photonvision.proto.Photon.ProtobufDeviceMetrics;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.RawPublisher;
import org.wpilib.vision.apriltag.AprilTagFieldLayout;
import org.wpilib.vision.apriltag.AprilTagFields;

class JetsonNTIOTest {

    @Test
    void readsMetricsFromAnyHost() {
        JetsonNTIO io = new JetsonNTIO();
        // The Jetson's hostname at TXMAN1, which the old fixed topic name missed.
        RawPublisher pub =
                NetworkTableInstance.getDefault()
                        .getRawTopic("/photonvision//metrics/photonvision")
                        .publish("proto:ProtobufDeviceMetrics");
        pub.set(
                ProtobufDeviceMetrics.newInstance()
                        .setCpuUtil(37.5)
                        .setRamUtil(41.0)
                        .setUptime(1234.0)
                        .toByteArray());

        JetsonInputs inputs = new JetsonInputs();
        io.updateInputs(inputs);

        assertEquals(37.5, inputs.cpuUtilPct, 1e-9);
        assertEquals(41.0, inputs.ramUtilPct, 1e-9);
        assertEquals(1234.0, inputs.uptimeSeconds, 1e-9);
        pub.close();
    }

    @Test
    void readsTheJetsonsFieldMapAndPublishesTheRobots() {
        JetsonNTIO io = new JetsonNTIO();
        NetworkTableInstance nt = NetworkTableInstance.getDefault();
        var hash = nt.getStringTopic("/photonvision/jetson/FieldMap/Hash").publish();
        var name = nt.getStringTopic("/photonvision/jetson/FieldMap/Name").publish();
        // The Jetson's clock is in seconds; it may publish it as an integer.
        var loadedAt = nt.getIntegerTopic("/photonvision/jetson/FieldMap/LoadedAt").publish();
        hash.set("2792cd4e868967b8");
        name.set("k2026RebuiltAndymark");
        loadedAt.set(1_790_000_000L);
        var error = nt.getStringTopic("/photonvision/jetson/FieldMap/Error").publish();
        error.set("none yet");

        JetsonInputs inputs = new JetsonInputs();
        io.updateInputs(inputs);
        assertEquals("2792cd4e868967b8", inputs.fieldMapHash);
        assertEquals("k2026RebuiltAndymark", inputs.fieldMapName);
        assertEquals(1_790_000_000.0, inputs.fieldMapLoadedAt, 1e-9);
        assertEquals("none yet", inputs.fieldMapError);

        AprilTagFieldLayout layout =
                AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
        io.publishFieldMap(
                "k2026RebuiltAndymark", FieldMapHash.of(layout), FieldMapJson.of(layout));
        String json = nt.getStringTopic("/SpectrumVision/FieldMap/Json").subscribe("").get();
        assertEquals(
                "2792cd4e868967b8",
                nt.getStringTopic("/SpectrumVision/FieldMap/Hash").subscribe("").get());
        assertEquals(
                "k2026RebuiltAndymark",
                nt.getStringTopic("/SpectrumVision/FieldMap/Name").subscribe("").get());
        // The published JSON is the same map: it reads back to the same layout and hash.
        AprilTagFieldLayout back = Jsonb.instance().type(AprilTagFieldLayout.class).fromJson(json);
        assertEquals(layout, back);
        assertEquals("2792cd4e868967b8", FieldMapHash.of(back));
        hash.close();
        name.close();
        loadedAt.close();
        error.close();
    }
}
