package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.spectrumLib.vision.FieldMapHash;
import frc.spectrumLib.vision.FieldMapJson;
import io.avaje.jsonb.Jsonb;
import org.junit.jupiter.api.Test;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.PubSubOption;
import org.wpilib.vision.apriltag.AprilTagFieldLayout;
import org.wpilib.vision.apriltag.AprilTagFields;

class JetsonNTIOTest {

    @Test
    void readsTheJetsonsDiagnostics() {
        JetsonNTIO io = new JetsonNTIO();
        NetworkTableInstance nt = NetworkTableInstance.getDefault();
        var cpu = nt.getDoubleTopic("/photonvision/jetson/cpuUtilPct").publish();
        var uptime = nt.getDoubleTopic("/photonvision/jetson/uptimeSeconds").publish();
        var bootId = nt.getStringTopic("/photonvision/jetson/bootId").publish();
        var usb = nt.getIntegerTopic("/photonvision/jetson/usbEvents").publish();
        var link = nt.getBooleanTopic("/photonvision/jetson/ethLinkUp").publish();
        cpu.set(37.5);
        uptime.set(1234.0);
        bootId.set("3f1c");
        usb.set(2);
        link.set(true);

        JetsonInputs inputs = new JetsonInputs();
        io.updateInputs(inputs);

        assertEquals(37.5, inputs.cpuUtilPct, 1e-9);
        assertEquals(1234.0, inputs.uptimeSeconds, 1e-9);
        assertEquals("3f1c", inputs.bootId);
        assertEquals(2, inputs.usbEvents);
        assertTrue(inputs.ethLinkUp);
        cpu.close();
        uptime.close();
        bootId.close();
        usb.close();
        link.close();
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

    @Test
    void readsEveryJetsonLogLineOnce() {
        JetsonNTIO io = new JetsonNTIO();
        var log =
                NetworkTableInstance.getDefault()
                        .getStringTopic("/photonvision/jetson/log")
                        .publish(PubSubOption.SEND_ALL, PubSubOption.KEEP_DUPLICATES);
        // Two identical lines in a row are two events (the same USB error twice).
        log.set("[WARN] usb 1-2.1: disconnect");
        log.set("[WARN] usb 1-2.1: disconnect");
        log.set("[INFO] Quiet mode on (the match ended)");

        JetsonInputs inputs = new JetsonInputs();
        io.updateInputs(inputs);
        assertArrayEquals(
                new String[] {
                    "[WARN] usb 1-2.1: disconnect",
                    "[WARN] usb 1-2.1: disconnect",
                    "[INFO] Quiet mode on (the match ended)"
                },
                inputs.logLines);
        io.updateInputs(inputs);
        assertArrayEquals(new String[0], inputs.logLines);
        log.close();
    }
}
