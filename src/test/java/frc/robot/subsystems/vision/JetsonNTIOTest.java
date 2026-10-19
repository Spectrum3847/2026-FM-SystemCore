package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.photonvision.proto.Photon.ProtobufDeviceMetrics;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.RawPublisher;

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
}
