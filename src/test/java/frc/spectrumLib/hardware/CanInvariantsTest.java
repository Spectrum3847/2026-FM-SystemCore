package frc.spectrumLib.hardware;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.signals.MotorAlignmentValue;
import frc.spectrumLib.hardware.CanInvariants.Device;
import frc.spectrumLib.hardware.CanInvariants.DeviceType;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.util.CanDeviceId;
import java.util.List;
import org.junit.jupiter.api.Test;

/** Each check catches the mistake it is for, so a robot's configs passing them means something. */
class CanInvariantsTest {

    private static Mechanism.Config mechanism(String name, int id, String bus) {
        return new Mechanism.Config(name, id, bus);
    }

    private static Mechanism.FollowerConfig follower(String name, int id, String bus) {
        return new Mechanism.FollowerConfig(name, id, bus, MotorAlignmentValue.Aligned);
    }

    @Test
    void sameTypeSameIdSameBusIsADuplicate() {
        var a = new Device("A", DeviceType.TALON_FX, new CanDeviceId(4, "*"));
        var b = new Device("B", DeviceType.TALON_FX, new CanDeviceId(4, "*"));
        assertEquals(1, CanInvariants.duplicateIds(List.of(a, b)).size());
    }

    @Test
    void otherTypeOrOtherBusIsNot() {
        var talon = new Device("A", DeviceType.TALON_FX, new CanDeviceId(1, "*"));
        var candle = new Device("B", DeviceType.CANDLE, new CanDeviceId(1, "*"));
        var otherBus = new Device("C", DeviceType.TALON_FX, new CanDeviceId(1, "systemcore:0"));
        assertTrue(CanInvariants.duplicateIds(List.of(talon, candle, otherBus)).isEmpty());
    }

    @Test
    void aFollowerIsCheckedLikeAnyOtherMotor() {
        var a = mechanism("A", 4, "*");
        a.setFollowerConfigs(follower("A2", 5, "*"));
        var b = mechanism("B", 5, "*");
        var devices = new java.util.ArrayList<Device>();
        devices.addAll(CanInvariants.devices(a));
        devices.addAll(CanInvariants.devices(b));
        assertEquals(1, CanInvariants.duplicateIds(devices).size());
    }

    @Test
    void aDeviceOnAnUnopenedBusIsFlagged() {
        var d = new Device("A", DeviceType.TALON_FX, new CanDeviceId(1, "rio"));
        assertEquals(1, CanInvariants.busesNotInUse(List.of(d), List.of("*")).size());
        assertTrue(CanInvariants.busesNotInUse(List.of(d), List.of("*", "rio")).isEmpty());
    }

    @Test
    void aFollowerOffItsLeadersBusIsFlagged() {
        var m = mechanism("A", 4, "*");
        m.setFollowerConfigs(follower("same", 5, "*"), follower("other", 6, "systemcore:0"));
        var violations = CanInvariants.followersOffLeaderBus(m);
        assertEquals(1, violations.size());
        assertTrue(violations.get(0).contains("other"));
    }

    @Test
    void aMotorClaimedTwiceIsShared() {
        var a = mechanism("A", 4, "*");
        var b = mechanism("B", 7, "*");
        b.setFollowerConfigs(follower("B2", 4, "*"));
        assertEquals(1, CanInvariants.sharedDevices(List.of(a, b)).size());

        var self = mechanism("Self", 9, "*");
        self.setFollowerConfigs(follower("Self2", 9, "*"));
        assertEquals(1, CanInvariants.sharedDevices(List.of(self)).size());

        var c = mechanism("C", 4, "systemcore:0");
        assertTrue(CanInvariants.sharedDevices(List.of(a, c)).isEmpty());
    }

    @Test
    void softLimitsMustBracketEveryHomePosition() {
        var m = mechanism("A", 4, "*");
        m.configReverseSoftLimit(0.0, true);
        m.configForwardSoftLimit(2.8, true);
        assertTrue(CanInvariants.softLimitsBracket(m, 0.0, 2.8, 1.0).isEmpty());
        assertEquals(1, CanInvariants.softLimitsBracket(m, -0.01).size());
        assertEquals(1, CanInvariants.softLimitsBracket(m, 2.81).size());
    }

    @Test
    void invertedSoftLimitsAreFlagged() {
        var m = mechanism("A", 4, "*");
        m.configReverseSoftLimit(1.0, true);
        m.configForwardSoftLimit(0.5, true);
        assertEquals(1, CanInvariants.softLimitsBracket(m).size());
    }

    @Test
    void aDisabledLimitBoundsNothing() {
        var m = mechanism("A", 4, "*");
        m.configReverseSoftLimit(0.0, true);
        m.configForwardSoftLimit(-5.0, false);
        assertTrue(CanInvariants.softLimitsBracket(m, 100.0).isEmpty());
        assertEquals(1, CanInvariants.softLimitsBracket(m, -1.0).size());
    }
}
