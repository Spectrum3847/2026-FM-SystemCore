package frc.spectrumLib.hardware;

import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.util.CanDeviceId;
import java.util.ArrayList;
import java.util.Collection;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Checks over a robot's CAN device configs. Each returns the violations it finds, one line each, so
 * a test can print them all at once. Nothing here knows about a particular robot: the robot's
 * configs are passed in.
 */
public final class CanInvariants {
    private CanInvariants() {}

    /**
     * Phoenix gives each device type its own ID space, so a TalonFX and a CANcoder may both be ID 3
     * on one bus. Two of the same type may not.
     */
    public enum DeviceType {
        TALON_FX,
        CANCODER,
        PIGEON2,
        CANDLE
    }

    /** One device the robot opens, and what opens it. */
    public record Device(String owner, DeviceType type, CanDeviceId id) {
        @Override
        public String toString() {
            return owner + " (" + type + " " + id.getDeviceNumber() + " on '" + id.getBus() + "')";
        }
    }

    /** Leader and followers of a mechanism, each as a TalonFX. */
    public static List<Device> devices(Mechanism.Config mechanism) {
        List<Device> out = new ArrayList<>();
        out.add(new Device(mechanism.getName(), DeviceType.TALON_FX, mechanism.getId()));
        for (Mechanism.FollowerConfig f : mechanism.getFollowerConfigs()) {
            out.add(
                    new Device(
                            mechanism.getName() + "/" + f.getName(),
                            DeviceType.TALON_FX,
                            f.getId()));
        }
        return out;
    }

    /** Two devices of the same type with the same ID on the same bus. */
    public static List<String> duplicateIds(Collection<Device> devices) {
        Map<String, Device> seen = new LinkedHashMap<>();
        List<String> out = new ArrayList<>();
        for (Device d : devices) {
            String key = d.id().getBus() + "|" + d.type() + "|" + d.id().getDeviceNumber();
            Device first = seen.putIfAbsent(key, d);
            if (first != null) {
                out.add(first + " and " + d + " have the same ID");
            }
        }
        return out;
    }

    /** A device on a bus the robot does not open (and so does not log the health of). */
    public static List<String> busesNotInUse(Collection<Device> devices, Collection<String> buses) {
        List<String> out = new ArrayList<>();
        for (Device d : devices) {
            if (!buses.contains(d.id().getBus())) {
                out.add(d + " is on a bus not in " + buses);
            }
        }
        return out;
    }

    /**
     * A follower on a different bus from its leader. Phoenix followers mirror the leader's status
     * frames, which only reach devices on the same bus.
     */
    public static List<String> followersOffLeaderBus(Mechanism.Config mechanism) {
        List<String> out = new ArrayList<>();
        String bus = mechanism.getId().getBus();
        for (Mechanism.FollowerConfig f : mechanism.getFollowerConfigs()) {
            if (!bus.equals(f.getId().getBus())) {
                out.add(
                        mechanism.getName()
                                + " follower "
                                + f.getName()
                                + " is on '"
                                + f.getId().getBus()
                                + "', its leader on '"
                                + bus
                                + "'");
            }
        }
        return out;
    }

    /**
     * A motor claimed by two mechanisms, or twice by one (a follower that is also its leader, or
     * listed twice). Two mechanisms driving one motor fight each other.
     */
    public static List<String> sharedDevices(Collection<Mechanism.Config> mechanisms) {
        Map<CanDeviceId, String> owner = new LinkedHashMap<>();
        List<String> out = new ArrayList<>();
        for (Mechanism.Config m : mechanisms) {
            for (Device d : devices(m)) {
                String first = owner.putIfAbsent(d.id(), d.owner());
                if (first != null) {
                    out.add(
                            "TalonFX "
                                    + d.id().getDeviceNumber()
                                    + " on '"
                                    + d.id().getBus()
                                    + "' is both "
                                    + first
                                    + " and "
                                    + d.owner());
                }
            }
        }
        return out;
    }

    /** Whether a mechanism has either soft limit on. */
    public static boolean hasSoftLimits(Mechanism.Config mechanism) {
        var limits = mechanism.getTalonConfig().SoftwareLimitSwitch;
        return limits.ForwardSoftLimitEnable || limits.ReverseSoftLimitEnable;
    }

    /**
     * Soft limits that are inverted, or that leave out a position the code writes into the motor
     * (its boot position, or where homing zeroes it). A motor told it is outside its soft limits
     * will only drive back toward them, and one with inverted limits will not move at all.
     *
     * @param mechanism the mechanism
     * @param homeRotations every position the code sets the motor to, in mechanism rotations
     */
    public static List<String> softLimitsBracket(
            Mechanism.Config mechanism, double... homeRotations) {
        var limits = mechanism.getTalonConfig().SoftwareLimitSwitch;
        double reverse =
                limits.ReverseSoftLimitEnable
                        ? limits.ReverseSoftLimitThreshold
                        : Double.NEGATIVE_INFINITY;
        double forward =
                limits.ForwardSoftLimitEnable
                        ? limits.ForwardSoftLimitThreshold
                        : Double.POSITIVE_INFINITY;
        List<String> out = new ArrayList<>();
        if (reverse >= forward) {
            out.add(
                    mechanism.getName()
                            + " reverse soft limit "
                            + reverse
                            + " is not below its forward limit "
                            + forward);
        }
        for (double home : homeRotations) {
            if (home < reverse || home > forward) {
                out.add(
                        mechanism.getName()
                                + " home position "
                                + home
                                + " rot is outside its soft limits ["
                                + reverse
                                + ", "
                                + forward
                                + "]");
            }
        }
        return out;
    }
}
