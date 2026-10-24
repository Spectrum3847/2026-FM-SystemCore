package frc.spectrumLib.checks;

import frc.spectrumLib.checks.CheckResult.Status;
import frc.spectrumLib.mechanism.Mechanism;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

/**
 * Every device answering, none rebooted recently, grouped by bus (or by kind, for coprocessors).
 *
 * <p>A device is missing once its {@code connected} input has been false for {@link
 * #MISSING_HOLD_SECONDS}, or if it has never answered. A reset counts only while the device is
 * connected, and only once it has been connected for {@link #ARM_SECONDS} at a stretch: Phoenix
 * reports its first contact with a device as a reset, and a disconnected device's flags mean
 * nothing. A device arms once per boot and stays armed through a dropout, since a brownout reboot
 * reports its reset on the first loop back.
 *
 * <p>A missing device fails the check unless its {@link Device#missingIs} says otherwise (a camera
 * that is only logged, not used, warns or is left out). A reboot in the last {@link
 * #RECENT_REBOOT_SECONDS} warns: the device works now, but its power wiring needs a look before it
 * browns out in a match.
 */
public final class DeviceHealthCheck implements Check {

    /** A one-loop dropout is not a missing device; half a second of silence is. */
    public static final double MISSING_HOLD_SECONDS = 0.5;

    /** Phoenix's first-contact reset report arrives well inside this. */
    public static final double ARM_SECONDS = 1.0;

    /** Covers the previous match and the queue before this one. */
    public static final double RECENT_REBOOT_SECONDS = 600;

    private final String what;
    private final List<Device> devices = new ArrayList<>();

    /** One device: its name, its group, and where its logged state comes from. */
    public static final class Device {
        final String group;
        final String name;
        final BooleanSupplier connected;
        BooleanSupplier reset = () -> false;
        Supplier<String> problem = () -> null;
        Supplier<Status> missing = () -> Status.FAIL;

        boolean everConnected = false;
        boolean armed = false;
        double connectedSince = Double.NaN;
        double missingSince = Double.NaN;
        double lastReset = Double.NaN;
        int resets = 0;

        Device(String group, String name, BooleanSupplier connected) {
            this.group = group;
            this.name = name;
            this.connected = connected;
        }

        /** Its reset input: true on the loop it reports having rebooted. */
        public Device resets(BooleanSupplier resetThisLoop) {
            this.reset = resetThisLoop;
            return this;
        }

        /**
         * How bad it is when this device is missing; {@link Status#FAIL} unless changed. {@link
         * Status#SKIP} leaves the device out of the check (e.g. a camera nothing uses right now).
         */
        public Device missingIs(Supplier<Status> status) {
            this.missing = status;
            return this;
        }

        /**
         * Anything else wrong with it while connected (e.g. its clock is off), as a short phrase,
         * or null. Reported as a warning.
         */
        public Device problem(Supplier<String> problem) {
            this.problem = problem;
            return this;
        }
    }

    /**
     * @param what what the devices are, for the pass message, e.g. {@code "CAN devices"}
     */
    public DeviceHealthCheck(String what) {
        this.what = what;
    }

    /**
     * Adds a device.
     *
     * @param group its bus, or another grouping shown in the reason
     * @param name its name in messages
     * @param connected its logged connected input
     * @return the device, to add a reset input or change its severity
     */
    public Device add(String group, String name, BooleanSupplier connected) {
        Device d = new Device(group, name, connected);
        devices.add(d);
        return d;
    }

    /**
     * Adds a mechanism's leader and followers from its logged {@code MotorInputs}. A mechanism with
     * no hardware attached is skipped.
     *
     * @param m the mechanism
     * @param group its bus label
     */
    public void addMechanism(Mechanism m, String group) {
        if (!m.isAttached()) {
            return;
        }
        var in = m.getInputs();
        Mechanism.Config config = m.config;
        add(group, m.getName() + " (" + config.getId().getDeviceNumber() + ")", () -> in.connected)
                .resets(() -> in.resetOccurred);
        var followers = config.getFollowerConfigs();
        for (int i = 0; i < followers.length; i++) {
            final int f = i;
            add(
                            group,
                            followers[i].getName()
                                    + " ("
                                    + followers[i].getId().getDeviceNumber()
                                    + ")",
                            () -> f < in.followerConnected.length && in.followerConnected[f])
                    .resets(
                            () ->
                                    f < in.followerResetOccurred.length
                                            && in.followerResetOccurred[f]);
        }
    }

    @Override
    public void observe(double now) {
        for (Device d : devices) {
            if (d.connected.getAsBoolean()) {
                if (Double.isNaN(d.connectedSince)) {
                    d.connectedSince = now;
                }
                d.everConnected = true;
                d.missingSince = Double.NaN;
                d.armed |= now - d.connectedSince >= ARM_SECONDS;
                if (d.armed && d.reset.getAsBoolean()) {
                    d.lastReset = now;
                    d.resets++;
                }
            } else {
                d.connectedSince = Double.NaN;
                if (Double.isNaN(d.missingSince)) {
                    d.missingSince = now;
                }
            }
        }
    }

    private static boolean missing(Device d, double now) {
        // Never seen at all counts at once; a dropout only once it has lasted.
        return Double.isNaN(d.connectedSince)
                && (!d.everConnected
                        || Double.isNaN(d.missingSince)
                        || now - d.missingSince >= MISSING_HOLD_SECONDS);
    }

    @Override
    public CheckResult evaluate(double now) {
        if (devices.isEmpty()) {
            return CheckResult.skip("no " + what);
        }
        Map<String, List<String>> missingByGroup = new LinkedHashMap<>();
        Status missingStatus = Status.PASS;
        List<String> rebooted = new ArrayList<>();
        List<String> problems = new ArrayList<>();
        int connected = 0;
        for (Device d : devices) {
            if (missing(d, now)) {
                Status s = d.missing.get();
                if (s != Status.SKIP) {
                    missingByGroup.computeIfAbsent(d.group, g -> new ArrayList<>()).add(d.name);
                    missingStatus = missingStatus.worst(s);
                }
                continue;
            }
            if (!Double.isNaN(d.connectedSince)) {
                connected++;
                String p = d.problem.get();
                if (p != null) {
                    problems.add(d.name + " " + p);
                }
            }
            if (!Double.isNaN(d.lastReset) && now - d.lastReset < RECENT_REBOOT_SECONDS) {
                // Whole minutes, so the line (and its alert) does not change every second.
                int minutes = (int) ((now - d.lastReset) / 60);
                rebooted.add(
                        d.name
                                + " rebooted "
                                + (minutes == 0 ? "in the last minute" : minutes + " min ago")
                                + (d.resets > 1 ? " (" + d.resets + "x)" : ""));
            }
        }
        if (!missingByGroup.isEmpty()) {
            List<String> parts = new ArrayList<>();
            for (var e : missingByGroup.entrySet()) {
                parts.add(e.getKey() + ": " + String.join(", ", e.getValue()) + " not answering");
            }
            String text = String.join("; ", parts);
            if (!rebooted.isEmpty()) {
                text += "; " + String.join(", ", rebooted);
            }
            return new CheckResult(missingStatus, text);
        }
        if (!rebooted.isEmpty() || !problems.isEmpty()) {
            List<String> parts = new ArrayList<>(rebooted);
            parts.addAll(problems);
            String text = String.join("; ", parts);
            if (!rebooted.isEmpty()) {
                text += " - check power wiring";
            }
            return CheckResult.warn(text);
        }
        return CheckResult.pass(connected + " " + what + " answering");
    }
}
