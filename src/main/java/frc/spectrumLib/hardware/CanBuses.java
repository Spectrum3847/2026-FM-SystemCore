package frc.spectrumLib.hardware;

import com.ctre.phoenix6.CANBus;

/**
 * The CAN buses FM runs on under SystemCore, and the one place a bus name becomes a {@link CANBus}.
 *
 * <p>On the 2026 roboRIO FM had two buses: a CANivore ({@code "*"}) carrying the drivetrain and
 * most mechanisms, and the roboRIO's own bus ({@code "rio"}) carrying the intake rollers.
 * SystemCore has no {@code "rio"} bus; it has five native CAN FD ports, addressed with {@link
 * CANBus#systemcore(int)}. A CANivore still works on SystemCore once CTRE's {@code canivore-usb}
 * package is installed on it (see wpilibsuite/SystemcoreTesting {@code CTR-Phoenix.md}), so the
 * CANivore half of the robot keeps its wiring and only the roboRIO-bus devices move.
 *
 * <p>Device configs keep carrying bus <em>names</em> (strings) as they did in 2026. {@link
 * #forName(String)} resolves {@link #SYSTEMCORE_PREFIX}{@code n} to {@code CANBus.systemcore(n)}
 * and anything else to a CANivore by name.
 *
 * <p><b>Check against the wiring before the event:</b> {@link #CANIVORE} and {@link #RIO_CANBUS}.
 */
public final class CanBuses {
    private CanBuses() {}

    /** Prefix for a SystemCore native port, e.g. {@code "systemcore:0"}. */
    public static final String SYSTEMCORE_PREFIX = "systemcore:";

    /** The first CANivore found. FM's drivetrain and most mechanisms. */
    public static final String CANIVORE = "*";

    /**
     * Where the devices that were on the roboRIO's own CAN bus in 2026 (the intake rollers) now
     * live: SystemCore CAN port 0.
     */
    public static final String RIO_CANBUS = SYSTEMCORE_PREFIX + "0";

    /**
     * Resolves a bus name from a config into a Phoenix {@link CANBus}.
     *
     * @param name {@code "systemcore:n"} for a native SystemCore port, else a CANivore name
     * @return the bus
     */
    public static CANBus forName(String name) {
        return forName(name, "");
    }

    /**
     * Resolves a bus name into a Phoenix {@link CANBus} that also records a hoot log.
     *
     * @param name {@code "systemcore:n"} for a native SystemCore port, else a CANivore name
     * @param hootPath hoot log path, or empty for none
     * @return the bus
     */
    public static CANBus forName(String name, String hootPath) {
        if (name != null && name.startsWith(SYSTEMCORE_PREFIX)) {
            int port = Integer.parseInt(name.substring(SYSTEMCORE_PREFIX.length()));
            return hootPath.isEmpty() ? CANBus.systemcore(port) : CANBus.systemcore(port, hootPath);
        }
        return hootPath.isEmpty() ? new CANBus(name) : new CANBus(name, hootPath);
    }
}
