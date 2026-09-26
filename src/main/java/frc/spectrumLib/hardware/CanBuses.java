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
 * <p><b>Check against the wiring before the event:</b> {@link #USE_CANIVORE} and the port of each
 * group.
 */
public final class CanBuses {
    private CanBuses() {}

    /** Prefix for a SystemCore native port, e.g. {@code "systemcore:0"}. */
    public static final String SYSTEMCORE_PREFIX = "systemcore:";

    /**
     * Whether FM runs its CANivore. {@code true} is the 2026 wiring: drivetrain and most mechanisms
     * on the CANivore, intake rollers on SystemCore port 0. {@code false} spreads everything over
     * the SystemCore's native ports instead (see {@link #DRIVETRAIN}, {@link #SHOOTER}, {@link
     * #MECHANISMS}); no {@code "*"} bus is ever opened.
     */
    public static final boolean USE_CANIVORE = false;

    /** The first CANivore found. */
    public static final String CANIVORE = "*";

    /**
     * Where the devices that were on the roboRIO's own CAN bus in 2026 (the intake rollers) now
     * live: SystemCore CAN port 0.
     */
    public static final String RIO_CANBUS = SYSTEMCORE_PREFIX + "0";

    /**
     * Swerve: 8 TalonFX, 4 CANcoders and the Pigeon. Without the CANivore it gets a port to itself,
     * for the 250 Hz odometry signals.
     */
    public static final String DRIVETRAIN = USE_CANIVORE ? CANIVORE : SYSTEMCORE_PREFIX + "1";

    /** Launcher (4 TalonFX) and hood. */
    public static final String SHOOTER = USE_CANIVORE ? CANIVORE : SYSTEMCORE_PREFIX + "2";

    /** Intake extension, indexer bed, indexer tower and the CANdle. */
    public static final String MECHANISMS = USE_CANIVORE ? CANIVORE : SYSTEMCORE_PREFIX + "3";

    /**
     * The buses this layout uses, by the name their health is logged under ({@code
     * <label>/StatusOK}, {@code <label>/BusUtilization}, ...).
     *
     * @return label to bus name, in port order
     */
    public static java.util.Map<String, String> inUse() {
        java.util.Map<String, String> buses = new java.util.LinkedHashMap<>();
        if (USE_CANIVORE) {
            buses.put("CANivore", CANIVORE);
        }
        for (String name : new String[] {RIO_CANBUS, DRIVETRAIN, SHOOTER, MECHANISMS}) {
            if (name.startsWith(SYSTEMCORE_PREFIX)) {
                buses.putIfAbsent(
                        "SystemCoreCAN" + name.substring(SYSTEMCORE_PREFIX.length()), name);
            }
        }
        return buses;
    }

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
