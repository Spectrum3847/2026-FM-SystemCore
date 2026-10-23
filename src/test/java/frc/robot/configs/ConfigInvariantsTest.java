package frc.robot.configs;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.Robot;
import frc.robot.subsystems.hood.HoodHoming;
import frc.robot.subsystems.intakeExtension.IntakeExtension.IntakeExtensionRight.RightConfig;
import frc.robot.subsystems.leds.Leds;
import frc.spectrumLib.hardware.CanBuses;
import frc.spectrumLib.hardware.CanInvariants;
import frc.spectrumLib.hardware.CanInvariants.Device;
import frc.spectrumLib.hardware.CanInvariants.DeviceType;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.util.CanDeviceId;
import java.io.File;
import java.net.URISyntaxException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.function.Function;
import java.util.stream.Stream;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;

/**
 * The CAN invariants ({@link CanInvariants}) over every robot config in this package, so a CAN ID,
 * bus or soft limit typed wrong fails the build instead of a robot.
 *
 * <p>These read the configs and never change them: the values are hard stops (AGENTS.md). A
 * violation found here is reported, not fixed.
 */
class ConfigInvariantsTest {

    /** Every {@link Robot.Config} subclass in this package: one per robot identity. */
    static Stream<Arguments> robots() throws URISyntaxException, ReflectiveOperationException {
        String pkg = ConfigInvariantsTest.class.getPackageName();
        // The robot's own classes directory, not this test's, which has the same package.
        File classes =
                new File(Robot.class.getProtectionDomain().getCodeSource().getLocation().toURI());
        File dir = new File(classes, pkg.replace('.', '/'));
        List<Arguments> out = new ArrayList<>();
        for (File f : dir.listFiles()) {
            String name = f.getName();
            if (!name.endsWith(".class") || name.contains("$")) {
                continue;
            }
            Class<?> c = Class.forName(pkg + "." + name.substring(0, name.length() - 6));
            if (Robot.Config.class.isAssignableFrom(c)) {
                out.add(
                        Arguments.of(
                                c.getSimpleName(),
                                (Robot.Config) c.getDeclaredConstructor().newInstance()));
            }
        }
        return out.stream();
    }

    @Test
    void everyRobotIdentityIsChecked() throws Exception {
        List<Object> names = robots().map(a -> a.get()[0]).toList();
        // Not a list to keep in sync: a new config class is picked up on its own. This only
        // guards against the discovery above quietly finding nothing.
        assertTrue(names.contains("FM2026"), "found " + names);
        assertTrue(names.size() >= 2, "found " + names);
    }

    /**
     * Mechanisms a config builds that are not fields of {@link Robot.Config}: the intake
     * extension's right axis, made from the left's config inside the subsystem.
     */
    private static List<Mechanism.Config> nestedMechanisms(Robot.Config config) {
        return List.of(new RightConfig(config.intakeExtension));
    }

    /**
     * Every position the code writes into each soft-limited motor, in mechanism rotations. A
     * soft-limited mechanism missing here fails {@link #softLimitsBracketHome}, so a new one has to
     * say where it homes.
     */
    private static final Map<String, Function<Robot.Config, double[]>> HOME_ROTATIONS =
            Map.of(
                    // Degrees in the config; Mechanism.degreesToRotations is deg / 360.
                    "Hood",
                    c ->
                            new double[] {
                                c.hood.getInitPosition() / 360,
                                HoodHoming.Params.fm().stopDeg() / 360
                            },
                    // Boot position, and the extended stop that resync and LB+Y zero at.
                    "IntakeExtension",
                    c ->
                            new double[] {
                                c.intakeExtension.getInitPosition(),
                                c.intakeExtension.getMaxRotations()
                            },
                    "IntakeExtensionRight",
                    c ->
                            new double[] {
                                c.intakeExtension.getInitPosition(),
                                c.intakeExtension.getMaxRotations()
                            });

    /** The mechanisms a config builds, attached or not. */
    static List<Mechanism.Config> mechanisms(Robot.Config config) {
        List<Mechanism.Config> out = new ArrayList<>();
        for (var field : Robot.Config.class.getFields()) {
            if (Mechanism.Config.class.isAssignableFrom(field.getType())) {
                try {
                    out.add((Mechanism.Config) field.get(config));
                } catch (IllegalAccessException e) {
                    throw new AssertionError(e);
                }
            }
        }
        out.addAll(nestedMechanisms(config));
        return out;
    }

    /** Every CAN device the robot opens: attached mechanisms, the drivetrain and the LEDs. */
    static List<Device> devices(Robot.Config config) {
        List<Device> out = new ArrayList<>();
        for (Mechanism.Config m : mechanisms(config)) {
            if (m.isAttached()) {
                out.addAll(CanInvariants.devices(m));
            }
        }
        var swerve = config.swerve;
        String bus = swerve.getDrivetrainConstants().CANBusName;
        String[] corners = {"FrontLeft", "FrontRight", "BackLeft", "BackRight"};
        var modules = swerve.getModules();
        for (int i = 0; i < modules.length; i++) {
            var m = modules[i];
            String name = "Swerve/" + corners[i];
            out.add(
                    new Device(
                            name + "/Drive",
                            DeviceType.TALON_FX,
                            new CanDeviceId(m.DriveMotorId, bus)));
            out.add(
                    new Device(
                            name + "/Steer",
                            DeviceType.TALON_FX,
                            new CanDeviceId(m.SteerMotorId, bus)));
            out.add(
                    new Device(
                            name + "/Encoder",
                            DeviceType.CANCODER,
                            new CanDeviceId(m.EncoderId, bus)));
        }
        out.add(
                new Device(
                        "Swerve/Pigeon",
                        DeviceType.PIGEON2,
                        new CanDeviceId(swerve.getDrivetrainConstants().Pigeon2Id, bus)));
        var leds = Leds.ledsConfig;
        if (leds.isAttached() && leds.getSharedCandle() == null) {
            out.add(
                    new Device(
                            leds.getName(),
                            DeviceType.CANDLE,
                            new CanDeviceId(leds.getDeviceId(), leds.getCanBus().getName())));
        }
        return out;
    }

    private static void assertNone(List<String> violations) {
        assertTrue(violations.isEmpty(), String.join("\n", violations));
    }

    @ParameterizedTest(name = "{0}")
    @MethodSource("robots")
    void canIdsAreUniquePerBus(String robot, Robot.Config config) {
        assertNone(CanInvariants.duplicateIds(devices(config)));
    }

    @ParameterizedTest(name = "{0}")
    @MethodSource("robots")
    void everyDeviceIsOnABusTheRobotOpens(String robot, Robot.Config config) {
        // inUse() returns names like "*"; a CANBus built from "*" reports its name as "*" too.
        assertNone(CanInvariants.busesNotInUse(devices(config), CanBuses.inUse().values()));
    }

    @ParameterizedTest(name = "{0}")
    @MethodSource("robots")
    void followersAreOnTheirLeadersBus(String robot, Robot.Config config) {
        List<String> violations = new ArrayList<>();
        for (Mechanism.Config m : mechanisms(config)) {
            violations.addAll(CanInvariants.followersOffLeaderBus(m));
        }
        assertNone(violations);
    }

    @ParameterizedTest(name = "{0}")
    @MethodSource("robots")
    void noTwoMechanismsShareADevice(String robot, Robot.Config config) {
        var all = mechanisms(config);
        assertNone(CanInvariants.sharedDevices(all));
        assertEquals(
                all.size(),
                all.stream().map(Mechanism.Config::getName).distinct().count(),
                "two mechanisms share a name, and so their log keys and alerts");
    }

    @ParameterizedTest(name = "{0}")
    @MethodSource("robots")
    void softLimitsBracketHome(String robot, Robot.Config config) {
        List<String> violations = new ArrayList<>();
        for (Mechanism.Config m : mechanisms(config)) {
            if (!CanInvariants.hasSoftLimits(m)) {
                continue;
            }
            var home = HOME_ROTATIONS.get(m.getName());
            if (home == null) {
                violations.add(
                        m.getName()
                                + " has soft limits but no entry in HOME_ROTATIONS: add every"
                                + " position the code sets its motor to");
                continue;
            }
            violations.addAll(CanInvariants.softLimitsBracket(m, home.apply(config)));
        }
        assertNone(violations);
    }

    @Test
    void homeTableNamesRealMechanisms() throws Exception {
        var names =
                robots().map(a -> (Robot.Config) a.get()[1])
                        .flatMap(c -> mechanisms(c).stream())
                        .map(Mechanism.Config::getName)
                        .distinct()
                        .toList();
        for (String name : HOME_ROTATIONS.keySet()) {
            assertTrue(names.contains(name), name + " is in HOME_ROTATIONS but no config has it");
        }
        assertFalse(names.isEmpty());
    }
}
