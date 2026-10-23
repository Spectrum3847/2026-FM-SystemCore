package frc.robot;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.subsystems.SuperStructure.WantedSuperState;
import frc.spectrumLib.gamepads.Gamepad;
import frc.spectrumLib.gamepads.GamepadLayout;
import java.io.File;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.command2.Subsystem;
import org.wpilib.driverstation.POVDirection;
import org.wpilib.framework.IterativeRobotBase;
import org.wpilib.hardware.hal.AllianceStationID;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.simulation.GenericHIDSim;
import org.wpilib.simulation.SimHooks;

/**
 * Boots the whole robot in simulation, headless, and presses every pilot and operator input, alone
 * and with each modifier, disabled and in teleop. Fails if any press throws, if a binding in {@link
 * #BINDINGS} does not schedule its command (or reach its wanted state), or if a single button
 * schedules a command that is not in {@link #BINDINGS}.
 *
 * <p>The sweep is generic: it walks the gamepad, not the binding list. {@link #BINDINGS} is the
 * robot's part, the README's binding table in code.
 *
 * <p>The robot loop is stepped by hand on a paused sim clock ({@link SimHooks#stepTiming}), the
 * same calls {@code LoggedRobot.startCompetition} makes each cycle, so the timing is the same on
 * every machine. Runs in its own JVM ({@code ./gradlew simTest}): it starts the logger and fills
 * the command scheduler.
 */
@Tag("sim")
class BindingsSmokeTest {

    /** Loops a combination is held, then let go: 0.12 s, past the chords' 0.08 s settle. */
    private static final int HOLD_LOOPS = 12;

    private static final int RELEASE_LOOPS = 8;

    /** Every input on a gamepad. */
    enum Input {
        A,
        B,
        X,
        Y,
        LB,
        RB,
        BACK,
        START,
        LEFT_STICK,
        RIGHT_STICK,
        UP,
        DOWN,
        LEFT,
        RIGHT,
        LT,
        RT
    }

    enum Mode {
        DISABLED,
        TELEOP
    }

    /**
     * What a binding does: the wanted state it sets on press and on release (null: no state), or
     * the name of the command it schedules (null: any command).
     */
    record Expect(WantedSuperState onPress, WantedSuperState onRelease, String command) {
        static Expect state(WantedSuperState onPress) {
            return new Expect(onPress, WantedSuperState.IDLE, null);
        }

        static Expect command(String name) {
            return new Expect(null, null, name);
        }

        static Expect anyCommand() {
            return new Expect(null, null, null);
        }
    }

    /** {@code "<mode> <pad> <inputs>"}, e.g. {@code "TELEOP pilot LB+A"}, to what it does. */
    private static final Map<String, Expect> BINDINGS = new LinkedHashMap<>();

    static {
        // Pilot, teleop (README "Bindings").
        bind("TELEOP pilot LT", Expect.state(WantedSuperState.INTAKE_FUEL));
        bind("TELEOP pilot RT", Expect.state(WantedSuperState.LAUNCH_WITH_SQUEEZE));
        bind("TELEOP pilot LT+RT", Expect.state(WantedSuperState.LAUNCH_WITHOUT_SQUEEZE));
        bind("TELEOP pilot LB+LT", Expect.state(WantedSuperState.EJECT));
        bind("TELEOP pilot A", Expect.state(WantedSuperState.UNJAM));
        bind("TELEOP pilot X", Expect.state(WantedSuperState.TRACK_TARGET));
        bind("TELEOP pilot LB+A", Expect.state(WantedSuperState.SET_SHOT));
        bind("TELEOP pilot LB+X", Expect.state(WantedSuperState.SET_SHOT));
        bind("TELEOP pilot LB+B", Expect.state(WantedSuperState.SET_SHOT));
        bind("TELEOP pilot BACK", Expect.state(WantedSuperState.FORCE_HOME));
        bind("TELEOP pilot LB+UP", Expect.command("reorientForward"));
        bind("TELEOP pilot LB+LEFT", Expect.command("reorientLeft"));
        bind("TELEOP pilot LB+DOWN", Expect.command("reorientBack"));
        bind("TELEOP pilot LB+RIGHT", Expect.command("reorientRight"));
        // Pilot, disabled: coast and brake.
        bind("DISABLED pilot A", Expect.anyCommand());
        bind("DISABLED pilot B", Expect.anyCommand());
        // Operator, teleop. Y (feed override) is polled, not scheduled: see feedOverrideIsHeld.
        bind("TELEOP operator LB+X", Expect.command("Vision.resetQuest"));
        bind("TELEOP operator LB+Y", Expect.anyCommand());
        bind("TELEOP operator BACK", Expect.state(WantedSuperState.FORCE_HOME));
        // Releases mechanisms held by their tracking stop (SuperStructure.clearTrackingStops).
        bind("TELEOP operator START", Expect.anyCommand());
        for (String trim : new String[] {"UP", "DOWN", "LEFT", "RIGHT"}) {
            bind("TELEOP operator " + trim, Expect.anyCommand());
            bind("DISABLED operator " + trim, Expect.anyCommand());
        }
        // Operator, disabled: coast, brake, and the two that ignore disable.
        bind("DISABLED operator A", Expect.anyCommand());
        bind("DISABLED operator B", Expect.anyCommand());
        bind("DISABLED operator LB+X", Expect.command("Vision.resetQuest"));
        bind("DISABLED operator LB+Y", Expect.anyCommand());
    }

    private static void bind(String key, Expect expect) {
        BINDINGS.put(key, expect);
    }

    private static Robot robot;
    private static Method beforeUser, afterUser, loopFunc;
    private static long bootMillis;
    private static final Set<File> logsBefore = new java.util.HashSet<>();

    /** Commands initialized since the last {@link #clearInitialized}, default commands left out. */
    private static final List<Command> initialized = new ArrayList<>();

    @BeforeAll
    static void boot() throws Exception {
        File[] logs = new File("logs").listFiles();
        if (logs != null) {
            logsBefore.addAll(List.of(logs));
        }
        assertTrue(HAL.initialize(500, 0), "HAL failed to initialize");
        SimHooks.pauseTiming();
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setAllianceStationId(AllianceStationID.BLUE_1);
        DriverStationSim.setRobotMode(RobotMode.TELEOPERATED);
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();

        long start = System.nanoTime();
        robot = new Robot();
        bootMillis = (System.nanoTime() - start) / 1_000_000;

        // What LoggedRobot.startCompetition calls around each loop.
        beforeUser = Logger.class.getDeclaredMethod("periodicBeforeUser");
        beforeUser.setAccessible(true);
        afterUser = Logger.class.getDeclaredMethod("periodicAfterUser", long.class, long.class);
        afterUser.setAccessible(true);
        loopFunc = IterativeRobotBase.class.getDeclaredMethod("loopFunc");
        loopFunc.setAccessible(true);
        robot.simulationInit();

        pilot = new Pad("pilot", 0, Robot.getPilot());
        operator = new Pad("operator", 1, Robot.getOperator());
        CommandScheduler.getInstance()
                .onCommandInitialize(
                        c -> {
                            if (!isDefaultCommand(c)) {
                                initialized.add(c);
                            }
                        });
        step(50);
    }

    @AfterAll
    static void cleanUp() {
        Logger.end();
        // The run's log is the robot's sim log; this test's results are in the test report.
        File[] logs = new File("logs").listFiles();
        if (logs != null) {
            for (File f : logs) {
                if (!logsBefore.contains(f) && f.getName().endsWith(".wpilog")) {
                    f.delete();
                }
            }
        }
    }

    private static boolean isDefaultCommand(Command c) {
        for (Subsystem s : c.getRequirements()) {
            if (CommandScheduler.getInstance().getDefaultCommand(s) == c) {
                return true;
            }
        }
        return false;
    }

    private static void step(int loops) throws Exception {
        for (int i = 0; i < loops; i++) {
            SimHooks.stepTiming(Constants.LOOP_PERIOD_SECONDS);
            beforeUser.invoke(null);
            try {
                loopFunc.invoke(robot);
            } catch (InvocationTargetException e) {
                throw (Exception) (e.getCause() instanceof Exception ? e.getCause() : e);
            }
            afterUser.invoke(null, 0L, 0L);
        }
    }

    private static Pad pilot;
    private static Pad operator;

    /** One controller on the simulated Driver Station, written in the layout its code reads. */
    private static final class Pad {
        final String name;
        final GenericHIDSim hid;
        final GamepadLayout layout;
        final Gamepad gamepad;

        Pad(String name, int port, Gamepad gamepad) {
            this.name = name;
            this.gamepad = gamepad;
            this.hid = new GenericHIDSim(port);
            this.layout = gamepad.getLayout();
            layout.simulateController(hid);
            DriverStationSim.setJoystickIsGamepad(port, true);
            DriverStationSim.setJoystickName(port, "BindingsSmokeTest " + name);
            releaseAll();
        }

        void set(Input input, boolean down) {
            switch (input) {
                case A -> hid.setRawButton(layout.a, down);
                case B -> hid.setRawButton(layout.b, down);
                case X -> hid.setRawButton(layout.x, down);
                case Y -> hid.setRawButton(layout.y, down);
                case LB -> hid.setRawButton(layout.leftBumper, down);
                case RB -> hid.setRawButton(layout.rightBumper, down);
                case BACK -> hid.setRawButton(layout.back, down);
                case START -> hid.setRawButton(layout.start, down);
                case LEFT_STICK -> hid.setRawButton(layout.leftStick, down);
                case RIGHT_STICK -> hid.setRawButton(layout.rightStick, down);
                case LT -> hid.setRawAxis(layout.leftTrigger, down ? 1.0 : 0.0);
                case RT -> hid.setRawAxis(layout.rightTrigger, down ? 1.0 : 0.0);
                case UP, DOWN, LEFT, RIGHT -> setDpad(input, down);
            }
        }

        private void setDpad(Input input, boolean down) {
            if (layout.dpadOnPov()) {
                POVDirection dir =
                        switch (input) {
                            case UP -> POVDirection.UP;
                            case DOWN -> POVDirection.DOWN;
                            case LEFT -> POVDirection.LEFT;
                            default -> POVDirection.RIGHT;
                        };
                hid.setPOV(0, down ? dir : POVDirection.CENTER);
                return;
            }
            int button =
                    switch (input) {
                        case UP -> layout.dpadUp;
                        case DOWN -> layout.dpadDown;
                        case LEFT -> layout.dpadLeft;
                        default -> layout.dpadRight;
                    };
            hid.setRawButton(button, down);
        }

        void releaseAll() {
            for (Input i : Input.values()) {
                set(i, false);
            }
            hid.notifyNewData();
        }
    }

    private static void setMode(Mode mode) throws Exception {
        DriverStationSim.setRobotMode(RobotMode.TELEOPERATED);
        DriverStationSim.setEnabled(mode == Mode.TELEOP);
        DriverStationSim.notifyNewData();
        step(20);
    }

    /** Every combination the sweep presses: each input alone, with LB, and LT with RT. */
    private static List<List<Input>> combinations() {
        List<List<Input>> out = new ArrayList<>();
        for (Input i : Input.values()) {
            out.add(List.of(i));
        }
        for (Input i : Input.values()) {
            if (i != Input.LB) {
                out.add(List.of(i, Input.LB));
            }
        }
        out.add(List.of(Input.LT, Input.RT));
        return out;
    }

    private static String key(Mode mode, Pad pad, List<Input> inputs) {
        return mode
                + " "
                + pad.name
                + " "
                + String.join("+", inputs.stream().map(Enum::name).toList());
    }

    /** The bindings table names the LB chords both ways; the sweep presses each one way. */
    private static Expect lookup(Mode mode, Pad pad, List<Input> inputs) {
        Expect e = BINDINGS.get(key(mode, pad, inputs));
        if (e == null && inputs.size() == 2) {
            e = BINDINGS.get(key(mode, pad, List.of(inputs.get(1), inputs.get(0))));
        }
        return e;
    }

    private static String names(List<Command> commands) {
        return commands.stream().map(Command::getName).toList().toString();
    }

    @Test
    void everyInputAndChordSurvivesAndEveryBindingSchedules() throws Exception {
        List<String> problems = new ArrayList<>();
        Set<String> exercised = new java.util.HashSet<>();
        long start = System.nanoTime();
        for (Mode mode : Mode.values()) {
            setMode(mode);
            for (Pad pad : List.of(pilot, operator)) {
                for (List<Input> inputs : combinations()) {
                    String key = key(mode, pad, inputs);
                    Expect expect = lookup(mode, pad, inputs);
                    if (expect != null) {
                        exercised.add(key);
                    }
                    press(key, mode, pad, inputs, expect, problems);
                }
            }
        }
        long millis = (System.nanoTime() - start) / 1_000_000;
        System.out.printf("BindingsSmokeTest: boot %d ms, sweep %d ms%n", bootMillis, millis);

        for (String key : BINDINGS.keySet()) {
            String[] parts = key.split(" ");
            String[] in = parts[2].split("\\+");
            String swapped =
                    in.length == 2 ? parts[0] + " " + parts[1] + " " + in[1] + "+" + in[0] : key;
            if (!exercised.contains(key) && !exercised.contains(swapped)) {
                problems.add(key + ": in BINDINGS but the sweep never presses it");
            }
        }
        assertTrue(problems.isEmpty(), "\n" + String.join("\n", problems));
    }

    private static void press(
            String key, Mode mode, Pad pad, List<Input> inputs, Expect expect, List<String> out)
            throws Exception {
        var superStructure = Robot.getSuperStructure();
        if (superStructure.getWantedSuperState() != WantedSuperState.IDLE) {
            // Test reset, not a binding: a press starts from IDLE so its own state shows.
            superStructure.setWantedSuperState(WantedSuperState.IDLE);
        }
        initialized.clear();
        try {
            for (Input i : inputs) {
                pad.set(i, true);
            }
            pad.hid.notifyNewData();
            step(HOLD_LOOPS);
        } catch (Exception | Error e) {
            throw new AssertionError("Crashed on press: " + key, e);
        }
        List<Command> onPress = new ArrayList<>(initialized);
        WantedSuperState pressed = superStructure.getWantedSuperState();
        initialized.clear();
        try {
            pad.releaseAll();
            step(RELEASE_LOOPS);
        } catch (Exception | Error e) {
            throw new AssertionError("Crashed on release: " + key, e);
        }
        WantedSuperState released = superStructure.getWantedSuperState();

        if (expect == null) {
            // Chords and modifiers alone may set off their parts' bindings; a lone input that
            // schedules something must be in the table.
            if (inputs.size() == 1 && !onPress.isEmpty()) {
                out.add(key + ": scheduled " + names(onPress) + " but is not in BINDINGS");
            }
            return;
        }
        if (onPress.isEmpty()) {
            out.add(key + ": scheduled nothing");
        }
        if (expect.command() != null
                && onPress.stream().noneMatch(c -> c.getName().equals(expect.command()))) {
            out.add(key + ": expected " + expect.command() + ", scheduled " + names(onPress));
        }
        if (expect.onPress() != null && pressed != expect.onPress()) {
            out.add(key + ": wanted state " + pressed + ", expected " + expect.onPress());
        }
        if (expect.onRelease() != null && released != expect.onRelease()) {
            out.add(key + ": after release " + released + ", expected " + expect.onRelease());
        }
    }

    @Test
    void feedOverrideIsHeldOnlyWithoutLb() throws Exception {
        setMode(Mode.TELEOP);
        var override = Robot.getOperator().feedOverride_Y;
        operator.set(Input.Y, true);
        operator.hid.notifyNewData();
        step(2);
        assertTrue(override.getAsBoolean(), "operator Y should hold the feed override");
        operator.set(Input.LB, true);
        operator.hid.notifyNewData();
        step(2);
        assertFalse(override.getAsBoolean(), "LB+Y is the intake reset, not the feed override");
        operator.releaseAll();
        step(RELEASE_LOOPS);
    }
}
