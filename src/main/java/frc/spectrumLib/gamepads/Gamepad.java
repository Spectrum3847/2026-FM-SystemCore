package frc.spectrumLib.gamepads;

import frc.spectrumLib.telemetry.Alert;
import frc.spectrumLib.telemetry.Telemetry;
import frc.spectrumLib.util.AllianceSource;
import frc.spectrumLib.util.ExpCurve;
import frc.spectrumLib.util.Util;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.Setter;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.command2.Commands;
import org.wpilib.command2.InstantCommand;
import org.wpilib.command2.Subsystem;
import org.wpilib.command2.button.Trigger;
import org.wpilib.driverstation.GenericHID;
import org.wpilib.driverstation.GenericHID.RumbleType;
import org.wpilib.driverstation.POVDirection;
import org.wpilib.math.geometry.Rotation2d;

/**
 * Abstract base class for robot gamepad (Xbox-compatible) controllers.
 *
 * <p>Reads a WPILib {@link GenericHID} through a {@link GamepadLayout}, which says where each
 * control's data arrives for the Driver Station in use ({@link Config#setMapping}), and exposes:
 *
 * <ul>
 *   <li>Pre-built {@link Trigger} fields for every button, bumper, trigger, stick-click, and D-pad
 *       direction.
 *   <li>Composite modifier triggers ({@link #noBumpers}, {@link #bothTriggers}, etc.) for
 *       chord-based bindings.
 *   <li>Exponential-curve axis helpers ({@link #leftStickCurve}, etc.) for driver-tuned response.
 *   <li>Stick-direction utilities ({@link #getLeftStickDirection()}, {@link
 *       #chooseCardinalDirections()}) for field-relative driving.
 *   <li>Rumble commands ({@link #rumbleCommand(double, double, double)}) for haptic feedback.
 * </ul>
 *
 * <p>Subclass this once per operator role (pilot, copilot) and override {@link #setupStates()} and
 * {@link #setupDefaultCommand()} to bind subsystem commands to triggers.
 *
 * <p>When {@link Config#isAttached()} returns {@code false}, all triggers remain permanently {@code
 * false} and axis reads return {@code 0.0}.
 *
 * <p>Each loop it logs the controller's raw data and what the code reads from it under {@code
 * Gamepads/<name>/} (dashboard keys), for checking the mapping on the bench (see the README).
 */
// Gamepad class
public abstract class Gamepad implements Subsystem {
    /** WPILib alert displayed on the driver station when this gamepad is disconnected. */
    private Alert disconnectedAlert;

    /** A trigger that is always {@code false}; used as a safe default before hardware is ready. */
    public static final Trigger kFalse = new Trigger(() -> false);

    /** The controller on the Driver Station, read by raw index through {@link #layout}. */
    private GenericHID hid;

    /**
     * Where this controller's buttons and axes arrive. Fixed by {@link Config#getMapping()}, or
     * followed from the controller's data with {@link Mapping#AUTO}.
     */
    @Getter private GamepadLayout layout;

    /** What {@link GamepadLayout#detect(GenericHID)} made of the controller this loop. */
    private Optional<GamepadLayout> detectedLayout = Optional.empty();

    /** Raised when the controller's data doesn't look like the layout being read. */
    private Alert layoutAlert;

    private String layoutWarning = "";

    /** Dashboard keys for {@link #logBench()}, built once. */
    private BenchKeys benchKeys;

    /** Trigger for the A (cross) face button. */
    protected Trigger A = kFalse;

    /** Trigger for the B (circle) face button. */
    protected Trigger B = kFalse;

    /** Trigger for the X (square) face button. */
    protected Trigger X = kFalse;

    /** Trigger for the Y (triangle) face button. */
    protected Trigger Y = kFalse;

    /** Trigger for the left bumper (LB). */
    protected Trigger leftBumper = kFalse;

    /** Trigger for the right bumper (RB). */
    protected Trigger rightBumper = kFalse;

    /** Trigger active when the left analog trigger exceeds the configured deadzone threshold. */
    protected Trigger leftTrigger = kFalse;

    /** Trigger active when the right analog trigger exceeds the configured deadzone threshold. */
    protected Trigger rightTrigger = kFalse;

    /** Trigger for pressing the left analog stick (L3). */
    protected Trigger leftStickClick = kFalse;

    /** Trigger for pressing the right analog stick (R3). */
    protected Trigger rightStickClick = kFalse;

    /** Trigger for the Start / Menu button. */
    protected Trigger start = kFalse;

    /** Trigger for the Select / Back / View button. */
    protected Trigger select = kFalse;

    /** Trigger for D-pad up. */
    protected Trigger upDpad = kFalse;

    /** Trigger for D-pad down. */
    protected Trigger downDpad = kFalse;

    /** Trigger for D-pad left (including up-left and down-left diagonals). */
    protected Trigger leftDpad = kFalse;

    /** Trigger for D-pad right (including up-right and down-right diagonals). */
    protected Trigger rightDpad = kFalse;

    /** Trigger active when the left stick Y-axis exceeds the configured deadzone. */
    protected Trigger leftStickY = kFalse;

    /** Trigger active when the left stick X-axis exceeds the configured deadzone. */
    protected Trigger leftStickX = kFalse;

    /** Trigger active when the right stick Y-axis exceeds the configured deadzone. */
    protected Trigger rightStickY = kFalse;

    /** Trigger active when the right stick X-axis exceeds the configured deadzone. */
    protected Trigger rightStickX = kFalse;

    // Function bumper and trigger buttons

    /** Active when neither bumper is pressed. */
    public Trigger noBumpers = kFalse;

    /** Active when only the left bumper is pressed. */
    public Trigger leftBumperOnly;

    /** Active when only the right bumper is pressed. */
    public Trigger rightBumperOnly;

    /** Active when both bumpers are pressed simultaneously. */
    public Trigger bothBumpers = kFalse;

    /** Active when neither analog trigger is pressed. */
    public Trigger noTriggers = kFalse;

    /** Active when only the left trigger is pressed. */
    public Trigger leftTriggerOnly;

    /** Active when only the right trigger is pressed. */
    public Trigger rightTriggerOnly;

    /** Active when both analog triggers are pressed simultaneously. */
    public Trigger bothTriggers = kFalse;

    /** Active when no bumpers and no triggers are pressed (no modifier held). */
    public Trigger noModifiers = kFalse;

    /** Most recently computed left-stick direction; retained when the stick returns to center. */
    private Rotation2d storedLeftStickDirection = new Rotation2d();

    /** Most recently computed right-stick direction; retained when the stick returns to center. */
    private Rotation2d storedRightStickDirection = new Rotation2d();

    /**
     * {@code true} once the gamepad has been detected as connected and its triggers have been
     * configured.
     */
    private boolean configured =
            false; // Used to determine if we detected the gamepad is plugged and we have configured

    // it

    /** {@code true} after the "gamepad not connected" warning has been printed once. */
    private boolean printed = false; // Used to only print Gamepad Not Detected once

    /** Exponential response curve applied to both left-stick axes. */
    @Getter protected final ExpCurve leftStickCurve;

    /** Exponential response curve applied to both right-stick axes. */
    @Getter protected final ExpCurve rightStickCurve;

    /** Exponential response curve applied to both analog trigger axes. */
    @Getter protected final ExpCurve triggersCurve;

    /** Trigger active during the teleoperated period. */
    protected Trigger teleop = Util.teleop;

    /** Trigger active during the autonomous period. */
    protected Trigger autoMode = Util.autoMode;

    /** Trigger active during the test mode period. */
    protected Trigger testMode = Util.testMode;

    /** Trigger active while the robot is disabled. */
    protected Trigger disabled = Util.disabled;

    /**
     * Which Driver Station's controller layout to read (see {@link GamepadLayout}).
     *
     * <p>{@link #NI_DS} is the default because the 2026 FMS only takes the 2026 NI Driver Station.
     */
    public enum Mapping {
        /** The 2026 NI FRC Driver Station (the legacy XInput order, D-pad on the POV). */
        NI_DS,
        /** The 2027 WPILib Driver Station (the 2027 {@code Gamepad} order, D-pad as buttons). */
        WPILIB_DS,
        /**
         * Follow {@link GamepadLayout#detect(GenericHID)}, starting from {@link #NI_DS} and keeping
         * the last recognised layout. The detection is a heuristic; bench-test it before using
         * this.
         */
        AUTO
    }

    /**
     * Configuration for a {@link Gamepad} instance, defining the DriverStation USB port, axis curve
     * parameters, and whether the controller should be used on this robot.
     */
    public static class Config {
        /** Human-readable controller name used in alerts and telemetry. */
        @Getter private String name;

        /** USB port number as shown in the DriverStation application (0-indexed). */
        @Getter private int port; // USB port on the DriverStation app

        // A configured value to say if we should use this controller on this robot
        /**
         * Whether this controller should be used on the current robot; {@code false} disables it.
         */
        @Getter @Setter private boolean attached;

        /** Deadzone applied to both left-stick axes before the exponential curve. */
        @Getter @Setter double leftStickDeadzone = 0.001;

        /** Exponent for the left-stick exponential response curve (1.0 = linear). */
        @Getter @Setter double leftStickExp = 1.0;

        /** Output scalar applied after the left-stick exponential curve. */
        @Getter @Setter double leftStickScalar = 1.0;

        /** Deadzone applied to both right-stick axes before the exponential curve. */
        @Getter @Setter double rightStickDeadzone = 0.001;

        /** Exponent for the right-stick exponential response curve. */
        @Getter @Setter double rightStickExp = 1.0;

        /** Output scalar applied after the right-stick exponential curve. */
        @Getter @Setter double rightStickScalar = 1.0;

        /** Deadzone applied to both analog trigger axes before the exponential curve. */
        @Getter @Setter double triggersDeadzone = 0.002;

        /** Exponent for the analog-trigger exponential response curve. */
        @Getter @Setter double triggersExp = 1.0;

        /** Output scalar applied after the analog-trigger exponential curve. */
        @Getter @Setter double triggersScalar = 1.0;

        /**
         * Which Driver Station's controller layout to read. {@link Mapping#NI_DS}: the 2026 NI DS
         * is what the October 2026 event's FMS runs.
         */
        @Getter @Setter Mapping mapping = Mapping.NI_DS;

        /**
         * Creates a gamepad configuration for the given port.
         *
         * @param name human-readable controller name (used in alerts)
         * @param port DriverStation USB port number (0-indexed)
         */
        public Config(String name, int port) {
            this.name = name;
            this.port = port;
        }
    }

    private Config config;

    /**
     * Constructs a Gamepad object with the specified configuration.
     *
     * @param config the configuration object containing settings for the gamepad
     *     <p>The constructor initializes the following: - Superclass with port and attachment
     *     status from the configuration. - Curve objects for left stick, right stick, and triggers
     *     using exponential curves. - If the gamepad is attached, initializes the Xbox controller
     *     and its buttons, triggers, sticks, and D-pad.
     */
    protected Gamepad(Config config) {
        this.config = config;
        disconnectedAlert = new Alert(config.name + " Gamepad Disconnected", Alert.Level.HIGH);

        // Curve objects that we use to configure the controller axis objects
        leftStickCurve =
                new ExpCurve(
                        config.getLeftStickExp(),
                        0,
                        config.getLeftStickScalar(),
                        config.getLeftStickDeadzone());
        rightStickCurve =
                new ExpCurve(
                        config.getRightStickExp(),
                        0,
                        config.getRightStickScalar(),
                        config.getRightStickDeadzone());
        triggersCurve =
                new ExpCurve(
                        config.getTriggersExp(),
                        0,
                        config.getTriggersScalar(),
                        config.getTriggersDeadzone());

        layout =
                config.getMapping() == Mapping.WPILIB_DS
                        ? GamepadLayout.WPILIB_DS
                        : GamepadLayout.NI_DS;
        layoutAlert = new Alert(config.name + " controller layout", Alert.Level.MEDIUM);
        benchKeys = new BenchKeys("Gamepads/" + config.name + "/");

        if (config.attached) {
            hid = new GenericHID(config.port);
            // Each trigger reads the layout when it is polled, so AUTO can change it.
            A = new Trigger(() -> button(layout.a));
            B = new Trigger(() -> button(layout.b));
            X = new Trigger(() -> button(layout.x));
            Y = new Trigger(() -> button(layout.y));
            leftBumper = new Trigger(() -> button(layout.leftBumper));
            rightBumper = new Trigger(() -> button(layout.rightBumper));
            leftTrigger = new Trigger(() -> getLeftTriggerAxis() > config.triggersDeadzone);
            rightTrigger = new Trigger(() -> getRightTriggerAxis() > config.triggersDeadzone);
            leftStickClick = new Trigger(() -> button(layout.leftStick));
            rightStickClick = new Trigger(() -> button(layout.rightStick));
            start = new Trigger(() -> button(layout.start));
            select = new Trigger(() -> button(layout.back));
            // FM's 2026 D-pad: up and down are the pure directions, left and right include their
            // diagonals.
            upDpad = new Trigger(() -> getDpad() == POVDirection.UP);
            downDpad = new Trigger(() -> getDpad() == POVDirection.DOWN);
            leftDpad = new Trigger(() -> isLeft(getDpad()));
            rightDpad = new Trigger(() -> isRight(getDpad()));
            leftStickY = leftYTrigger(Threshold.ABS_GREATER, config.leftStickDeadzone);
            leftStickX = leftXTrigger(Threshold.ABS_GREATER, config.leftStickDeadzone);
            rightStickY = rightYTrigger(Threshold.ABS_GREATER, config.rightStickDeadzone);
            rightStickX = rightXTrigger(Threshold.ABS_GREATER, config.rightStickDeadzone);

            // Setup function bumper and trigger buttons
            noBumpers = rightBumper.negate().and(leftBumper.negate());
            leftBumperOnly = leftBumper.and(rightBumper.negate());
            rightBumperOnly = rightBumper.and(leftBumper.negate());
            bothBumpers = rightBumper.and(leftBumper);
            noTriggers = leftTrigger.negate().and(rightTrigger.negate());
            leftTriggerOnly = leftTrigger.and(rightTrigger.negate());
            rightTriggerOnly = rightTrigger.and(leftTrigger.negate());
            bothTriggers = leftTrigger.and(rightTrigger);
            noModifiers = noBumpers.and(noTriggers);
        }

        CommandScheduler.getInstance().registerSubsystem(this);
    }

    /** Runs the periodic update. */
    @Override
    public void periodic() {
        configure();
        if (config.isAttached()) {
            updateLayout();
            logBench();
        }
    }

    /**
     * Picks the layout for {@link Mapping#AUTO}, and raises {@link #layoutAlert} when the
     * controller's data looks like a different layout from the one being read.
     */
    private void updateLayout() {
        detectedLayout = isConnected() ? GamepadLayout.detect(hid) : Optional.empty();
        if (config.getMapping() == Mapping.AUTO && detectedLayout.isPresent()) {
            if (detectedLayout.get() != layout) {
                Telemetry.print(
                        "## " + getName() + ": reading the " + detectedLayout.get() + " layout ##");
            }
            layout = detectedLayout.get();
        }

        String warning = "";
        if (detectedLayout.isPresent() && detectedLayout.get() != layout) {
            warning =
                    config.name
                            + " controller looks like the "
                            + detectedLayout.get()
                            + " layout, but is read as "
                            + layout
                            + ": check Gamepad mapping";
        } else if (config.getMapping() == Mapping.AUTO
                && isConnected()
                && detectedLayout.isEmpty()) {
            warning = config.name + " controller layout not recognised, reading it as " + layout;
        }
        if (!warning.equals(layoutWarning)) {
            layoutWarning = warning;
            if (!warning.isEmpty()) {
                layoutAlert.setText(warning);
            }
        }
        layoutAlert.set(!warning.isEmpty());
    }

    private static final class BenchKeys {
        final String mapping, layout, detected, connected, name, pressed;
        final String rawButtons, rawButtonsAvailable, rawPov;
        final String[] rawAxes = new String[6];
        final String leftX, leftY, rightX, rightY, leftTrigger, rightTrigger;

        BenchKeys(String p) {
            mapping = p + "Mapping";
            layout = p + "Layout";
            detected = p + "DetectedLayout";
            connected = p + "Connected";
            name = p + "Name";
            pressed = p + "Pressed";
            rawButtons = p + "Raw/Buttons";
            rawButtonsAvailable = p + "Raw/ButtonsAvailable";
            rawPov = p + "Raw/POV";
            for (int i = 0; i < rawAxes.length; i++) {
                rawAxes[i] = p + "Raw/Axis" + i;
            }
            leftX = p + "LeftX";
            leftY = p + "LeftY";
            rightX = p + "RightX";
            rightY = p + "RightY";
            leftTrigger = p + "LeftTrigger";
            rightTrigger = p + "RightTrigger";
        }
    }

    /**
     * Logs the controller's raw data next to what the code reads from it, for the bench test in the
     * README: press each control and check {@code Pressed} names it.
     */
    private void logBench() {
        var k = benchKeys;
        Telemetry.logDash(k.mapping, config.getMapping().name());
        Telemetry.logDash(k.layout, layout.name());
        Telemetry.logDash(k.detected, detectedLayout.map(Enum::name).orElse("UNKNOWN"));
        boolean connected = isConnected();
        Telemetry.logDash(k.connected, connected);
        if (!connected) {
            return;
        }
        Telemetry.logDash(k.name, hid.getName());

        long available = hid.getButtonsAvailable();
        long pressedBits = 0;
        for (int i = 0; i < 64; i++) {
            if ((available & (1L << i)) != 0 && hid.getRawButton(i)) {
                pressedBits |= 1L << i;
            }
        }
        Telemetry.logDash(k.rawButtons, pressedBits);
        Telemetry.logDash(k.rawButtonsAvailable, available);
        Telemetry.logDash(
                k.rawPov, (hid.getPOVsAvailable() & 1) != 0 ? hid.getPOV(0).name() : "NONE");
        for (int i = 0; i < k.rawAxes.length; i++) {
            Telemetry.logDash(k.rawAxes[i], axis(i));
        }

        Telemetry.logDash(k.pressed, describePressed());
        Telemetry.logDash(k.leftX, getLeftX());
        Telemetry.logDash(k.leftY, getLeftY());
        Telemetry.logDash(k.rightX, getRightX());
        Telemetry.logDash(k.rightY, getRightY());
        Telemetry.logDash(k.leftTrigger, getLeftTriggerAxis());
        Telemetry.logDash(k.rightTrigger, getRightTriggerAxis());
    }

    /**
     * The controls the code reads as pressed, e.g. {@code "A LB DpadUp"}, or {@code ""}.
     *
     * @return space-separated control names
     */
    public String describePressed() {
        StringBuilder sb = new StringBuilder();
        appendIf(sb, A, "A");
        appendIf(sb, B, "B");
        appendIf(sb, X, "X");
        appendIf(sb, Y, "Y");
        appendIf(sb, leftBumper, "LB");
        appendIf(sb, rightBumper, "RB");
        appendIf(sb, leftTrigger, "LT");
        appendIf(sb, rightTrigger, "RT");
        appendIf(sb, select, "Back");
        appendIf(sb, start, "Start");
        appendIf(sb, leftStickClick, "LS");
        appendIf(sb, rightStickClick, "RS");
        appendIf(sb, upDpad, "DpadUp");
        appendIf(sb, downDpad, "DpadDown");
        appendIf(sb, leftDpad, "DpadLeft");
        appendIf(sb, rightDpad, "DpadRight");
        return sb.toString();
    }

    private static void appendIf(StringBuilder sb, Trigger t, String name) {
        if (t.getAsBoolean()) {
            if (!sb.isEmpty()) {
                sb.append(' ');
            }
            sb.append(name);
        }
    }

    /**
     * Reads a button by raw index, only if the controller has it (reading a missing one makes
     * WPILib print a warning every second).
     */
    private boolean button(int index) {
        if (index < 0 || !isConnected()) {
            return false;
        }
        return (hid.getButtonsAvailable() & (1L << index)) != 0 && hid.getRawButton(index);
    }

    /** Reads an axis by raw index, or 0 if the controller doesn't have it. */
    private double axis(int index) {
        if (!isConnected() || (hid.getAxesAvailable() & (1 << index)) == 0) {
            return 0.0;
        }
        return hid.getRawAxis(index);
    }

    /**
     * The D-pad direction, from POV 0 or from the four D-pad buttons, depending on the layout.
     *
     * @return the D-pad direction, {@link POVDirection#CENTER} if released or not connected
     */
    protected POVDirection getDpad() {
        if (!isConnected()) {
            return POVDirection.CENTER;
        }
        if (layout.dpadOnPov()) {
            return (hid.getPOVsAvailable() & 1) != 0 ? hid.getPOV(0) : POVDirection.CENTER;
        }
        boolean up = button(layout.dpadUp);
        boolean down = button(layout.dpadDown) && !up;
        boolean left = button(layout.dpadLeft);
        boolean right = button(layout.dpadRight) && !left;
        if (up) {
            return left ? POVDirection.UP_LEFT : right ? POVDirection.UP_RIGHT : POVDirection.UP;
        }
        if (down) {
            return left
                    ? POVDirection.DOWN_LEFT
                    : right ? POVDirection.DOWN_RIGHT : POVDirection.DOWN;
        }
        return left ? POVDirection.LEFT : right ? POVDirection.RIGHT : POVDirection.CENTER;
    }

    private static boolean isLeft(POVDirection d) {
        return d == POVDirection.LEFT || d == POVDirection.UP_LEFT || d == POVDirection.DOWN_LEFT;
    }

    private static boolean isRight(POVDirection d) {
        return d == POVDirection.RIGHT
                || d == POVDirection.UP_RIGHT
                || d == POVDirection.DOWN_RIGHT;
    }

    /**
     * Detects whether the gamepad has been connected since power-on and prints a one-time
     * confirmation message. Also raises a {@link Alert} whenever the controller is disconnected.
     * Called automatically by {@link #periodic()}.
     */
    public void configure() {
        if (config.isAttached()) {
            disconnectedAlert.set(!isConnected()); // Display if the controller is disconnected

            // Detect whether the Xbox controller has been plugged in after start-up
            if (!configured) {
                if (!isConnected()) {
                    if (!printed) {
                        Telemetry.print("##" + getName() + ": GAMEPAD NOT CONNECTED ##");
                        printed = true;
                    }
                    return;
                }

                configured = true;
                Telemetry.print("## " + getName() + ": gamepad is connected ##");
            }
        }
    }

    /**
     * Resets the controller configuration state so that the next {@link #configure()} call will
     * re-detect connection and re-apply button bindings. Should be paired with {@code
     * CommandScheduler.getInstance().clearButtons()}.
     */
    public void resetConfig() {
        configured = false;
        configure();
    }

    /**
     * Returns the current direction of the left stick as a {@link Rotation2d}. Zero points up
     * (toward positive Y), and 90° points to the left (toward negative X). The last non-zero
     * direction is retained when the stick is released.
     *
     * @return left-stick direction; zero-up / 90-left convention
     */
    public Rotation2d getLeftStickDirection() {
        double x = -1 * getLeftX();
        double y = -1 * getLeftY();
        if (x != 0 || y != 0) {
            Rotation2d angle = new Rotation2d(y, x);
            storedLeftStickDirection = angle;
        }
        return storedLeftStickDirection;
    }

    /**
     * Returns the current direction of the right stick as a {@link Rotation2d}. The last non-zero
     * direction is retained when the stick is released.
     *
     * @return right-stick direction
     */
    public Rotation2d getRightStickDirection() {
        double x = getRightX();
        double y = getRightY();
        if (x != 0 || y != 0) {
            Rotation2d angle = new Rotation2d(y, x);
            storedRightStickDirection = angle;
        }
        return storedRightStickDirection;
    }

    /**
     * Snaps the left-stick direction to the nearest cardinal angle (0, ±π/2, π radians).
     *
     * @return the snapped angle in radians
     */
    public double getLeftStickCardinals() {
        double stickAngle = getLeftStickDirection().getRadians();
        if (stickAngle > -Math.PI / 4 && stickAngle <= Math.PI / 4) {
            return 0;
        } else if (stickAngle > Math.PI / 4 && stickAngle <= 3 * Math.PI / 4) {
            return Math.PI / 2;
        } else if (stickAngle > 3 * Math.PI / 4 || stickAngle <= -3 * Math.PI / 4) {
            return Math.PI;
        } else {
            return -Math.PI / 2;
        }
    }

    /**
     * Snaps the right-stick direction to the nearest cardinal angle (0, ±π/2, π radians).
     *
     * @return the snapped angle in radians
     */
    public double getRightStickCardinals() {
        double stickAngle = getRightStickDirection().getRadians();
        if (stickAngle > -Math.PI / 4 && stickAngle <= Math.PI / 4) {
            return 0;
        } else if (stickAngle > Math.PI / 4 && stickAngle <= 3 * Math.PI / 4) {
            return Math.PI / 2;
        } else if (stickAngle > 3 * Math.PI / 4 || stickAngle <= -3 * Math.PI / 4) {
            return Math.PI;
        } else {
            return -Math.PI / 2;
        }
    }

    /**
     * Returns the Euclidean magnitude of the left stick deflection (0–√2 before curve, 0–1 after
     * typical scalar).
     *
     * @return left-stick vector magnitude
     */
    public double getLeftStickMagnitude() {
        double x = -1 * getLeftX();
        double y = -1 * getLeftY();
        return Math.sqrt(x * x + y * y);
    }

    /**
     * Returns the Euclidean magnitude of the right stick deflection.
     *
     * @return right-stick vector magnitude
     */
    public double getRightStickMagnitude() {
        double x = getRightX();
        double y = getRightY();
        return Math.sqrt(x * x + y * y);
    }

    /**
     * Get proper stick angles for each alliance
     *
     * @return
     */
    public double chooseCardinalDirections() {
        // hotfix
        if (AllianceSource.isBlue()) {
            return getRedAllianceStickCardinals();
        }
        return getBlueAllianceStickCardinals();
    }

    /**
     * Snaps the right stick to the nearest 45° increment using the Blue-alliance field orientation
     * (forward = 0 rad).
     *
     * @return the snapped heading in radians for the Blue alliance perspective
     */
    public double getBlueAllianceStickCardinals() {
        double stickAngle = getRightStickDirection().getRadians();
        if (stickAngle > -Math.PI / 8 && stickAngle <= Math.PI / 8) {
            return 0;
        } else if (stickAngle > Math.PI / 8 && stickAngle <= 3 * Math.PI / 8) {
            return Math.PI / 4;
        } else if (stickAngle > 3 * Math.PI / 8 && stickAngle <= 5 * Math.PI / 8) {
            return Math.PI / 2;
        } else if (stickAngle > 5 * Math.PI / 8 && stickAngle <= 7 * Math.PI / 8) {
            return 3 * Math.PI / 4;
        } // other half of circle
        else if (stickAngle < -Math.PI / 8 && stickAngle >= -3 * Math.PI / 8) {
            return -Math.PI / 4;
        } else if (stickAngle < -3 * Math.PI / 8 && stickAngle >= -5 * Math.PI / 8) {
            return -Math.PI / 2;
        } else if (stickAngle < -5 * Math.PI / 8 && stickAngle >= -7 * Math.PI / 8) {
            return -3 * Math.PI / 4;
        } else {
            return Math.PI; // greater than 7 * Math.PI / 8 or less than -7 * Math.PI / 8 (bottom of
            // circle)
        }
    }

    /**
     * Flips the stick direction for the red alliance.
     *
     * @return
     */
    public double getRedAllianceStickCardinals() {
        double stickAngle = getRightStickDirection().getRadians();

        if (stickAngle > -Math.PI / 8 && stickAngle <= Math.PI / 8) {
            return Math.PI;
        } else if (stickAngle > Math.PI / 8 && stickAngle <= 3 * Math.PI / 8) {
            return -3 * Math.PI / 4;
        } else if (stickAngle > 3 * Math.PI / 8 && stickAngle <= 5 * Math.PI / 8) {
            return -Math.PI / 2;
        } else if (stickAngle > 5 * Math.PI / 8 && stickAngle <= 7 * Math.PI / 8) {
            return -Math.PI / 4;
        } // other half of circle
        else if (stickAngle < -Math.PI / 8 && stickAngle >= -3 * Math.PI / 8) {
            return 3 * Math.PI / 4;
        } else if (stickAngle < -3 * Math.PI / 8 && stickAngle >= -5 * Math.PI / 8) {
            return Math.PI / 2;
        } else if (stickAngle < -5 * Math.PI / 8 && stickAngle >= -7 * Math.PI / 8) {
            return Math.PI / 4;
        } else {
            return 0; // greater than 7 * Math.PI / 8 or less than -7 * Math.PI / 8 (bottom of
            // circle)
        }
    }

    /**
     * Returns a {@link Trigger} that fires based on the left-stick Y axis and the given threshold
     * comparison.
     *
     * @param t the {@link Threshold} comparison type
     * @param threshold the value to compare against
     * @return trigger based on the left Y axis
     */
    public Trigger leftYTrigger(Threshold t, double threshold) {
        return axisTrigger(t, threshold, this::getLeftY);
    }

    /**
     * Returns a {@link Trigger} that fires based on the left-stick X axis and the given threshold
     * comparison.
     *
     * @param t the {@link Threshold} comparison type
     * @param threshold the value to compare against
     * @return trigger based on the left X axis
     */
    public Trigger leftXTrigger(Threshold t, double threshold) {
        return axisTrigger(t, threshold, this::getLeftX);
    }

    /**
     * Returns a {@link Trigger} that fires based on the right-stick Y axis and the given threshold
     * comparison.
     *
     * @param t the {@link Threshold} comparison type
     * @param threshold the value to compare against
     * @return trigger based on the right Y axis
     */
    public Trigger rightYTrigger(Threshold t, double threshold) {
        return axisTrigger(t, threshold, this::getRightY);
    }

    /**
     * Returns a {@link Trigger} that fires based on the right-stick X axis and the given threshold
     * comparison.
     *
     * @param t the {@link Threshold} comparison type
     * @param threshold the value to compare against
     * @return trigger based on the right X axis
     */
    public Trigger rightXTrigger(Threshold t, double threshold) {
        return axisTrigger(t, threshold, this::getRightX);
    }

    /**
     * Returns a {@link Trigger} that fires when either right-stick axis exceeds the given absolute
     * threshold.
     *
     * @param threshold minimum absolute axis value to activate the trigger
     * @return trigger active when the right stick is deflected beyond the threshold
     */
    public Trigger rightStick(double threshold) {
        return new Trigger(
                () -> Math.abs(getRightX()) >= threshold || Math.abs(getRightY()) >= threshold);
    }

    /**
     * Returns a {@link Trigger} that fires when either left-stick axis exceeds the given absolute
     * threshold.
     *
     * @param threshold minimum absolute axis value to activate the trigger
     * @return trigger active when the left stick is deflected beyond the threshold
     */
    public Trigger leftStick(double threshold) {
        return new Trigger(
                () -> Math.abs(getLeftX()) >= threshold || Math.abs(getLeftY()) >= threshold);
    }

    /** Axis trigger. */
    private Trigger axisTrigger(Threshold t, double threshold, DoubleSupplier v) {
        return new Trigger(
                () -> {
                    double value = v.getAsDouble();
                    switch (t) {
                        case GREATER:
                            return value > threshold;
                        case LESS:
                            return value < threshold;
                        case ABS_GREATER: // Also called Deadband
                            return Math.abs(value) > threshold;
                        default:
                            return false;
                    }
                });
    }

    /**
     * Comparison type used by axis-based {@link Trigger} factories such as {@link
     * #leftYTrigger(Threshold, double)}.
     */
    public enum Threshold {
        /** Fires when the axis value is strictly greater than the threshold. */
        GREATER,
        /** Fires when the axis value is strictly less than the threshold. */
        LESS,
        /** Fires when the absolute axis value is greater than the threshold (deadband check). */
        ABS_GREATER;
    }

    /**
     * Command that can be used to rumble the pilot controller. The intensity should be a value
     * between 0 and 1, where 0 is no rumble and 1 is full rumble. The duration of the rumble is
     * specified in seconds.
     *
     * @param leftIntensity the intensity of the left rumble motor (0 to 1)
     * @param rightIntensity the intensity of the right rumble motor (0 to 1)
     * @param durationSeconds the duration of the rumble in seconds
     * @return a Command object that can be used to rumble the controller with the specified
     *     intensities and duration
     */
    public Command rumbleCommand(
            double leftIntensity, double rightIntensity, double durationSeconds) {
        return Commands.sequence(
                        new InstantCommand(
                                () -> rumbleController(leftIntensity, rightIntensity), this),
                        Commands.waitSeconds(durationSeconds),
                        new InstantCommand(() -> rumbleController(0, 0), this))
                .ignoringDisable(true)
                .withName("Gamepad.Rumble");
    }

    /**
     * Overloaded method for rumbleCommand that allows for the same intensity on both rumble motors.
     * The duration of the rumble is specified in seconds. The intensity should be a value between 0
     * and 1, where 0 is no rumble and 1 is full rumble.
     *
     * @param intensity the intensity of the rumble (0 to 1)
     * @param durationSeconds the duration of the rumble in seconds
     * @return a Command object that can be used to rumble the controller with the specified
     *     intensity and duration
     */
    public Command rumbleCommand(double intensity, double durationSeconds) {
        return rumbleCommand(intensity, intensity, durationSeconds);
    }

    /**
     * Returns a new Command object that combines the given command with a rumble command. The
     * rumble command has a rumble strength of 1 and a duration of 0.5 seconds. The name of the
     * returned command is set to the name of the given command.
     *
     * @param command the command to be combined with the rumble command
     * @return a new Command object with rumble command
     */
    public Command rumbleCommand(Command command) {
        return command.alongWith(rumbleCommand(1, 0.5)).withName(command.getName());
    }

    /**
     * Returns whether the physical gamepad is currently connected to the DriverStation.
     *
     * @return {@code true} if the controller is attached and reports as connected
     */
    public boolean isConnected() {
        if (config.attached) {
            return this.getHID().isConnected();
        } else {
            return false;
        }
    }

    /**
     * Returns the raw right-trigger axis value (0–1), or {@code 0.0} if not connected.
     *
     * @return right-trigger axis value
     */
    protected double getRightTriggerAxis() {
        if (!isConnected()) {
            return 0.0;
        }
        return axis(layout.rightTrigger);
    }

    /**
     * Returns the raw left-trigger axis value (0–1), or {@code 0.0} if not connected.
     *
     * @return left-trigger axis value
     */
    protected double getLeftTriggerAxis() {
        if (!isConnected()) {
            return 0.0;
        }
        return axis(layout.leftTrigger);
    }

    /**
     * Returns the differential trigger value ({@code rightTrigger - leftTrigger}), useful as a
     * single "twist" axis for field-relative rotation commands.
     *
     * @return twist value in the range [-1, 1]
     */
    protected double getTwist() {
        double right = getRightTriggerAxis();
        double left = getLeftTriggerAxis();
        double value = right - left;
        return value;
    }

    /**
     * Returns the left-stick X axis value, or {@code 0.0} if not connected.
     *
     * @return left X axis value in the range [-1, 1]
     */
    protected double getLeftX() {
        if (!isConnected()) {
            return 0.0;
        }
        return axis(layout.leftX);
    }

    /**
     * Returns the left-stick Y axis value, or {@code 0.0} if not connected.
     *
     * @return left Y axis value in the range [-1, 1] (negative = up on most gamepads)
     */
    protected double getLeftY() {
        if (!isConnected()) {
            return 0.0;
        }
        return axis(layout.leftY);
    }

    /**
     * Returns the right-stick X axis value, or {@code 0.0} if not connected.
     *
     * @return right X axis value in the range [-1, 1]
     */
    protected double getRightX() {
        if (!isConnected()) {
            return 0.0;
        }
        return axis(layout.rightX);
    }

    /**
     * Returns the right-stick Y axis value, or {@code 0.0} if not connected.
     *
     * @return right Y axis value in the range [-1, 1] (negative = up on most gamepads)
     */
    protected double getRightY() {
        if (!isConnected()) {
            return 0.0;
        }
        return axis(layout.rightY);
    }

    /**
     * Returns the underlying {@link GenericHID} for low-level access, or {@code null} if not
     * attached.
     *
     * @return the raw HID device, or {@code null}
     */
    protected GenericHID getHID() {
        if (!config.attached) {
            return null;
        }
        return hid;
    }

    /**
     * Returns the underlying {@link GenericHID} for rumble output, or {@code null} if not
     * connected.
     *
     * @return the raw HID device (only when connected), or {@code null}
     */
    protected GenericHID getRumbleHID() {
        if (!isConnected()) {
            return null;
        }
        return hid;
    }

    /**
     * Immediately sets the left and right rumble motor intensities. Use {@link
     * #rumbleCommand(double, double, double)} for timed rumble sequences.
     *
     * @param leftIntensity left rumble motor intensity (0–1)
     * @param rightIntensity right rumble motor intensity (0–1)
     */
    public void rumbleController(double leftIntensity, double rightIntensity) {
        if (!isConnected()) {
            return;
        }
        getRumbleHID().setRumble(RumbleType.LEFT_RUMBLE, leftIntensity);
        getRumbleHID().setRumble(RumbleType.RIGHT_RUMBLE, rightIntensity);
    }
}
