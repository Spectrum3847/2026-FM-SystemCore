package frc.spectrumLib.gamepads;

import java.util.Optional;
import org.wpilib.driverstation.GenericHID;
import org.wpilib.simulation.GenericHIDSim;

/**
 * Where an Xbox controller's buttons and axes arrive, which depends on the Driver Station that
 * sends them, not on the robot controller.
 *
 * <p>The SystemCore HAL copies the joystick arrays it gets from MrcCommDaemon without remapping
 * them (allwpilib v2027.0.0-alpha-6 {@code hal/src/main/native/systemcore/FIRSTDriverStation.cpp}),
 * and WPILib ships separate {@code NiDs*} controller classes because "the old DS doesn't have the
 * same mappings as a new DS" (allwpilib PR #8376). So:
 *
 * <ul>
 *   <li>{@link #NI_DS}: the 2026 NI FRC Driver Station, the one the 2026 FMS supports. The legacy
 *       XInput order: A B X Y LB RB Back Start LS RS = buttons 0-9; axes LX LY LT RT RX RY = 0-5;
 *       the D-pad is POV 0. The same as WPILib alpha-6's {@code NiDsXboxController}. The sim GUI's
 *       "Map gamepad" also produces this order for an Xbox controller on Windows.
 *   <li>{@link #WPILIB_DS}: the 2027 WPILib Driver Station (SDL gamepad order). Buttons south east
 *       west north back guide start LS RS LB RB = 0-10, the D-pad as buttons 11-14; axes LX LY RX
 *       RY LT RT = 0-5. Taken from WPILib's {@link org.wpilib.driverstation.Gamepad}.
 * </ul>
 */
public enum GamepadLayout {
    // Literal indices: the NiDs* classes are due to be removed from WPILib before 2027 kickoff
    // (allwpilib #9406). GamepadLayoutTest checks these against alpha-6's NiDsXboxController.
    NI_DS(
            0, // A
            1, // B
            2, // X
            3, // Y
            4, // left bumper
            5, // right bumper
            6, // back
            7, // start
            8, // left stick
            9, // right stick
            -1, // D-pad: POV 0 (NO_BUTTON)
            -1, -1, -1, 0, // left X
            1, // left Y
            4, // right X
            5, // right Y
            2, // left trigger
            3), // right trigger
    WPILIB_DS(
            org.wpilib.driverstation.Gamepad.Button.SOUTH_FACE.value,
            org.wpilib.driverstation.Gamepad.Button.EAST_FACE.value,
            org.wpilib.driverstation.Gamepad.Button.WEST_FACE.value,
            org.wpilib.driverstation.Gamepad.Button.NORTH_FACE.value,
            org.wpilib.driverstation.Gamepad.Button.LEFT_BUMPER.value,
            org.wpilib.driverstation.Gamepad.Button.RIGHT_BUMPER.value,
            org.wpilib.driverstation.Gamepad.Button.BACK.value,
            org.wpilib.driverstation.Gamepad.Button.START.value,
            org.wpilib.driverstation.Gamepad.Button.LEFT_STICK.value,
            org.wpilib.driverstation.Gamepad.Button.RIGHT_STICK.value,
            org.wpilib.driverstation.Gamepad.Button.DPAD_UP.value,
            org.wpilib.driverstation.Gamepad.Button.DPAD_DOWN.value,
            org.wpilib.driverstation.Gamepad.Button.DPAD_LEFT.value,
            org.wpilib.driverstation.Gamepad.Button.DPAD_RIGHT.value,
            org.wpilib.driverstation.Gamepad.Axis.LEFT_X.value,
            org.wpilib.driverstation.Gamepad.Axis.LEFT_Y.value,
            org.wpilib.driverstation.Gamepad.Axis.RIGHT_X.value,
            org.wpilib.driverstation.Gamepad.Axis.RIGHT_Y.value,
            org.wpilib.driverstation.Gamepad.Axis.LEFT_TRIGGER.value,
            org.wpilib.driverstation.Gamepad.Axis.RIGHT_TRIGGER.value);

    /** A D-pad button index for a layout whose D-pad is POV 0 instead. */
    public static final int NO_BUTTON = -1;

    public final int a, b, x, y;
    public final int leftBumper, rightBumper, back, start, leftStick, rightStick;

    /** D-pad buttons, or {@link #NO_BUTTON} when the D-pad is POV 0 ({@link #dpadOnPov()}). */
    public final int dpadUp, dpadDown, dpadLeft, dpadRight;

    public final int leftX, leftY, rightX, rightY, leftTrigger, rightTrigger;

    GamepadLayout(
            int a,
            int b,
            int x,
            int y,
            int leftBumper,
            int rightBumper,
            int back,
            int start,
            int leftStick,
            int rightStick,
            int dpadUp,
            int dpadDown,
            int dpadLeft,
            int dpadRight,
            int leftX,
            int leftY,
            int rightX,
            int rightY,
            int leftTrigger,
            int rightTrigger) {
        this.a = a;
        this.b = b;
        this.x = x;
        this.y = y;
        this.leftBumper = leftBumper;
        this.rightBumper = rightBumper;
        this.back = back;
        this.start = start;
        this.leftStick = leftStick;
        this.rightStick = rightStick;
        this.dpadUp = dpadUp;
        this.dpadDown = dpadDown;
        this.dpadLeft = dpadLeft;
        this.dpadRight = dpadRight;
        this.leftX = leftX;
        this.leftY = leftY;
        this.rightX = rightX;
        this.rightY = rightY;
        this.leftTrigger = leftTrigger;
        this.rightTrigger = rightTrigger;
    }

    /** Whether the D-pad arrives as POV 0 rather than as four buttons. */
    public boolean dpadOnPov() {
        return dpadUp == NO_BUTTON;
    }

    /**
     * Guesses which Driver Station layout a connected controller's data is in, from what the DS
     * says the controller has (logged by AdvantageKit, so this replays). Empty when it can't tell.
     *
     * <p>This is inferred, not documented: the 2027 layout carries the D-pad as buttons 11-14, so a
     * controller with button 14 is taken as {@link #WPILIB_DS}; an NI DS Xbox controller has 10
     * buttons and one POV, so a controller with a POV and no button 14 is taken as {@link #NI_DS}.
     * Anything else (a controller that is not an Xbox pad, nothing plugged in) is unknown. Check it
     * with the bench test in the README before trusting {@code AUTO}.
     *
     * @param hid the controller
     * @return the layout its data looks like, if it is recognisable
     */
    public static Optional<GamepadLayout> detect(GenericHID hid) {
        return detect(hid.getButtonsAvailable(), hid.getPOVsAvailable(), hid.getAxesAvailable());
    }

    /**
     * {@link #detect(GenericHID)} on the availability bitmasks.
     *
     * @param buttonsAvailable bit {@code i} set if button {@code i} is present
     * @param povsAvailable bit {@code i} set if POV {@code i} is present
     * @param axesAvailable bit {@code i} set if axis {@code i} is present
     * @return the layout the data looks like, if it is recognisable
     */
    public static Optional<GamepadLayout> detect(
            long buttonsAvailable, int povsAvailable, int axesAvailable) {
        boolean sixAxes = (axesAvailable & 0x3F) == 0x3F;
        if (!sixAxes) {
            return Optional.empty();
        }
        if ((buttonsAvailable & (1L << WPILIB_DS.dpadRight)) != 0) {
            return Optional.of(WPILIB_DS);
        }
        if ((povsAvailable & 1) != 0) {
            return Optional.of(NI_DS);
        }
        return Optional.empty();
    }

    /**
     * Makes a simulated joystick look like an Xbox controller as this Driver Station reports it:
     * the same buttons, axes and POV, so {@link #detect(GenericHID)} recognises it. For the
     * scripted sim and the tests.
     *
     * @param sim the simulated joystick
     */
    public void simulateController(GenericHIDSim sim) {
        if (this == NI_DS) {
            sim.setName("Controller (Xbox One For Windows)");
            sim.setButtonsMaximumIndex(10);
            sim.setPOVsMaximumIndex(1);
        } else {
            sim.setName("Xbox Series X Controller");
            sim.setButtonsMaximumIndex(26);
            sim.setPOVsMaximumIndex(0);
        }
        sim.setAxesMaximumIndex(6);
        sim.setGamepadType(GenericHID.HIDType.STANDARD);
    }
}
