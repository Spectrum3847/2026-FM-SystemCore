package frc.spectrumLib.gamepads;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Optional;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.driverstation.NiDsXboxController;
import org.wpilib.driverstation.POVDirection;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.simulation.GenericHIDSim;

/**
 * The layout tables against WPILib's own classes, and the {@link Gamepad} wrapper reading a
 * simulated controller in each layout: what the bench test in the README checks on hardware.
 */
class GamepadLayoutTest {
    private static final int PORT = 3;

    /** A bare wrapper, as Pilot and Operator are. */
    private static final class TestGamepad extends Gamepad {
        TestGamepad(Config config) {
            super(config);
        }

        double leftX() {
            return getLeftX();
        }

        double leftY() {
            return getLeftY();
        }

        double rightX() {
            return getRightX();
        }

        double rightY() {
            return getRightY();
        }

        double leftTriggerAxis() {
            return getLeftTriggerAxis();
        }

        double rightTriggerAxis() {
            return getRightTriggerAxis();
        }
    }

    private GenericHIDSim sim;

    @BeforeAll
    static void initHal() {
        assertTrue(HAL.initialize(500, 0));
    }

    @BeforeEach
    void setUp() {
        DriverStationSim.resetData();
        DriverStationSim.setSendError(false);
        sim = new GenericHIDSim(PORT);
    }

    @AfterEach
    void tearDown() {
        CommandScheduler.getInstance().unregisterAllSubsystems();
    }

    private static Gamepad.Config config(Gamepad.Mapping mapping) {
        var c = new Gamepad.Config("Test", PORT);
        c.setAttached(true);
        c.setMapping(mapping);
        return c;
    }

    @Test
    void niDsTableMatchesWpilibNiDsXboxController() {
        var l = GamepadLayout.NI_DS;
        assertEquals(NiDsXboxController.Button.kA.value, l.a);
        assertEquals(NiDsXboxController.Button.kB.value, l.b);
        assertEquals(NiDsXboxController.Button.kX.value, l.x);
        assertEquals(NiDsXboxController.Button.kY.value, l.y);
        assertEquals(NiDsXboxController.Button.kLeftBumper.value, l.leftBumper);
        assertEquals(NiDsXboxController.Button.kRightBumper.value, l.rightBumper);
        assertEquals(NiDsXboxController.Button.kBack.value, l.back);
        assertEquals(NiDsXboxController.Button.kStart.value, l.start);
        assertEquals(NiDsXboxController.Button.kLeftStick.value, l.leftStick);
        assertEquals(NiDsXboxController.Button.kRightStick.value, l.rightStick);
        assertEquals(NiDsXboxController.Axis.kLeftX.value, l.leftX);
        assertEquals(NiDsXboxController.Axis.kLeftY.value, l.leftY);
        assertEquals(NiDsXboxController.Axis.kRightX.value, l.rightX);
        assertEquals(NiDsXboxController.Axis.kRightY.value, l.rightY);
        assertEquals(NiDsXboxController.Axis.kLeftTrigger.value, l.leftTrigger);
        assertEquals(NiDsXboxController.Axis.kRightTrigger.value, l.rightTrigger);
        assertTrue(l.dpadOnPov());
        assertFalse(GamepadLayout.WPILIB_DS.dpadOnPov());
    }

    @Test
    void detectsEachLayoutFromWhatTheControllerHas() {
        // NI DS Xbox pad: 10 buttons, 6 axes, 1 POV.
        assertEquals(Optional.of(GamepadLayout.NI_DS), GamepadLayout.detect(0x3FF, 0x1, 0x3F));
        // Sim GUI, Xbox pad on Windows: 10 buttons + the hat as buttons 10-13, and the hat.
        assertEquals(Optional.of(GamepadLayout.NI_DS), GamepadLayout.detect(0x3FFF, 0x1, 0x3F));
        // 2027 DS (SDL): the D-pad as buttons 11-14, no POV.
        assertEquals(
                Optional.of(GamepadLayout.WPILIB_DS), GamepadLayout.detect(0x3FFFFFF, 0x0, 0x3F));
        assertEquals(Optional.empty(), GamepadLayout.detect(0, 0, 0)); // unplugged
        assertEquals(Optional.empty(), GamepadLayout.detect(0xFFF, 0x1, 0x0F)); // a joystick
        assertEquals(Optional.empty(), GamepadLayout.detect(0x3FF, 0x0, 0x3F)); // no POV
    }

    @Test
    void niDsControllerReadsTheRightControls() {
        var pad = new TestGamepad(config(Gamepad.Mapping.NI_DS));
        GamepadLayout.NI_DS.simulateController(sim);
        // Each control at its NI DS index; none of these is where the 2027 Gamepad class looks.
        sim.setRawButton(4, true); // LB (2027: back)
        sim.setRawButton(6, true); // Back (2027: start)
        sim.setRawButton(9, true); // RS (2027: left bumper)
        sim.setRawAxis(4, 0.7); // right X (2027: left trigger)
        sim.setRawAxis(3, 0.9); // right trigger (2027: right Y)
        sim.setRawAxis(1, -0.5); // left Y
        sim.setPOV(POVDirection.UP_LEFT);
        sim.notifyNewData();
        pad.periodic();

        assertEquals(GamepadLayout.NI_DS, pad.getLayout());
        assertEquals("LB RT Back RS DpadLeft", pad.describePressed());
        assertEquals(0.7, pad.rightX(), 1e-2);
        assertEquals(0.0, pad.rightY(), 1e-9);
        assertEquals(0.9, pad.rightTriggerAxis(), 1e-2);
        assertEquals(0.0, pad.leftTriggerAxis(), 1e-9);
        assertEquals(-0.5, pad.leftY(), 1e-2);
        assertEquals(0.0, pad.leftX(), 1e-9);

        sim.setRawButton(4, false);
        sim.setRawButton(6, false);
        sim.setRawButton(9, false);
        sim.setRawAxis(3, 0.0);
        sim.setRawButton(0, true);
        sim.setRawButton(5, true);
        sim.setRawButton(7, true);
        sim.setPOV(POVDirection.UP);
        sim.notifyNewData();
        assertEquals("A RB Start DpadUp", pad.describePressed());
    }

    @Test
    void wpilibDsControllerReadsTheRightControls() {
        var pad = new TestGamepad(config(Gamepad.Mapping.WPILIB_DS));
        GamepadLayout.WPILIB_DS.simulateController(sim);
        sim.setRawButton(9, true); // LB
        sim.setRawButton(4, true); // Back
        sim.setRawButton(14, true); // D-pad right
        sim.setRawButton(12, true); // D-pad down
        sim.setRawAxis(2, 0.7); // right X
        sim.setRawAxis(4, 0.6); // left trigger
        sim.notifyNewData();
        pad.periodic();

        assertEquals("LB LT Back DpadRight", pad.describePressed());
        assertEquals(0.7, pad.rightX(), 1e-2);
        assertEquals(0.6, pad.leftTriggerAxis(), 1e-2);
        assertEquals(0.0, pad.rightTriggerAxis(), 1e-9);
    }

    @Test
    void autoFollowsTheController() {
        var pad = new TestGamepad(config(Gamepad.Mapping.AUTO));
        assertEquals(GamepadLayout.NI_DS, pad.getLayout()); // before anything is plugged in

        GamepadLayout.WPILIB_DS.simulateController(sim);
        sim.setRawAxis(2, 0.4);
        sim.notifyNewData();
        pad.periodic();
        assertEquals(GamepadLayout.WPILIB_DS, pad.getLayout());
        assertEquals(0.4, pad.rightX(), 1e-2);

        DriverStationSim.resetData(); // unplugged: keeps the last layout
        sim.notifyNewData();
        pad.periodic();
        assertEquals(GamepadLayout.WPILIB_DS, pad.getLayout());

        GamepadLayout.NI_DS.simulateController(sim);
        sim.setRawAxis(4, -0.3);
        sim.notifyNewData();
        pad.periodic();
        assertEquals(GamepadLayout.NI_DS, pad.getLayout());
        assertEquals(-0.3, pad.rightX(), 1e-2);
    }

    @Test
    void fixedMappingKeepsItsLayoutWhateverIsPluggedIn() {
        var pad = new TestGamepad(config(Gamepad.Mapping.NI_DS));
        GamepadLayout.WPILIB_DS.simulateController(sim);
        sim.notifyNewData();
        pad.periodic();
        assertEquals(GamepadLayout.NI_DS, pad.getLayout());
    }
}
