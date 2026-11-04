package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.CurrentSuperState;
import frc.robot.subsystems.SuperStructure.WantedSuperState;
import frc.robot.subsystems.fuelIntake.FuelIntake;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.indexerBed.IndexerBed;
import frc.robot.subsystems.indexerTower.IndexerTower;
import frc.robot.subsystems.intakeExtension.IntakeExtension;
import frc.robot.subsystems.launcher.Launcher;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import org.wpilib.hardware.hal.RobotMode;

/**
 * Drives the booted robot's SuperStructure through a match's worth of wanted states and checks the
 * state it lands in and what it asks of each mechanism. A sequence, not one state at a time: the
 * squeeze timer and the disabled idle depend on what came before.
 */
@Tag("sim")
class SuperStructureSequenceTest {
    /** Loops per second at FM's loop period. */
    private static final int LOOPS_PER_SECOND = (int) Math.round(1 / Constants.LOOP_PERIOD_SECONDS);

    @BeforeAll
    static void boot() throws Exception {
        SimRobot.boot();
    }

    @AfterAll
    static void cleanUp() {
        SimRobot.cleanUp();
    }

    private static SuperStructure ss() {
        return Robot.getSuperStructure();
    }

    private static void want(WantedSuperState state) throws Exception {
        ss().setWantedSuperState(state);
        SimRobot.step(2);
    }

    @Test
    void aTeleopCycleAndBackToDisabled() throws Exception {
        SimRobot.setMode(RobotMode.TELEOPERATED, true);

        want(WantedSuperState.IDLE);
        assertEquals(CurrentSuperState.IDLE, ss().getCurrentSuperState());
        assertEquals(Launcher.WantedState.IDLE_PREP, Robot.getLauncher().getWantedState());
        assertEquals(
                IntakeExtension.WantedState.STOPPED, Robot.getIntakeExtension().getWantedState());

        want(WantedSuperState.INTAKE_FUEL);
        assertEquals(CurrentSuperState.INTAKE_FUEL, ss().getCurrentSuperState());
        assertEquals(FuelIntake.WantedState.INTAKE, Robot.getFuelIntake().getWantedState());
        assertEquals(IndexerBed.WantedState.SLOW_INDEX, Robot.getIndexerBed().getWantedState());
        assertEquals(
                IntakeExtension.WantedState.FULL_EXTEND,
                Robot.getIntakeExtension().getWantedState());

        // Launching keeps the intake out for a second, then squeezes the fuel in.
        want(WantedSuperState.LAUNCH_WITH_SQUEEZE);
        assertEquals(CurrentSuperState.LAUNCH_WITH_SQUEEZE, ss().getCurrentSuperState());
        assertEquals(Launcher.WantedState.AIM_AT_TARGET, Robot.getLauncher().getWantedState());
        assertEquals(Hood.WantedState.AIM_AT_TARGET, Robot.getHood().getWantedState());
        assertEquals(
                IntakeExtension.WantedState.FULL_EXTEND,
                Robot.getIntakeExtension().getWantedState());
        SimRobot.step(LOOPS_PER_SECOND + LOOPS_PER_SECOND / 10);
        assertEquals(
                IntakeExtension.WantedState.SLOW_CLOSE,
                Robot.getIntakeExtension().getWantedState(),
                "squeezes after 1 s");

        // Leaving and launching again starts the squeeze timer over.
        want(WantedSuperState.INTAKE_FUEL);
        want(WantedSuperState.LAUNCH_WITH_SQUEEZE);
        assertEquals(
                IntakeExtension.WantedState.FULL_EXTEND,
                Robot.getIntakeExtension().getWantedState(),
                "a new launch extends again before squeezing");

        want(WantedSuperState.UNJAM);
        assertEquals(CurrentSuperState.UNJAM, ss().getCurrentSuperState());
        assertEquals(IndexerTower.WantedState.UNJAM, Robot.getIndexerTower().getWantedState());
        assertEquals(IndexerBed.WantedState.UNJAM, Robot.getIndexerBed().getWantedState());
        assertEquals(Launcher.WantedState.OFF, Robot.getLauncher().getWantedState());

        want(WantedSuperState.EJECT);
        assertEquals(FuelIntake.WantedState.OUTTAKE, Robot.getFuelIntake().getWantedState());

        // Disabled, idle is the auto idle: nothing runs.
        want(WantedSuperState.IDLE);
        SimRobot.setMode(RobotMode.TELEOPERATED, false);
        assertEquals(CurrentSuperState.AUTON_IDLE, ss().getCurrentSuperState());
        assertEquals(FuelIntake.WantedState.NEUTRAL, Robot.getFuelIntake().getWantedState());
        assertEquals(IndexerTower.WantedState.OFF, Robot.getIndexerTower().getWantedState());
        assertEquals(
                IntakeExtension.WantedState.STOPPED, Robot.getIntakeExtension().getWantedState());
    }
}
