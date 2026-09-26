package frc.robot;

import frc.rebuilt.Field;
import frc.rebuilt.ShotCalculator;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.wpilib.driverstation.Gamepad;
import org.wpilib.hardware.hal.AllianceStationID;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.simulation.GamepadSim;
import org.wpilib.system.Timer;

/**
 * A scripted drive for the desktop simulation, so the localization testbed can be exercised (and
 * checked) without a driver: sit disabled so the Limelights seed the pose, then drive a lap of
 * straights, strafes and spins in teleop while every pose source is logged.
 *
 * <p>Opt-in only: set the environment variable {@code FM_SIM_SCRIPT} before {@code ./gradlew
 * simulateJava}. With it unset the sim behaves normally and takes a real Driver Station.
 *
 * <ul>
 *   <li>{@code drive}: the localization lap.
 *   <li>{@code shoot}: aiming and launching, standing and on the move, then each set shot from its
 *       spot, with fuel put in the hopper before each launch. For the aim feedforward, the shot
 *       gate, the set shots and the shot log.
 *   <li>{@code auto:<chooser name>}: one PathPlanner auto.
 * </ul>
 *
 * <p>With {@code FM_SIM_EXIT=1} as well, the program closes the log and exits once the script is
 * done, so a scripted run can be the whole of a command line.
 */
public final class SimScript {
    private SimScript() {}

    /** Whether the script runs: requested in simulation, and always in replay. */
    public static boolean requested() {
        String script = System.getenv("FM_SIM_SCRIPT");
        return Constants.currentMode == Constants.Mode.REPLAY
                || "drive".equalsIgnoreCase(script)
                || "shoot".equalsIgnoreCase(script)
                || (script != null && script.startsWith("auto:"));
    }

    /**
     * The auto to run for {@code FM_SIM_SCRIPT=auto:<chooser name>}, e.g. {@code auto:TBTB Left},
     * or null for the teleop drive.
     */
    private static String requestedAuto() {
        String script = System.getenv("FM_SIM_SCRIPT");
        return script != null && script.startsWith("auto:") ? script.substring(5) : null;
    }

    private static boolean shootScript() {
        return "shoot".equalsIgnoreCase(System.getenv("FM_SIM_SCRIPT"));
    }

    /** Seconds the auto script runs autonomous for, after 6 s disabled. */
    private static final double AUTO_SECONDS = 20.0;

    private static GamepadSim pilot;
    private static double start = Double.NaN;

    // Step columns. Stick up is negative Y; the buttons are 1 for held.
    private static final int UNTIL = 0;
    private static final int LEFT_Y = 1;
    private static final int LEFT_X = 2;
    private static final int RIGHT_X = 3;
    private static final int RT = 4;
    private static final int X = 5;
    private static final int LB = 6;
    private static final int A = 7;
    private static final int B = 8;

    /** The localization lap: {@code [until seconds, leftY, leftX, rightX]}. */
    private static final double[][] DRIVE_STEPS = {
        {6.0, 0, 0, 0}, // disabled: Limelights seed and confirm
        {9.0, -0.35, 0, 0}, // forward
        {11.0, 0, 0, 0.8}, // spin in place (the spin gate should reject frames)
        {13.5, 0, -0.35, 0}, // strafe left
        {16.0, 0.35, 0, 0}, // back
        {19.0, -0.3, 0.3, 0.3}, // arc
        {22.0, 0, 0, 0}, // stop: stationary tiers
        {24.5, 0, 0.35, 0}, // strafe right
        {27.0, -0.3, 0, 0.2},
        {29.0, 0, 0, 0},
    };

    /**
     * The shooting run, from 2.5 m in front of the blue hub with the launcher facing it: {@code
     * [until, leftY, leftX, rightX, RT, X, LB, A, B]}. Strafing at 0.35 stick is about 1 m/s, a
     * bearing rate of about 0.4 rad/s at this range: enough that aiming on the heading PID alone
     * lags visibly and the feedforward has something to do. Every button change has an empty step
     * before it, because releasing X and pressing RT on the same loop lands in IDLE (X's release
     * binding runs after RT's press binding).
     */
    private static final double[][] SHOOT_STEPS = {
        {6.0, 0, 0, 0, 0, 0, 0, 0, 0}, // disabled: seed
        {8.0, 0, 0, 0, 0, 1, 0, 0, 0}, // X: track target standing (spin up, aim)
        {8.3, 0, 0, 0, 0, 0, 0, 0, 0},
        {10.5, 0, 0, 0, 1, 0, 0, 0, 0}, // RT: launch standing, from a spun-down flywheel
        {11.0, 0, 0, 0, 0, 0, 0, 0, 0},
        {12.5, 0, -0.35, 0, 0, 1, 0, 0, 0}, // X + strafe left: aim on the move
        {15.5, 0, 0.35, 0, 0, 1, 0, 0, 0}, // X + strafe right
        {17.0, 0, -0.35, 0, 0, 1, 0, 0, 0}, // X + strafe left, back to the middle
        {17.5, 0, 0, 0, 0, 0, 0, 0, 0},
        {20.0, 0, -0.35, 0, 1, 0, 0, 0, 0}, // RT + strafe: launch on the move (slowed to 10%)
        {21.0, 0, 0, 0, 0, 0, 0, 0, 0}, // placed on the tower spot
        {24.0, 0, 0, 0, 0, 0, 1, 1, 0}, // LB+A: tower set shot
        {24.5, 0, 0, 0, 0, 0, 0, 0, 0}, // placed on the left trench spot
        {27.5, 0, 0, 0, 0, 1, 1, 0, 0}, // LB+X: left trench set shot
        {28.0, 0, 0, 0, 0, 0, 0, 0, 0}, // placed on the right trench spot
        {31.0, 0, 0, 0, 0, 0, 1, 0, 1}, // LB+B: right trench set shot
        {32.0, 0, 0, 0, 0, 0, 0, 0, 0},
    };

    /** Loads the hopper at the start of these steps (index into {@link #SHOOT_STEPS}). */
    private static final int[] SHOOT_REFILL_STEPS = {3, 9, 11, 13, 15};

    /**
     * Puts the robot on a set-shot spot, at its heading, at the start of these steps: {@code {step,
     * SetShot ordinal}}.
     */
    private static final int[][] SHOOT_PLACE_STEPS = {
        {10, ShotCalculator.SetShot.TOWER.ordinal()},
        {12, ShotCalculator.SetShot.LEFT_TRENCH.ordinal()},
        {14, ShotCalculator.SetShot.RIGHT_TRENCH.ordinal()},
    };

    /** Fuel per refill: the sim launches it four lanes at a time. */
    private static final int REFILL_FUEL = 8;

    private static int lastStep = -1;

    /** Metres from the field edge inside which the script stops translating (sim has no walls). */
    private static final double EDGE_MARGIN_METERS = 1.0;

    /**
     * The script's two non-driver actions, as an AdvantageKit input so a replay of a scripted run
     * performs them on the same loop (the driving itself is already replayed, through the logged
     * Driver Station).
     */
    public static final class Actions implements LoggableInputs {
        public boolean placeRobot = false;
        public Pose2d placePose = Pose2d.kZero;
        public boolean resetQuest = false;

        @Override
        public void toLog(LogTable table) {
            table.put("PlaceRobot", placeRobot);
            table.put("PlacePose", placePose);
            table.put("ResetQuest", resetQuest);
        }

        @Override
        public void fromLog(LogTable table) {
            placeRobot = table.get("PlaceRobot", placeRobot);
            placePose = table.get("PlacePose", placePose);
            resetQuest = table.get("ResetQuest", resetQuest);
        }
    }

    private static final Actions actions = new Actions();
    private static boolean questResetDone = false;

    /**
     * Advances the script. Call from {@code simulationPeriodic}, in simulation (script requested)
     * and in replay (so a replayed scripted run repeats the script's actions).
     */
    public static void periodic() {
        actions.placeRobot = false;
        actions.resetQuest = false;
        if (Constants.currentMode == Constants.Mode.SIM) {
            drive();
        }
        Logger.processInputs("SimScript", actions);
        if (actions.placeRobot) {
            Robot.getSwerve().resetPose(actions.placePose);
        }
        if (actions.resetQuest) {
            Robot.getVision().resetQuestToRobotPose();
        }
    }

    private static void drive() {
        if (requestedAuto() != null) {
            runAuto(requestedAuto());
            return;
        }
        double[][] steps = shootScript() ? SHOOT_STEPS : DRIVE_STEPS;
        double now = Timer.getTimestamp();
        if (Double.isNaN(start)) {
            start = now;
            pilot = new GamepadSim(0);
            DriverStationSim.setJoystickIsGamepad(0, true);
            DriverStationSim.setJoystickName(0, "SimScript Pilot");
            DriverStationSim.setJoystickAxesAvailable(0, 0x3F); // bitmask: axes 0-5
            DriverStationSim.setJoystickButtonsAvailable(0, 0xFFFFFFFFL);
            DriverStationSim.setDsAttached(true);
            DriverStationSim.setAllianceStationId(AllianceStationID.BLUE_1);
            DriverStationSim.setRobotMode(RobotMode.TELEOPERATED);
            DriverStationSim.setEnabled(false);
            // Back to the blue hub, 2.5 m out: the back Limelight looks straight at a hub face,
            // enough tags for the disabled seed to confirm.
            var hub = Field.getBlueHubCenter();
            actions.placeRobot = true;
            actions.placePose = new Pose2d(hub.getX() - 2.5, hub.getY(), Rotation2d.k180deg);
        }
        double t = now - start;
        if (!questResetDone && t > 4.0) {
            // What the operator's LB+X does: put the QuestNav's frame on the robot's pose.
            questResetDone = true;
            actions.resetQuest = true;
        }
        int index = steps.length - 1;
        for (int i = 0; i < steps.length; i++) {
            if (t < steps[i][UNTIL]) {
                index = i;
                break;
            }
        }
        double[] step = steps[index].clone();
        if (index != lastStep) {
            lastStep = index;
            onStepStart(index);
        }
        boolean enabled = t >= steps[0][UNTIL] && t < steps[steps.length - 1][UNTIL];
        var truth = Robot.getSwerve().getSimTruthPose();
        if (truth.isPresent()) {
            var p = truth.get();
            boolean nearEdge =
                    p.getX() < EDGE_MARGIN_METERS
                            || p.getY() < EDGE_MARGIN_METERS
                            || p.getX() > Field.fieldLength - EDGE_MARGIN_METERS
                            || p.getY() > Field.fieldWidth - EDGE_MARGIN_METERS;
            if (nearEdge) {
                step[LEFT_Y] = 0;
                step[LEFT_X] = 0;
            }
        }
        DriverStationSim.setEnabled(enabled);
        pilot.setAxis(Gamepad.Axis.LEFT_Y, step[LEFT_Y]);
        pilot.setAxis(Gamepad.Axis.LEFT_X, step[LEFT_X]);
        pilot.setAxis(Gamepad.Axis.RIGHT_X, step[RIGHT_X]);
        pilot.setAxis(Gamepad.Axis.RIGHT_TRIGGER, column(step, RT));
        pilot.setButton(Gamepad.Button.WEST_FACE, column(step, X) > 0.5);
        pilot.setButton(Gamepad.Button.LEFT_BUMPER, column(step, LB) > 0.5);
        pilot.setButton(Gamepad.Button.SOUTH_FACE, column(step, A) > 0.5);
        pilot.setButton(Gamepad.Button.EAST_FACE, column(step, B) > 0.5);
        pilot.notifyNewData();
        // Known limitation: a simulated DS change made mid-loop reaches AdvantageKit's HAL snapshot
        // one loop before WPILib's Driver Station cache, so replaying a *scripted* run enables one
        // loop early. Refreshing the cache here does not help. Real Driver Station packets are
        // picked up at the top of a loop and are not affected.
        DriverStationSim.notifyNewData();
        exitWhenDone(t, steps[steps.length - 1][UNTIL]);
    }

    private static double column(double[] step, int column) {
        return column < step.length ? step[column] : 0;
    }

    /** Things that happen once, on the loop a step begins. */
    private static void onStepStart(int index) {
        if (!shootScript() || Robot.getRobotSim() == null) {
            return;
        }
        for (int refill : SHOOT_REFILL_STEPS) {
            if (refill == index) {
                Robot.getRobotSim().getBallSim().setHopperCount(REFILL_FUEL);
            }
        }
        for (int[] place : SHOOT_PLACE_STEPS) {
            if (place[0] == index) {
                actions.placeRobot = true;
                actions.placePose = ShotCalculator.SetShot.values()[place[1]].pose();
            }
        }
    }

    /**
     * Picks an auto on the chooser, sits disabled while the robot places itself on the auto's start
     * and the cameras seed, then runs autonomous for {@link #AUTO_SECONDS}.
     */
    private static void runAuto(String autoName) {
        double now = Timer.getTimestamp();
        if (Double.isNaN(start)) {
            start = now;
            DriverStationSim.setDsAttached(true);
            DriverStationSim.setAllianceStationId(AllianceStationID.BLUE_1);
            DriverStationSim.setRobotMode(RobotMode.AUTONOMOUS);
            DriverStationSim.setEnabled(false);
            org.wpilib.networktables.NetworkTableInstance.getDefault()
                    .getTable("SmartDashboard")
                    .getSubTable("Auto Chooser")
                    .getEntry("selected")
                    .setString(autoName);
        }
        double t = now - start;
        DriverStationSim.setEnabled(t >= 6.0 && t < 6.0 + AUTO_SECONDS);
        // Known limitation: a simulated DS change made mid-loop reaches AdvantageKit's HAL snapshot
        // one loop before WPILib's Driver Station cache, so replaying a *scripted* run enables one
        // loop early. Refreshing the cache here does not help. Real Driver Station packets are
        // picked up at the top of a loop and are not affected.
        DriverStationSim.notifyNewData();
        exitWhenDone(t, 6.0 + AUTO_SECONDS);
    }

    /** With {@code FM_SIM_EXIT=1}, closes the log and exits a second after the script ends. */
    private static void exitWhenDone(double t, double endSeconds) {
        if (t > endSeconds + 1.0 && "1".equals(System.getenv("FM_SIM_EXIT"))) {
            Logger.end();
            System.exit(0);
        }
    }
}
