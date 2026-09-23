package frc.robot;

import frc.rebuilt.Field;
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
 * <p>Opt-in only: set the environment variable {@code FM_SIM_SCRIPT=drive} before {@code ./gradlew
 * simulateJava}. With it unset the sim behaves normally and takes a real Driver Station.
 */
public final class SimScript {
    private SimScript() {}

    /** Whether the script runs: requested in simulation, and always in replay. */
    public static boolean requested() {
        String script = System.getenv("FM_SIM_SCRIPT");
        return Constants.currentMode == Constants.Mode.REPLAY
                || "drive".equalsIgnoreCase(script)
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

    /** Seconds the auto script runs autonomous for, after 6 s disabled. */
    private static final double AUTO_SECONDS = 20.0;

    private static GamepadSim pilot;
    private static double start = Double.NaN;

    /** One step: {@code [until seconds, leftY, leftX, rightX]}; stick up is negative Y. */
    private static final double[][] STEPS = {
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
        double[] step = STEPS[STEPS.length - 1];
        for (double[] s : STEPS) {
            if (t < s[0]) {
                step = s;
                break;
            }
        }
        boolean enabled = t >= STEPS[0][0] && t < STEPS[STEPS.length - 1][0];
        var truth = Robot.getSwerve().getSimTruthPose();
        if (truth.isPresent()) {
            var p = truth.get();
            boolean nearEdge =
                    p.getX() < EDGE_MARGIN_METERS
                            || p.getY() < EDGE_MARGIN_METERS
                            || p.getX() > Field.fieldLength - EDGE_MARGIN_METERS
                            || p.getY() > Field.fieldWidth - EDGE_MARGIN_METERS;
            if (nearEdge) {
                step = new double[] {step[0], 0, 0, step[3]};
            }
        }
        DriverStationSim.setEnabled(enabled);
        pilot.setAxis(Gamepad.Axis.LEFT_Y, step[1]);
        pilot.setAxis(Gamepad.Axis.LEFT_X, step[2]);
        pilot.setAxis(Gamepad.Axis.RIGHT_X, step[3]);
        pilot.notifyNewData();
        DriverStationSim.notifyNewData();
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
        DriverStationSim.notifyNewData();
    }
}
