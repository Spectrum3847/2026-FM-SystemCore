package frc.robot;

import frc.rebuilt.Field;
import frc.rebuilt.ShotCalculator;
import frc.spectrumLib.gamepads.GamepadLayout;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.telemetry.Telemetry;
import java.util.List;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.wpilib.driverstation.MatchType;
import org.wpilib.hardware.hal.AllianceStationID;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.simulation.GenericHIDSim;
import org.wpilib.system.Timer;

/**
 * A scripted drive for the desktop simulation, so the localization testbed can be exercised (and
 * checked) without a driver: sit disabled so the Limelights seed the pose, then drive a lap of
 * straights, strafes and spins in teleop while every pose source is logged.
 *
 * <p>Opt-in only: set the environment variable {@code FM_SIM_SCRIPT} before {@code ./gradlew run}.
 * With it unset the sim behaves normally and takes a real Driver Station.
 *
 * <ul>
 *   <li>{@code drive}: the localization lap.
 *   <li>{@code shoot}: aiming and launching, standing and on the move, then each set shot from its
 *       spot, with fuel put in the hopper before each launch. For the aim feedforward, the shot
 *       gate, the set shots and the shot log.
 *   <li>{@code chords}: the pilot's LB + face-button chords pressed and let go in sloppy orders,
 *       standing still. For {@link frc.spectrumLib.gamepads.ModifierChords}: read {@code
 *       SuperStructure/WantedSuperState}.
 *   <li>{@code auto:<chooser name>}: one PathPlanner auto.
 * </ul>
 *
 * <p>The program closes the log and exits once the script is done, so a scripted run can be the
 * whole of a command line; {@code FM_SIM_EXIT=0} keeps it running. With {@code
 * FM_SIM_REBOOT=Hood@9,IntakeExtension@12}, those mechanisms' motors reboot at those script
 * seconds, as in a brownout. With {@code FM_SIM_MATCH=Q12} (or {@code P3}, {@code E5}), the Driver
 * Station reports that FMS match, so the end of teleop writes the match summary file.
 */
public final class SimScript {
    private SimScript() {}

    /** Whether the script runs: requested in simulation, and always in replay. */
    public static boolean requested() {
        String script = System.getenv("FM_SIM_SCRIPT");
        return Constants.currentMode == Constants.Mode.REPLAY
                || "drive".equalsIgnoreCase(script)
                || "shoot".equalsIgnoreCase(script)
                || "chords".equalsIgnoreCase(script)
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

    private static boolean chordsScript() {
        return "chords".equalsIgnoreCase(System.getenv("FM_SIM_SCRIPT"));
    }

    /** Seconds the auto script runs autonomous for, after 6 s disabled. */
    private static final double AUTO_SECONDS = 20.0;

    private static GenericHIDSim pilot;

    /**
     * The layout the pilot's controller is read in, so the script's stick moves land on the axes
     * the code reads (the NI DS order by default, see {@link frc.spectrumLib.gamepads.Gamepad}).
     */
    private static GamepadLayout pilotLayout;

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

    /**
     * The chord run, standing still, same columns as {@link #SHOOT_STEPS}. The 0.03 and 0.05 s
     * steps are a few loops: a face button that leads or trails LB by less than the settle time.
     * The expected wanted states are in the comments.
     */
    private static final double[][] CHORD_STEPS = {
        {6.0, 0, 0, 0, 0, 0, 0, 0, 0}, // disabled: seed
        {7.0, 0, 0, 0, 0, 0, 1, 0, 0}, // LB alone: nothing
        {8.5, 0, 0, 0, 0, 1, 1, 0, 0}, // LB+X: left trench set shot
        {10.0, 0, 0, 0, 0, 1, 0, 0, 0}, // LB let go first, X held: IDLE, then TRACK_TARGET
        {10.5, 0, 0, 0, 0, 0, 0, 0, 0}, // IDLE
        {10.53, 0, 0, 0, 0, 0, 0, 1, 0}, // A a few loops before LB: no UNJAM
        {12.0, 0, 0, 0, 0, 0, 1, 1, 0}, // LB+A: tower set shot
        {12.05, 0, 0, 0, 0, 0, 0, 1, 0}, // LB let go a few loops before A: IDLE, no UNJAM
        {12.5, 0, 0, 0, 0, 0, 0, 0, 0},
        {14.0, 0, 0, 0, 0, 0, 0, 1, 0}, // A: UNJAM
        {15.0, 0, 0, 0, 0, 1, 0, 1, 0}, // A+X: still UNJAM (A beats X)
        {16.0, 0, 0, 0, 0, 1, 0, 0, 0}, // A let go, X held: TRACK_TARGET at once
        {16.5, 0, 0, 0, 0, 0, 0, 0, 0}, // IDLE
        {18.0, 0, 0, 0, 0, 0, 1, 0, 1}, // LB+B: right trench set shot
        {18.5, 0, 0, 0, 0, 0, 0, 0, 0}, // IDLE
        {18.55, 0, 0, 0, 0, 0, 0, 1, 0}, // A tapped for 0.05 s: UNJAM for 0.05 s, after the settle
        {19.5, 0, 0, 0, 0, 0, 0, 0, 0}, // IDLE
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

    /**
     * Loops in a row the swerve's CTRE signals must be fresh before the script enables. On a busy
     * CI runner Phoenix's simulated devices sometimes report stale positions and yaw for a moment,
     * and enabling into that jumped the pose by up to 0.5 m.
     */
    private static final int FRESH_LOOPS_TO_ENABLE = 25;

    /** Longest the script holds the enable for fresh signals before enabling anyway. */
    private static final double MAX_ENABLE_HOLD_SECONDS = 5.0;

    private static int freshLoops = 0;
    private static int lastFailedDaqs = -1;
    private static double lastLoopTime = Double.NaN;
    private static boolean enableReleased = false;
    private static double enableHoldStart = Double.NaN;

    /** Metres from the field edge inside which the script stops translating (sim has no walls). */
    private static final double EDGE_MARGIN_METERS = 1.0;

    /**
     * The script's two non-driver actions, as an AdvantageKit input so a replay of a scripted run
     * performs them on the same loop (the driving itself is already replayed, through the logged
     * Driver Station).
     */
    public static final class Actions implements LoggableInputs {
        public boolean placeRobot = false;
        public Pose2d placePose = Pose2d.ZERO;

        @Override
        public void toLog(LogTable table) {
            // Sim script: the robot was placed at PlacePose this loop.
            table.put("PlaceRobot", placeRobot);
            // Sim script: where it was placed.
            table.put("PlacePose", placePose);
        }

        @Override
        public void fromLog(LogTable table) {
            placeRobot = table.get("PlaceRobot", placeRobot);
            placePose = table.get("PlacePose", placePose);
        }
    }

    private static final Actions actions = new Actions();

    /**
     * Advances the script. Call from {@code simulationPeriodic}, in simulation (script requested)
     * and in replay (so a replayed scripted run repeats the script's actions).
     */
    public static void periodic() {
        actions.placeRobot = false;
        if (Constants.currentMode == Constants.Mode.SIM) {
            drive();
        }
        Logger.processInputs("SimScript", actions);
        if (actions.placeRobot) {
            Robot.getSwerve().resetPose(actions.placePose);
        }
    }

    private static void drive() {
        if (requestedAuto() != null) {
            runAuto(requestedAuto());
            return;
        }
        double[][] steps = shootScript() ? SHOOT_STEPS : chordsScript() ? CHORD_STEPS : DRIVE_STEPS;
        double now = Timer.getTimestamp();
        if (Double.isNaN(start)) {
            start = now;
            pilot = new GenericHIDSim(0);
            pilotLayout = Robot.getPilot().getLayout();
            pilotLayout.simulateController(pilot);
            DriverStationSim.setJoystickIsGamepad(0, true);
            DriverStationSim.setJoystickName(0, "SimScript Pilot (" + pilotLayout + ")");
            DriverStationSim.setDsAttached(true);
            DriverStationSim.setAllianceStationId(AllianceStationID.BLUE_1);
            DriverStationSim.setRobotMode(RobotMode.TELEOPERATED);
            DriverStationSim.setEnabled(false);
            fakeMatch();
            // Back to the blue hub, 2.5 m out: the back Limelight looks straight at a hub face,
            // enough tags for the disabled seed to confirm.
            var hub = Field.getBlueHubCenter();
            actions.placeRobot = true;
            actions.placePose = new Pose2d(hub.getX() - 2.5, hub.getY(), Rotation2d.k180deg);
        }
        double t = holdForFreshSignals(now, steps[0][UNTIL]);
        rebootMotors(t);
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
        pilot.setRawAxis(pilotLayout.leftY, step[LEFT_Y]);
        pilot.setRawAxis(pilotLayout.leftX, step[LEFT_X]);
        pilot.setRawAxis(pilotLayout.rightX, step[RIGHT_X]);
        pilot.setRawAxis(pilotLayout.rightTrigger, column(step, RT));
        pilot.setRawButton(pilotLayout.x, column(step, X) > 0.5);
        pilot.setRawButton(pilotLayout.leftBumper, column(step, LB) > 0.5);
        pilot.setRawButton(pilotLayout.a, column(step, A) > 0.5);
        pilot.setRawButton(pilotLayout.b, column(step, B) > 0.5);
        pilot.notifyNewData();
        // Known limitation: a simulated DS change made mid-loop reaches AdvantageKit's HAL snapshot
        // one loop before WPILib's Driver Station cache, so replaying a *scripted* run enables one
        // loop early. Refreshing the cache here does not help. Real Driver Station packets are
        // picked up at the top of a loop and are not affected.
        DriverStationSim.notifyNewData();
        exitWhenDone(t, steps[steps.length - 1][UNTIL]);
    }

    /**
     * Script seconds, held just short of {@code enableAt} until the swerve's CTRE signals have been
     * fresh for {@link #FRESH_LOOPS_TO_ENABLE} loops: no failed odometry DAQ, every module and the
     * Pigeon connected, odometry samples arriving and the loop on time. Gives up after {@link
     * #MAX_ENABLE_HOLD_SECONDS} and enables anyway, so a sim that never settles still runs.
     */
    private static double holdForFreshSignals(double now, double enableAt) {
        var in = Robot.getSwerve().getInputs();
        boolean daqOk = lastFailedDaqs >= 0 && in.failedDaqs == lastFailedDaqs;
        lastFailedDaqs = in.failedDaqs;
        boolean onTime =
                !Double.isNaN(lastLoopTime)
                        && now - lastLoopTime <= 1.25 * Constants.LOOP_PERIOD_SECONDS;
        lastLoopTime = now;
        boolean modulesOk = in.moduleConnected.length > 0;
        for (boolean ok : in.moduleConnected) {
            modulesOk &= ok;
        }
        boolean fresh =
                daqOk
                        && onTime
                        && modulesOk
                        && in.gyroConnected
                        && in.odometryTimestamps.length > 0;
        freshLoops = fresh ? freshLoops + 1 : 0;

        double t = now - start;
        if (enableReleased || t < enableAt) {
            return t;
        }
        if (Double.isNaN(enableHoldStart)) {
            enableHoldStart = now;
        }
        if (freshLoops >= FRESH_LOOPS_TO_ENABLE) {
            enableReleased = true;
        } else if (now - enableHoldStart >= MAX_ENABLE_HOLD_SECONDS) {
            enableReleased = true;
            Telemetry.print(
                    "SimScript: CTRE signals still not fresh after "
                            + MAX_ENABLE_HOLD_SECONDS
                            + " s; enabling anyway");
        } else {
            // Slide the script's clock so it resumes from here once the signals settle.
            start = now - enableAt + 1e-6;
            return enableAt - 1e-6;
        }
        if (now > enableHoldStart) {
            Telemetry.print(
                    String.format(
                            "SimScript: held the enable %.2f s for fresh CTRE signals",
                            now - enableHoldStart));
        }
        return t;
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
            fakeMatch();
            // Where a dashboard picks an option on AdvantageKit's LoggedNetworkChooser.
            org.wpilib.networktables.NetworkTableInstance.getDefault()
                    .getEntry("/SmartDashboard/Auto Chooser/selected/tune")
                    .setString(autoName);
        }
        double t = holdForFreshSignals(now, 6.0);
        rebootMotors(t);
        DriverStationSim.setEnabled(t >= 6.0 && t < 6.0 + AUTO_SECONDS);
        // Known limitation: a simulated DS change made mid-loop reaches AdvantageKit's HAL snapshot
        // one loop before WPILib's Driver Station cache, so replaying a *scripted* run enables one
        // loop early. Refreshing the cache here does not help. Real Driver Station packets are
        // picked up at the top of a loop and are not affected.
        DriverStationSim.notifyNewData();
        exitWhenDone(t, 6.0 + AUTO_SECONDS);
    }

    /**
     * Match type and number from {@code FM_SIM_MATCH} (e.g. {@code Q12}), as the FMS would send.
     * Leaves the FMS-attached flag alone, so nothing else changes behavior.
     */
    private static void fakeMatch() {
        String spec = System.getenv("FM_SIM_MATCH");
        if (spec == null || spec.length() < 2) {
            return;
        }
        MatchType type =
                switch (Character.toUpperCase(spec.charAt(0))) {
                    case 'P' -> MatchType.PRACTICE;
                    case 'Q' -> MatchType.QUALIFICATION;
                    case 'E' -> MatchType.ELIMINATION;
                    default -> MatchType.NONE;
                };
        DriverStationSim.setEventName("SIM");
        DriverStationSim.setMatchType(type);
        DriverStationSim.setMatchNumber(Integer.parseInt(spec.substring(1)));
    }

    private static double lastRebootCheck = -1;

    /** Reboots each motor named in {@code FM_SIM_REBOOT} on the loop its time passes. */
    private static void rebootMotors(double t) {
        String spec = System.getenv("FM_SIM_REBOOT");
        if (spec == null || spec.isBlank()) {
            return;
        }
        List<Mechanism> mechanisms = List.of(Robot.getHood(), Robot.getIntakeExtension());
        for (String entry : spec.split(",")) {
            String[] parts = entry.trim().split("@");
            double at = Double.parseDouble(parts[1]);
            if (lastRebootCheck < at && t >= at) {
                for (Mechanism m : mechanisms) {
                    if (m.getName().equals(parts[0])) {
                        m.simulateReboot();
                    }
                }
            }
        }
        lastRebootCheck = t;
    }

    /** Closes the log and exits a second after the script ends, unless {@code FM_SIM_EXIT=0}. */
    private static void exitWhenDone(double t, double endSeconds) {
        if (t > endSeconds + 1.0 && !"0".equals(System.getenv("FM_SIM_EXIT"))) {
            Logger.end();
            System.exit(0);
        }
    }
}
