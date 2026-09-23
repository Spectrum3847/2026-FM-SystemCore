package frc.robot;

import org.wpilib.driverstation.Gamepad;
import org.wpilib.hardware.hal.AllianceStationID;
import org.wpilib.hardware.hal.RobotMode;
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

    /** Whether the script was requested. */
    public static boolean requested() {
        return "drive".equalsIgnoreCase(System.getenv("FM_SIM_SCRIPT"));
    }

    private static GamepadSim pilot;
    private static double start = Double.NaN;
    private static boolean questReset = false;

    /** One step: {@code [until seconds, leftY, leftX, rightX]}; stick up is negative Y. */
    private static final double[][] STEPS = {
        {6.0, 0, 0, 0}, // disabled: Limelights seed and confirm
        {9.0, -0.6, 0, 0}, // forward
        {11.0, 0, 0, 0.8}, // spin in place (spin gate should reject frames)
        {14.0, 0, -0.6, 0}, // strafe
        {17.0, 0.6, 0, 0}, // back
        {20.0, -0.4, 0.4, 0.3}, // arc
        {23.0, 0, 0, 0}, // stop: stationary tiers
        {26.0, -0.5, 0, 0},
        {28.0, 0, 0, 0},
    };

    /** Advances the script. Call from {@code simulationPeriodic}. */
    public static void periodic() {
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
            var hub = frc.rebuilt.Field.getBlueHubCenter();
            Robot.getSwerve()
                    .resetPose(
                            new org.wpilib.math.geometry.Pose2d(
                                    hub.getX() - 2.5,
                                    hub.getY(),
                                    org.wpilib.math.geometry.Rotation2d.k180deg));
        }
        double t = now - start;
        if (!questReset && t > 4.0) {
            // What the operator's LB+X does: put the QuestNav's frame on the robot's pose.
            questReset = true;
            Robot.getVision().resetQuestToRobotPose();
        }
        double[] step = STEPS[STEPS.length - 1];
        for (double[] s : STEPS) {
            if (t < s[0]) {
                step = s;
                break;
            }
        }
        boolean enabled = t >= STEPS[0][0] && t < STEPS[STEPS.length - 1][0];
        DriverStationSim.setEnabled(enabled);
        pilot.setAxis(Gamepad.Axis.LEFT_Y, step[1]);
        pilot.setAxis(Gamepad.Axis.LEFT_X, step[2]);
        pilot.setAxis(Gamepad.Axis.RIGHT_X, step[3]);
        pilot.notifyNewData();
        DriverStationSim.notifyNewData();
    }
}
