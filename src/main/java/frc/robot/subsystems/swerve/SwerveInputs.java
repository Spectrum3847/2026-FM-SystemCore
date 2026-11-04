package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveModuleVelocity;

/**
 * Everything the drivetrain reads from CTRE in one loop, as an AdvantageKit input.
 *
 * <p>The CTRE {@code SwerveDrivetrain} still runs a 250 Hz odometry thread and the swerve requests.
 * None of the robot's logic reads its pose directly. Instead, every odometry sample it produced
 * since the last loop is queued off its thread and recorded here, and {@link
 * frc.spectrumLib.localization.PoseFusion} integrates them on the main loop. That is what makes the
 * robot pose replayable: in replay these arrays come back from the log, the fusion re-runs over
 * them, and every vision gate sees the same pose it saw on the field.
 *
 * <p>Odometry samples are stored flattened: module {@code m} of sample {@code s} is at index {@code
 * s * 4 + m}.
 */
public class SwerveInputs implements LoggableInputs {
    /** Sample times, {@code Timer.getTimestamp()} base. */
    public double[] odometryTimestamps = new double[0];

    /** Gyro yaw per sample, radians. */
    public double[] odometryYawRadians = new double[0];

    /** Module drive distance per sample, metres, flattened. */
    public double[] odometryModuleDistances = new double[0];

    /** Module steer angle per sample, radians, flattened. */
    public double[] odometryModuleAngles = new double[0];

    /** CTRE's own pose estimate, for comparison only. */
    public Pose2d ctrePose = Pose2d.ZERO;

    /** Chassis velocity CTRE measured from the modules, robot-relative. */
    public ChassisVelocities robotVelocity = new ChassisVelocities();

    /** Each module's measured speed and angle, front-left, front-right, back-left, back-right. */
    public SwerveModuleVelocity[] moduleVelocities = new SwerveModuleVelocity[0];

    /** Each module's commanded speed and angle, same order. */
    public SwerveModuleVelocity[] moduleTargets = new SwerveModuleVelocity[0];

    /** Per module: drive and steer motors both answering this loop. */
    public boolean[] moduleConnected = new boolean[0];

    /**
     * Per module: Phoenix's raw reset flag for the drive motor, which is also true on the first
     * contact after boot. Logic reads {@code Swerve.moduleRebootedThisLoop} instead.
     */
    public boolean[] moduleResetOccurred = new boolean[0];

    /** The Pigeon answered this loop. */
    public boolean gyroConnected = false;

    /** Phoenix's raw Pigeon reset flag; logic reads {@code Swerve.gyroRebootedThisLoop}. */
    public boolean gyroResetOccurred = false;

    /** Pigeon yaw rate, counterclockwise positive. */
    public double gyroYawRateRadPerSec = 0;

    /** Pigeon pitch: the robot tipping forward or back. */
    public double gyroPitchDegrees = 0;

    /** Pigeon roll: the robot tipping sideways. */
    public double gyroRollDegrees = 0;

    /** Pigeon pitch rate. */
    public double gyroPitchRateDegPerSec = 0;

    /** Pigeon roll rate. */
    public double gyroRollRateDegPerSec = 0;

    /** Pigeon acceleration along its own axes, g (gravity included). All 0 when not read. */
    public double accelXG = 0;

    /** Pigeon acceleration, y axis. */
    public double accelYG = 0;

    /** Pigeon acceleration, z axis; about 1 g sitting flat. */
    public double accelZG = 0;

    /** SystemCore's onboard IMU, the backup gyro. Not on CAN, so a CANivore drop spares it. */
    public boolean onboardImuPresent = false;

    /** Onboard IMU yaw. */
    public double onboardImuYawRadians = 0;

    /** Onboard IMU yaw rate. */
    public double onboardImuYawRateRadPerSec = 0;

    /** Raw, m/s^2: WPILib does not apply the mount orientation to the accelerometer. */
    public double onboardImuAccelX = 0;

    /** Onboard IMU acceleration, y. */
    public double onboardImuAccelY = 0;

    /** Onboard IMU acceleration, z. */
    public double onboardImuAccelZ = 0;

    /** Average time between CTRE odometry samples; 0.004 at 250 Hz. */
    public double odometryPeriodSeconds = 0;

    /** CTRE odometry thread reads that got every signal, since boot. */
    public int successfulDaqs = 0;

    /**
     * CTRE odometry thread reads that timed out or missed a signal, since boot; rising means CAN
     * trouble.
     */
    public int failedDaqs = 0;

    @Override
    public void toLog(LogTable table) {
        table.put("OdometryTimestamps", odometryTimestamps);
        table.put("OdometryYawRadians", odometryYawRadians);
        table.put("OdometryModuleDistances", odometryModuleDistances);
        table.put("OdometryModuleAngles", odometryModuleAngles);
        table.put("CtrePose", ctrePose);
        table.put("RobotVelocity", robotVelocity);
        table.put("ModuleVelocities", moduleVelocities);
        table.put("ModuleTargets", moduleTargets);
        table.put("ModuleConnected", moduleConnected);
        table.put("ModuleResetOccurred", moduleResetOccurred);
        table.put("GyroConnected", gyroConnected);
        table.put("GyroResetOccurred", gyroResetOccurred);
        table.put("GyroYawRateRadPerSec", gyroYawRateRadPerSec);
        table.put("GyroPitchDegrees", gyroPitchDegrees, "degrees");
        table.put("GyroRollDegrees", gyroRollDegrees, "degrees");
        table.put("GyroPitchRateDegPerSec", gyroPitchRateDegPerSec, "degrees per second");
        table.put("GyroRollRateDegPerSec", gyroRollRateDegPerSec, "degrees per second");
        table.put("AccelXG", accelXG);
        table.put("AccelYG", accelYG);
        table.put("AccelZG", accelZG);
        table.put("OnboardImu/Present", onboardImuPresent);
        table.put("OnboardImu/YawRadians", onboardImuYawRadians, "radians");
        table.put("OnboardImu/YawRateRadPerSec", onboardImuYawRateRadPerSec);
        table.put("OnboardImu/AccelX", onboardImuAccelX);
        table.put("OnboardImu/AccelY", onboardImuAccelY);
        table.put("OnboardImu/AccelZ", onboardImuAccelZ);
        table.put("OdometryPeriodSeconds", odometryPeriodSeconds, "seconds");
        table.put("SuccessfulDaqs", successfulDaqs);
        table.put("FailedDaqs", failedDaqs);
    }

    @Override
    public void fromLog(LogTable table) {
        odometryTimestamps = table.get("OdometryTimestamps", odometryTimestamps);
        odometryYawRadians = table.get("OdometryYawRadians", odometryYawRadians);
        odometryModuleDistances = table.get("OdometryModuleDistances", odometryModuleDistances);
        odometryModuleAngles = table.get("OdometryModuleAngles", odometryModuleAngles);
        ctrePose = table.get("CtrePose", ctrePose);
        robotVelocity = table.get("RobotVelocity", robotVelocity);
        moduleVelocities = table.get("ModuleVelocities", moduleVelocities);
        moduleTargets = table.get("ModuleTargets", moduleTargets);
        moduleConnected = table.get("ModuleConnected", moduleConnected);
        moduleResetOccurred = table.get("ModuleResetOccurred", moduleResetOccurred);
        gyroConnected = table.get("GyroConnected", gyroConnected);
        gyroResetOccurred = table.get("GyroResetOccurred", gyroResetOccurred);
        gyroYawRateRadPerSec = table.get("GyroYawRateRadPerSec", gyroYawRateRadPerSec);
        gyroPitchDegrees = table.get("GyroPitchDegrees", gyroPitchDegrees);
        gyroRollDegrees = table.get("GyroRollDegrees", gyroRollDegrees);
        gyroPitchRateDegPerSec = table.get("GyroPitchRateDegPerSec", gyroPitchRateDegPerSec);
        gyroRollRateDegPerSec = table.get("GyroRollRateDegPerSec", gyroRollRateDegPerSec);
        accelXG = table.get("AccelXG", accelXG);
        accelYG = table.get("AccelYG", accelYG);
        accelZG = table.get("AccelZG", accelZG);
        onboardImuPresent = table.get("OnboardImu/Present", onboardImuPresent);
        onboardImuYawRadians = table.get("OnboardImu/YawRadians", onboardImuYawRadians);
        onboardImuYawRateRadPerSec =
                table.get("OnboardImu/YawRateRadPerSec", onboardImuYawRateRadPerSec);
        onboardImuAccelX = table.get("OnboardImu/AccelX", onboardImuAccelX);
        onboardImuAccelY = table.get("OnboardImu/AccelY", onboardImuAccelY);
        onboardImuAccelZ = table.get("OnboardImu/AccelZ", onboardImuAccelZ);
        odometryPeriodSeconds = table.get("OdometryPeriodSeconds", odometryPeriodSeconds);
        successfulDaqs = table.get("SuccessfulDaqs", successfulDaqs);
        failedDaqs = table.get("FailedDaqs", failedDaqs);
    }
}
