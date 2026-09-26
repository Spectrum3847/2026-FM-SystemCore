package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveModuleVelocity;

/**
 * Everything the drivetrain reads from CTRE in one loop, as an AdvantageKit input.
 *
 * <p>The CTRE {@code SwerveDrivetrain} keeps doing what it does best -- a 250 Hz odometry thread
 * and the swerve requests -- but none of the robot's logic reads its pose directly. Instead, every
 * odometry sample it produced since the last loop is queued off its thread and recorded here, and
 * {@link frc.spectrumLib.localization.PoseFusion} integrates them on the main loop. That is what
 * makes the robot pose replayable: in replay these arrays come back from the log, the fusion
 * re-runs over them, and every vision gate sees the same pose it saw on the field.
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
    public Pose2d ctrePose = Pose2d.kZero;

    public ChassisVelocities robotVelocity = new ChassisVelocities();
    public SwerveModuleVelocity[] moduleVelocities = new SwerveModuleVelocity[0];
    public SwerveModuleVelocity[] moduleTargets = new SwerveModuleVelocity[0];

    public boolean gyroConnected = false;
    public double gyroYawRateRadPerSec = 0;
    public double gyroPitchDegrees = 0;
    public double gyroRollDegrees = 0;
    public double gyroPitchRateDegPerSec = 0;
    public double gyroRollRateDegPerSec = 0;

    /** Pigeon acceleration along its own axes, g (gravity included). All 0 when not read. */
    public double accelXG = 0;

    public double accelYG = 0;
    public double accelZG = 0;

    public double odometryPeriodSeconds = 0;
    public int successfulDaqs = 0;
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
        table.put("GyroConnected", gyroConnected);
        table.put("GyroYawRateRadPerSec", gyroYawRateRadPerSec);
        table.put("GyroPitchDegrees", gyroPitchDegrees);
        table.put("GyroRollDegrees", gyroRollDegrees);
        table.put("GyroPitchRateDegPerSec", gyroPitchRateDegPerSec);
        table.put("GyroRollRateDegPerSec", gyroRollRateDegPerSec);
        table.put("AccelXG", accelXG);
        table.put("AccelYG", accelYG);
        table.put("AccelZG", accelZG);
        table.put("OdometryPeriodSeconds", odometryPeriodSeconds);
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
        gyroConnected = table.get("GyroConnected", gyroConnected);
        gyroYawRateRadPerSec = table.get("GyroYawRateRadPerSec", gyroYawRateRadPerSec);
        gyroPitchDegrees = table.get("GyroPitchDegrees", gyroPitchDegrees);
        gyroRollDegrees = table.get("GyroRollDegrees", gyroRollDegrees);
        gyroPitchRateDegPerSec = table.get("GyroPitchRateDegPerSec", gyroPitchRateDegPerSec);
        gyroRollRateDegPerSec = table.get("GyroRollRateDegPerSec", gyroRollRateDegPerSec);
        accelXG = table.get("AccelXG", accelXG);
        accelYG = table.get("AccelYG", accelYG);
        accelZG = table.get("AccelZG", accelZG);
        odometryPeriodSeconds = table.get("OdometryPeriodSeconds", odometryPeriodSeconds);
        successfulDaqs = table.get("SuccessfulDaqs", successfulDaqs);
        failedDaqs = table.get("FailedDaqs", failedDaqs);
    }
}
