package frc.spectrumLib.localization;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;

/**
 * Robot state a gate may consult, built once per loop from logged inputs so gating replays
 * deterministically.
 *
 * @param nowSeconds current time, {@code Timer.getTimestamp()} base
 * @param fusedPose the fused robot pose before this loop's measurements
 * @param odometryPose the wheel-and-gyro-only pose, never corrected by vision
 * @param robotVelocity robot-relative chassis velocity
 * @param peakYawRateRadPerSec largest |yaw rate| over the configured lookback window
 * @param disabled whether the robot is disabled
 * @param headingSeeded whether a camera has set the field heading yet
 */
public record GateContext(
        double nowSeconds,
        Pose2d fusedPose,
        Pose2d odometryPose,
        ChassisVelocities robotVelocity,
        double peakYawRateRadPerSec,
        boolean disabled,
        boolean headingSeeded) {

    /** Robot linear speed in m/s. */
    public double linearSpeed() {
        return Math.hypot(robotVelocity.vx, robotVelocity.vy);
    }
}
