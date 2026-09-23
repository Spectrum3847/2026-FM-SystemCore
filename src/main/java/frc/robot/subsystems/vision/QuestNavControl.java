package frc.robot.subsystems.vision;

import org.wpilib.math.geometry.Pose2d;

/** The one command the robot sends a QuestNav: where the robot is, so it can re-origin. */
public interface QuestNavControl {
    /** Does nothing, for replay. */
    QuestNavControl NONE = pose -> {};

    /**
     * Tells the Quest the robot's current field pose.
     *
     * @param robotPose robot pose on the field now
     */
    void resetPose(Pose2d robotPose);
}
