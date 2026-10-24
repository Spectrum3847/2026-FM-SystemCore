package frc.robot;

import org.wpilib.vision.apriltag.AprilTagFields;

/** What changes from one event to the next. Check every value here before each event. */
public final class Event {
    private Event() {}

    /**
     * This event's field: AndyMark or Welded, from the event's field announcement. Vision solves
     * with this layout. The Limelights' and the Jetson's field maps are uploaded to them separately
     * and must match it; the robot cannot read them, so no check covers them.
     */
    public static final AprilTagFields APRILTAG_FIELD = AprilTagFields.k2026RebuiltAndymark;
}
