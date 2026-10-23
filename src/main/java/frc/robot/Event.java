package frc.robot;

import org.wpilib.vision.apriltag.AprilTagFields;

/** What changes from one event to the next. Check every value here before each event. */
public final class Event {
    private Event() {}

    /**
     * This event's field: AndyMark or Welded, from the event's field announcement. The vision code
     * loads its own layout ({@code Vision.TAG_FIELD}, which the Limelights and the Jetson must
     * match); the pre-match Settings check warns when that layout is not this one.
     */
    public static final AprilTagFields APRILTAG_FIELD = AprilTagFields.k2026RebuiltAndymark;
}
