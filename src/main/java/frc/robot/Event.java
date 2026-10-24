package frc.robot;

import org.wpilib.vision.apriltag.AprilTagFields;

/** What changes from one event to the next. Check every value here before each event. */
public final class Event {
    private Event() {}

    /**
     * This event's field: AndyMark or Welded, from the event's field announcement. Vision solves
     * with this layout and publishes it at {@code /SpectrumVision/FieldMap} for the Jetson, whose
     * reported hash the pre-match Settings check compares.
     */
    public static final AprilTagFields APRILTAG_FIELD = AprilTagFields.k2026RebuiltAndymark;
}
