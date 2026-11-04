package frc.robot.subsystems;

import frc.rebuilt.ShotCalculator;
import frc.spectrumLib.telemetry.Telemetry;
import org.wpilib.math.geometry.Pose2d;

/**
 * One row per volley, under {@code Shooting/Log/}: what was aimed, and what the mechanisms were
 * doing, on the loop the feed opened. From the offseason bot's shot record (eb2d86e, {@code
 * ShotCalc/Shot/*}), extended with the gate's view and the pose's trust. Schema and how to read it:
 * docs/shot-log.md; {@code tools/shot_log.py} turns a log into a CSV.
 *
 * <p>The row is written on the rising edge of the feed: the first loop fuel is allowed into the
 * flywheel, and so the last loop on which the aim is still a prediction. When the feed closes, an
 * end row under {@code Shooting/Log/End/} gives the volley's length and how many flywheel dips
 * (roughly, balls) it saw.
 *
 * <p>AdvantageKit writes a value only when it changes, so a key that is the same as last shot's has
 * no record at this shot's time. Read a row as each key's last value at or before the row's {@code
 * Index} record; the tool does exactly that. Main thread only, like every {@link Telemetry} call.
 */
public final class ShotLog {

    /** Everything a row needs that the log cannot look up itself. */
    public record Shot(
            double nowSeconds,
            double matchTimeSeconds,
            String alliance,
            boolean autonomous,
            String superState,
            ShotGate.Decision decision,
            double secondsWaited,
            /** The calculated shot's solution, or null for a set shot. */
            ShotCalculator.ShootingParameters params,
            /** The set shot being fired, or null for a calculated shot. */
            ShotCalculator.SetShot setShot,
            double wantedRpm,
            double actualRpm,
            double wantedHoodDeg,
            double actualHoodDeg,
            double headingErrorRad,
            double aimToleranceRad,
            boolean poseTrusted,
            double secondsSinceFusedEstimate,
            boolean seedConfirmed,
            double robotSpeedMs,
            Pose2d pose) {}

    private long index = 0;
    private double startSeconds = Double.NaN;
    private final DipCounter dips = new DipCounter();
    private double minRpm = Double.NaN;

    /** Volleys logged since boot. */
    public long count() {
        return index;
    }

    /** Writes the row for a volley that starts this loop. */
    public void start(Shot shot) {
        index++;
        startSeconds = shot.nowSeconds();
        dips.reset();
        minRpm = shot.actualRpm();

        // The one key on the dashboard: bursts counting up is how you know rows are being written.
        Telemetry.logDash("Shooting/Log/Index", index);
        Telemetry.log("Shooting/Log/TimestampSeconds", shot.nowSeconds(), "seconds");
        Telemetry.log("Shooting/Log/MatchTimeSeconds", shot.matchTimeSeconds(), "seconds");
        Telemetry.log("Shooting/Log/Alliance", shot.alliance());
        Telemetry.log("Shooting/Log/Mode", shot.autonomous() ? "Auto" : "Teleop");
        Telemetry.log("Shooting/Log/SuperState", shot.superState());
        Telemetry.log("Shooting/Log/FeedReason", shot.decision().reason().label);
        Telemetry.log("Shooting/Log/Bypassed", shot.decision().blocker().label);
        Telemetry.log("Shooting/Log/SecondsWaited", shot.secondsWaited(), "seconds");

        boolean set = shot.setShot() != null;
        var p = shot.params();
        Telemetry.log("Shooting/Log/Kind", set ? "SetShot" : "Calculated");
        Telemetry.log("Shooting/Log/SetShot", set ? shot.setShot().label : "");
        Telemetry.log("Shooting/Log/FeedShot", p != null && p.feedShot());
        Telemetry.log("Shooting/Log/Model", p != null ? p.modelName() : "HubModelStandstill");
        double distance =
                set
                        ? shot.setShot().distanceMeters
                        : p != null ? p.distanceNoLookahead() : Double.NaN;
        Telemetry.log("Shooting/Log/DistanceMeters", distance, "meters");
        Telemetry.log(
                "Shooting/Log/LookaheadDistanceMeters",
                p != null ? p.distance() : distance,
                "meters");
        Telemetry.log(
                "Shooting/Log/RadialVelocityMs",
                p != null ? p.radialVelocity() : Double.NaN,
                "m/s");
        Telemetry.log(
                "Shooting/Log/TangentialVelocityMs",
                p != null ? p.tangentialVelocity() : Double.NaN,
                "m/s");
        Telemetry.log("Shooting/Log/RobotSpeedMs", shot.robotSpeedMs(), "m/s");
        Telemetry.log(
                "Shooting/Log/TimeOfFlightSeconds", p != null ? p.timeOfFlight() : Double.NaN, "s");
        Telemetry.log("Shooting/Log/InRange", p != null ? p.isValid() : set);

        Telemetry.log("Shooting/Log/WantedRPM", shot.wantedRpm(), "RPM");
        Telemetry.log("Shooting/Log/ActualRPM", shot.actualRpm(), "RPM");
        Telemetry.log("Shooting/Log/WantedHoodDeg", shot.wantedHoodDeg(), "degrees");
        Telemetry.log("Shooting/Log/ActualHoodDeg", shot.actualHoodDeg(), "degrees");
        // Fused heading minus the aim target, while the drivetrain aims; NaN for a set shot.
        Telemetry.log(
                "Shooting/Log/HeadingErrorDeg", Math.toDegrees(shot.headingErrorRad()), "degrees");
        Telemetry.log(
                "Shooting/Log/AimToleranceDeg", Math.toDegrees(shot.aimToleranceRad()), "degrees");
        Telemetry.log("Shooting/Log/HoodTrimDeg", ShotCalculator.HOOD_ANGLE_OFFSET, "degrees");
        Telemetry.log("Shooting/Log/AimTrimDeg", ShotCalculator.DRIVE_ANGLE_OFFSET, "degrees");

        Telemetry.log("Shooting/Log/PoseTrusted", shot.poseTrusted());
        Telemetry.log(
                "Shooting/Log/SecondsSinceFusedEstimate",
                Math.min(shot.secondsSinceFusedEstimate(), 999.0),
                "seconds");
        Telemetry.log("Shooting/Log/SeedConfirmed", shot.seedConfirmed());
        Telemetry.log("Shooting/Log/Pose", shot.pose());
    }

    /** Called every loop while a volley is feeding. */
    public void during(double rpm, double targetRpm) {
        dips.update(rpm, targetRpm);
        if (!(rpm >= minRpm)) {
            minRpm = rpm;
        }
    }

    /** Writes the end row for the volley that stopped feeding this loop. */
    public void end(double nowSeconds) {
        Telemetry.log("Shooting/Log/End/Index", index);
        Telemetry.log("Shooting/Log/End/BurstSeconds", nowSeconds - startSeconds, "seconds");
        Telemetry.log("Shooting/Log/End/FlywheelDips", (long) dips.count());
        Telemetry.log("Shooting/Log/End/MinRPM", minRpm, "RPM");
    }

    /**
     * Counts the flywheel dips of a volley: each ball loading the wheel pulls its speed down, then
     * it recovers (3467's {@code detectFlywheelDrop}). A dip is the speed falling more than {@link
     * #DIP_RPM} below target after having been within half of that; the half-way re-arm is the
     * hysteresis that stops one slow recovery counting twice.
     *
     * <p>CALIBRATE: {@link #DIP_RPM} is a guess until a real volley's {@code Launcher/RPM} has been
     * looked at. The sim flywheel does not model fuel loading, so it never dips there. If counts do
     * not match the balls on video, change the threshold before trusting {@code End/FlywheelDips}.
     */
    static final class DipCounter {
        /** How far below target counts as a ball, RPM. */
        static final double DIP_RPM = 150;

        private boolean armed = false;
        private int count = 0;

        void reset() {
            armed = false;
            count = 0;
        }

        void update(double rpm, double targetRpm) {
            if (!(targetRpm > 0)) {
                return;
            }
            double below = targetRpm - rpm;
            if (below <= DIP_RPM / 2) {
                armed = true;
            } else if (armed && below > DIP_RPM) {
                count++;
                armed = false;
            }
        }

        int count() {
            return count;
        }
    }
}
