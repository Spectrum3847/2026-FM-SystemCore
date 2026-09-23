package frc.robot.subsystems;

import frc.rebuilt.ShotCalculator;
import frc.spectrumLib.telemetry.Telemetry;
import org.wpilib.math.geometry.Pose2d;

/**
 * One row per volley, under {@code ShotLog/}: what was aimed, and what the mechanisms were doing,
 * on the loop the feed opened. From the offseason bot's shot record (eb2d86e, {@code
 * ShotCalc/Shot/*}), extended with the gate's view and the pose's trust. Schema and how to read it:
 * docs/shot-log.md; {@code tools/shot_log.py} turns a log into a CSV.
 *
 * <p>The row is written on the rising edge of the feed, the first loop fuel is allowed into the
 * flywheel and so the last one on which the aim was still a prediction. When the feed closes, an
 * end row under {@code ShotLog/End/} gives the volley's length and how many flywheel dips (roughly,
 * balls) it saw.
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
        Telemetry.logDash("ShotLog/Index", index);
        Telemetry.log("ShotLog/TimestampSeconds", shot.nowSeconds(), "seconds");
        Telemetry.log("ShotLog/MatchTimeSeconds", shot.matchTimeSeconds(), "seconds");
        Telemetry.log("ShotLog/Alliance", shot.alliance());
        Telemetry.log("ShotLog/Mode", shot.autonomous() ? "Auto" : "Teleop");
        Telemetry.log("ShotLog/SuperState", shot.superState());
        Telemetry.log("ShotLog/FeedReason", shot.decision().reason().label);
        Telemetry.log("ShotLog/Bypassed", shot.decision().blocker().label);
        Telemetry.log("ShotLog/SecondsWaited", shot.secondsWaited(), "seconds");

        boolean set = shot.setShot() != null;
        var p = shot.params();
        Telemetry.log("ShotLog/Kind", set ? "SetShot" : "Calculated");
        Telemetry.log("ShotLog/SetShot", set ? shot.setShot().label : "");
        Telemetry.log("ShotLog/FeedShot", p != null && p.feedShot());
        Telemetry.log("ShotLog/Model", p != null ? p.modelName() : "HubModelStandstill");
        double distance =
                set
                        ? shot.setShot().distanceMeters
                        : p != null ? p.distanceNoLookahead() : Double.NaN;
        Telemetry.log("ShotLog/DistanceMeters", distance, "meters");
        Telemetry.log(
                "ShotLog/LookaheadDistanceMeters", p != null ? p.distance() : distance, "meters");
        Telemetry.log(
                "ShotLog/RadialVelocityMs", p != null ? p.radialVelocity() : Double.NaN, "m/s");
        Telemetry.log(
                "ShotLog/TangentialVelocityMs",
                p != null ? p.tangentialVelocity() : Double.NaN,
                "m/s");
        Telemetry.log("ShotLog/RobotSpeedMs", shot.robotSpeedMs(), "m/s");
        Telemetry.log(
                "ShotLog/TimeOfFlightSeconds", p != null ? p.timeOfFlight() : Double.NaN, "s");
        Telemetry.log("ShotLog/InRange", p != null ? p.isValid() : set);

        Telemetry.log("ShotLog/WantedRPM", shot.wantedRpm(), "RPM");
        Telemetry.log("ShotLog/ActualRPM", shot.actualRpm(), "RPM");
        Telemetry.log("ShotLog/WantedHoodDeg", shot.wantedHoodDeg(), "degrees");
        Telemetry.log("ShotLog/ActualHoodDeg", shot.actualHoodDeg(), "degrees");
        // Fused heading minus the aim target, while the drivetrain aims; NaN for a set shot.
        Telemetry.log("ShotLog/HeadingErrorDeg", Math.toDegrees(shot.headingErrorRad()), "deg");
        Telemetry.log("ShotLog/AimToleranceDeg", Math.toDegrees(shot.aimToleranceRad()), "deg");
        Telemetry.log("ShotLog/HoodTrimDeg", ShotCalculator.HOOD_ANGLE_OFFSET, "degrees");
        Telemetry.log("ShotLog/AimTrimDeg", ShotCalculator.DRIVE_ANGLE_OFFSET, "degrees");

        Telemetry.log("ShotLog/PoseTrusted", shot.poseTrusted());
        Telemetry.log(
                "ShotLog/SecondsSinceFusedEstimate",
                Math.min(shot.secondsSinceFusedEstimate(), 999.0),
                "seconds");
        Telemetry.log("ShotLog/SeedConfirmed", shot.seedConfirmed());
        Telemetry.log("ShotLog/Pose", shot.pose());
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
        Telemetry.log("ShotLog/End/Index", index);
        Telemetry.log("ShotLog/End/BurstSeconds", nowSeconds - startSeconds, "seconds");
        Telemetry.log("ShotLog/End/FlywheelDips", (long) dips.count());
        Telemetry.log("ShotLog/End/MinRPM", minRpm, "RPM");
    }

    /**
     * Counts the flywheel dips of a volley: each ball loading the wheel pulls its speed down, then
     * it recovers (3467's {@code detectFlywheelDrop}). A dip is the speed falling more than {@link
     * #DIP_RPM} below target after having been within half of that; the half-way re-arm is the
     * hysteresis that stops one slow recovery counting twice.
     *
     * <p>CALIBRATE: {@link #DIP_RPM} is a guess until a real volley's {@code Launcher/RPM} has been
     * looked at; the sim flywheel does not feel the fuel, so it never dips there. If counts do not
     * match the balls on video, change the threshold before trusting {@code End/FlywheelDips}.
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
