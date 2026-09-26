package frc.spectrumLib.util;

import frc.spectrumLib.framework.RobotLoop;
import frc.spectrumLib.telemetry.Alert;
import frc.spectrumLib.telemetry.Telemetry;
import java.util.Optional;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.wpilib.driverstation.Alert.Level;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;

/**
 * The one alliance the whole robot uses: teleop drive direction, aim and shot targets, field
 * flipping, PathPlanner's {@code shouldFlip}, the auto preview, LEDs, the shift schedule. Nothing
 * else should call {@link MatchState#getAlliance()} to decide anything.
 *
 * <p>Why: in 2027 alpha-6 {@code MatchState.getAlliance()} is empty until the Driver Station (or
 * FMS) reports one, and code that did {@code orElse(BLUE)} in one place and "stop when empty" in
 * another disagreed. Teleop drive returned zero velocity whenever it was empty, so the robot would
 * not drive at all (2026 defaulted to blue). Here, in order:
 *
 * <ol>
 *   <li>{@link Source#OVERRIDE}: the dashboard's {@code Alliance Override} chooser, if set to Blue
 *       or Red (default Auto). For a DS that reports the wrong alliance, or bench testing.
 *   <li>{@link Source#DS}: what the Driver Station / FMS reports right now. Checked on every call,
 *       so the robot switches as soon as one arrives, whenever in the match that is.
 *   <li>{@link Source#LAST_KNOWN}: the last alliance the DS reported this boot, so a momentary
 *       drop-out does not flip the robot back to blue mid-match.
 *   <li>{@link Source#DEFAULT}: blue, so the robot always drives.
 * </ol>
 *
 * <p>Anything but {@code DS} raises a warning; the FMS attached with no alliance ever reported for
 * {@link #NEVER_ASSIGNED_SECONDS} raises an error. The alliance in use and where it came from are
 * logged under {@code Alliance/}.
 *
 * <p>Replay: the DS alliance comes through AdvantageKit's replayed match data and the override is a
 * {@link LoggedDashboardChooser}, so replay resolves the same alliance on the same loop.
 */
public final class AllianceSource {
    private AllianceSource() {}

    /** Where the alliance in use came from. */
    public enum Source {
        OVERRIDE,
        DS,
        LAST_KNOWN,
        DEFAULT
    }

    /** The dashboard override's choices. */
    public enum Override {
        AUTO,
        BLUE,
        RED
    }

    /**
     * An alliance and where it came from.
     *
     * @param alliance the alliance to use
     * @param source where it came from
     */
    public record Resolution(Alliance alliance, Source source) {}

    /** FMS attached this long with no alliance ever reported raises the error alert. */
    static final double NEVER_ASSIGNED_SECONDS = 2.0;

    private static LoggedDashboardChooser<Override> overrideChooser;
    private static volatile Alliance lastKnown;
    private static Alert fallbackAlert;
    private static Alert neverAssignedAlert;
    private static int fmsLoopsWithoutAlliance;

    /**
     * Resolves the alliance from the override, the Driver Station, and the last alliance the DS
     * reported, in that order, falling back to blue. Pure, for tests.
     *
     * @param override the dashboard override
     * @param ds what the Driver Station reports now (empty if nothing)
     * @param lastKnown the last alliance the DS reported (empty if never)
     * @return the alliance and its source
     */
    public static Resolution resolve(
            Override override, Optional<Alliance> ds, Optional<Alliance> lastKnown) {
        if (override == Override.BLUE) {
            return new Resolution(Alliance.BLUE, Source.OVERRIDE);
        }
        if (override == Override.RED) {
            return new Resolution(Alliance.RED, Source.OVERRIDE);
        }
        if (ds.isPresent()) {
            return new Resolution(ds.get(), Source.DS);
        }
        if (lastKnown.isPresent()) {
            return new Resolution(lastKnown.get(), Source.LAST_KNOWN);
        }
        return new Resolution(Alliance.BLUE, Source.DEFAULT);
    }

    /**
     * Creates the dashboard override chooser ({@code /SmartDashboard/Alliance Override}) and the
     * alerts. Call once from the robot constructor; until then (and in unit tests) the override
     * reads Auto.
     */
    public static void init() {
        if (overrideChooser != null) {
            return;
        }
        overrideChooser = new LoggedDashboardChooser<>("Alliance Override");
        overrideChooser.addDefaultOption("Auto", Override.AUTO);
        overrideChooser.addOption("Blue", Override.BLUE);
        overrideChooser.addOption("Red", Override.RED);
        fallbackAlert = new Alert("", Level.MEDIUM);
        neverAssignedAlert =
                new Alert(
                        "FMS attached but no alliance assigned: driving and aiming as BLUE",
                        Level.HIGH);
    }

    private static Override override() {
        Override o = overrideChooser == null ? null : overrideChooser.get();
        return o == null ? Override.AUTO : o;
    }

    /**
     * The alliance in use and its source, from this moment's DS data. Cheap; call wherever needed.
     *
     * @return the resolution
     */
    public static Resolution resolution() {
        Optional<Alliance> ds = MatchState.getAlliance();
        if (ds.isPresent()) {
            lastKnown = ds.get();
        }
        return resolve(override(), ds, Optional.ofNullable(lastKnown));
    }

    /** The alliance the robot is on (never empty; see the class comment for the fallbacks). */
    public static Alliance get() {
        return resolution().alliance();
    }

    /** Whether the robot is on the blue alliance. */
    public static boolean isBlue() {
        return get() == Alliance.BLUE;
    }

    /** Whether the robot is on the red alliance. */
    public static boolean isRed() {
        return get() == Alliance.RED;
    }

    /** Alerts and log values. Call once per loop from {@code robotPeriodic()}. */
    public static void periodic() {
        Resolution r = resolution();
        boolean fms = RobotState.isFMSAttached();
        if (fms && lastKnown == null) {
            fmsLoopsWithoutAlliance++;
        } else {
            fmsLoopsWithoutAlliance = 0;
        }
        // Counted in loops, not wall time, so replay raises it on the same loop.
        boolean neverAssigned =
                fmsLoopsWithoutAlliance * RobotLoop.periodSeconds() >= NEVER_ASSIGNED_SECONDS;

        if (fallbackAlert != null) {
            String why =
                    switch (r.source()) {
                        case OVERRIDE -> "dashboard override";
                        case LAST_KNOWN -> "last known; the DS stopped reporting one";
                        case DEFAULT -> "default; the DS has not reported one";
                        case DS -> "";
                    };
            fallbackAlert.setText("Alliance: using " + r.alliance().name() + " (" + why + ")");
            fallbackAlert.set(r.source() != Source.DS);
            neverAssignedAlert.set(neverAssigned);
        }

        Telemetry.log("Alliance/InUse", r.alliance().name());
        Telemetry.log("Alliance/Source", r.source().name());
        Telemetry.log("Alliance/Display", r.alliance().name() + " (" + r.source().name() + ")");
        Telemetry.log("Alliance/Override", override().name());
    }
}
