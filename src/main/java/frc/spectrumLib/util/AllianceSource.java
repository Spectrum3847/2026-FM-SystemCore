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
 * The one alliance the whole robot uses. Nothing else should call {@link MatchState#getAlliance()},
 * which is empty until the DS reports one, so every caller agrees on a fallback.
 *
 * <p>In order: the dashboard override, the DS, the last alliance the DS reported this boot, then
 * blue so the robot always drives.
 *
 * <p>Replay resolves the same alliance, since the DS data is replayed and the override is a {@link
 * LoggedDashboardChooser}.
 */
public final class AllianceSource {
    private AllianceSource() {}

    public enum Source {
        OVERRIDE,
        DS,
        LAST_KNOWN,
        DEFAULT
    }

    public enum Override {
        AUTO,
        BLUE,
        RED
    }

    public record Resolution(Alliance alliance, Source source) {}

    /** FMS attached this long with no alliance raises the error alert. */
    static final double NEVER_ASSIGNED_SECONDS = 2.0;

    private static LoggedDashboardChooser<Override> overrideChooser;
    private static volatile Alliance lastKnown;
    private static Alert fallbackAlert;
    private static Alert neverAssignedAlert;
    private static int fmsLoopsWithoutAlliance;

    /** Pure, for tests. */
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

    /** Creates the override chooser and alerts. Until then the override reads Auto. */
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

    public static Resolution resolution() {
        Optional<Alliance> ds = MatchState.getAlliance();
        if (ds.isPresent()) {
            lastKnown = ds.get();
        }
        return resolve(override(), ds, Optional.ofNullable(lastKnown));
    }

    public static Alliance get() {
        return resolution().alliance();
    }

    public static boolean isBlue() {
        return get() == Alliance.BLUE;
    }

    public static boolean isRed() {
        return get() == Alliance.RED;
    }

    /** Call once per loop. */
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
