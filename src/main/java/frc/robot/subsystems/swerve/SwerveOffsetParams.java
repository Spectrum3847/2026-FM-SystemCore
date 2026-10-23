package frc.robot.subsystems.swerve;

import frc.spectrumLib.params.DoubleParam;
import frc.spectrumLib.params.ParamGroup;
import frc.spectrumLib.params.Params;
import frc.spectrumLib.telemetry.Alert;
import java.util.ArrayList;
import java.util.List;
import org.wpilib.driverstation.Alert.Level;

/**
 * The swerve CANcoder magnet offsets as params, group {@code swerve}, so a module re-zeroed in the
 * pit ({@link SwerveAlignment}) can be saved without a code change.
 *
 * <p>The offsets are per robot: they live in {@code params/robots/<robot>/swerve.json}, and each
 * default is the offset in that robot's config class ({@code FM2026} and so on). A missing or
 * unreadable file, or a value out of bounds, leaves the config's offset.
 *
 * <p>Read once at boot, before the drivetrain is built, because CTRE writes each offset into its
 * CANcoder's config when the drivetrain starts. A value changed later (tuning mode, disabled) does
 * not reach the modules until the robot reboots, and an alert says so until then. Applying it live
 * would take a blocking CANcoder config call and rebuilt module constants.
 */
public final class SwerveOffsetParams {
    private SwerveOffsetParams() {}

    public static final String GROUP = "swerve";

    /** In the order of {@link SwerveConfig#getEncoderOffsetRotations()}. */
    public static final List<String> KEYS =
            List.of(
                    "frontLeftEncoderOffset",
                    "frontRightEncoderOffset",
                    "backLeftEncoderOffset",
                    "backRightEncoderOffset");

    /**
     * Any offset has an equivalent within half a turn, and every robot's is there, so a value
     * outside it is a typo (a misplaced digit, or degrees for rotations).
     */
    public static final double MAX_ABS_ROTATIONS = 0.5;

    private static List<DoubleParam> params;
    private static double[] applied;
    private static Alert rebootAlert;

    /** Bit i set: offset i differs from the boot value. */
    private static int shownChanged = 0;

    /**
     * Declares the four offsets with this robot's config values as the defaults. Declared once per
     * run; a second call must come from the same robot config.
     *
     * @param config the robot's swerve config, offsets as its config class set them
     * @return the params, in the order of {@link #KEYS}
     */
    public static List<DoubleParam> declare(SwerveConfig config) {
        double[] defaults = config.getEncoderOffsetRotations();
        if (defaults == null) {
            throw new IllegalStateException("The robot config sets no swerve encoder offsets");
        }
        if (params == null) {
            ParamGroup group = Params.group(GROUP);
            List<DoubleParam> out = new ArrayList<>();
            for (int i = 0; i < KEYS.size(); i++) {
                out.add(
                        group.number(
                                KEYS.get(i),
                                defaults[i],
                                "rot",
                                -MAX_ABS_ROTATIONS,
                                MAX_ABS_ROTATIONS));
            }
            params = List.copyOf(out);
        }
        for (int i = 0; i < KEYS.size(); i++) {
            if (Double.compare(params.get(i).getDefault(), defaults[i]) != 0) {
                throw new IllegalStateException(
                        "Swerve offsets were declared for another robot config");
            }
        }
        return params;
    }

    /**
     * Declares the offsets and gives the config their resolved values. Call once at boot, after
     * {@link Params#start} and before the drivetrain is built.
     *
     * @param config the robot's swerve config
     */
    public static void apply(SwerveConfig config) {
        List<DoubleParam> p = declare(config);
        double[] current = config.getEncoderOffsetRotations();
        double[] resolved = new double[p.size()];
        boolean differs = false;
        for (int i = 0; i < resolved.length; i++) {
            resolved[i] = p.get(i).get();
            differs |= Double.compare(resolved[i], current[i]) != 0;
        }
        if (differs) {
            config.configEncoderOffsets(resolved[0], resolved[1], resolved[2], resolved[3]);
        }
        applied = resolved;
        if (rebootAlert == null) {
            rebootAlert = new Alert("", Level.MEDIUM);
        }
        shownChanged = 0;
        rebootAlert.set(false);
    }

    /**
     * Whether an offset differs from the one the drivetrain booted with, as of {@link #periodic}.
     */
    public static boolean changedSinceBoot() {
        return shownChanged != 0;
    }

    /** Raises an alert while an offset differs from the one the drivetrain booted with. */
    public static void periodic() {
        if (applied == null) {
            return;
        }
        int changed = 0;
        for (int i = 0; i < applied.length; i++) {
            if (Double.compare(params.get(i).get(), applied[i]) != 0) {
                changed |= 1 << i;
            }
        }
        if (changed == shownChanged) {
            return;
        }
        shownChanged = changed;
        List<String> keys = new ArrayList<>();
        for (int i = 0; i < applied.length; i++) {
            if ((changed & (1 << i)) != 0) {
                keys.add(KEYS.get(i));
            }
        }
        rebootAlert.setText(
                "Swerve offsets changed since boot ("
                        + String.join(", ", keys)
                        + "): save params and reboot to apply");
        rebootAlert.set(changed != 0);
    }
}
