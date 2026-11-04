package frc.robot.subsystems.vision;

import frc.spectrumLib.params.BooleanParam;
import frc.spectrumLib.params.DoubleParam;
import frc.spectrumLib.params.ParamGroup;
import frc.spectrumLib.params.Params;

/** Camera housekeeping, from {@code params/cameras.json}. None of these gate a pose. */
public final class CameraParams {
    private CameraParams() {}

    private static final ParamGroup CAMERAS = Params.group("cameras");

    /** Limelight rewind saved at the end of auto: auto's 20 s plus a few seconds either side. */
    public static final DoubleParam AUTO_REWIND_SECONDS =
            CAMERAS.number("autoRewindSeconds", 25, "s", 0, 165);

    /**
     * Limelight rewind saved at the end of teleop: the most a Limelight keeps. Teleop's 140 s plus
     * the gap after auto, so it also reaches back into the end of auto.
     */
    public static final DoubleParam TELEOP_REWIND_SECONDS =
            CAMERAS.number("teleopRewindSeconds", 165, "s", 0, 165);

    /**
     * How often the settings the robot owns (IMU mode, and the mount poses when {@link
     * #PUSH_LIMELIGHT_MOUNTS} is on) are re-sent. A Limelight that boots after the robot, or
     * reboots mid-match, comes back in IMU mode 0 and on whatever mount was last saved to it; in
     * mode 0 its MegaTag2 poses are garbage, and FM fuses MegaTag2 once the seed is confirmed.
     * Offseason 71dc7a7. The floor keeps a typo from flooding NT every loop.
     */
    public static final DoubleParam SETTINGS_RESEND_PERIOD_SECONDS =
            CAMERAS.number("settingsResendPeriodSeconds", 2.0, "s", 0.5, 30);

    /**
     * Whether to push the configured mounts to the Limelights, so a reflashed or swapped camera
     * still gets the right mount.
     */
    public static final BooleanParam PUSH_LIMELIGHT_MOUNTS =
            CAMERAS.bool("pushLimelightMounts", true);
}
