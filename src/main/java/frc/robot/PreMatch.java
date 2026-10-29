package frc.robot;

import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveAlignment;
import frc.robot.subsystems.swerve.SwerveOffsetParams;
import frc.robot.subsystems.vision.Jetson;
import frc.robot.subsystems.vision.Vision;
import frc.spectrumLib.checks.AutoStartCheck;
import frc.spectrumLib.checks.BatteryCheck;
import frc.spectrumLib.checks.Check;
import frc.spectrumLib.checks.CheckResult;
import frc.spectrumLib.checks.CheckResult.Status;
import frc.spectrumLib.checks.DeviceHealthCheck;
import frc.spectrumLib.checks.HomingCheck;
import frc.spectrumLib.checks.PreMatchChecks;
import frc.spectrumLib.checks.SettingsCheck;
import frc.spectrumLib.hardware.CanBuses;
import frc.spectrumLib.localization.PoseSource;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.params.Param;
import frc.spectrumLib.params.ParamSource;
import frc.spectrumLib.params.Params;
import frc.spectrumLib.params.TuningMode;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;
import java.util.OptionalDouble;
import java.util.Set;
import java.util.TreeSet;
import java.util.function.Supplier;
import java.util.stream.Collectors;
import org.wpilib.driverstation.RobotState;

/** FM's pre-match checks: which devices, mechanisms, cameras and settings each check covers. */
final class PreMatch {
    private PreMatch() {}

    static PreMatchChecks create(
            Swerve swerve,
            Vision vision,
            Hood hood,
            AutoStartCheck autoStart,
            OptionalDouble batteryVoltsOffset) {
        return new PreMatchChecks("PreMatch")
                .register("CAN", can(swerve))
                .register("Homing", homing(hood))
                .register("CameraHeading", vision.getCameraHeadings())
                .register("GyroMount", swerve.getMountCheck())
                .register("Auto", autoStart)
                .register("Battery", battery(batteryVoltsOffset))
                .register("Coprocessors", coprocessors(vision))
                .register("Settings", settings(swerve, vision));
    }

    /** The simulator's battery is a constant 12 V, which would only ever warn. */
    private static Check battery(OptionalDouble offsetVolts) {
        BatteryCheck battery =
                new BatteryCheck(
                        RuntimeInputs::batteryVoltage,
                        offsetVolts,
                        RobotState::isDisabled,
                        () -> BatteryCheck.Identity.NONE);
        return new Check() {
            @Override
            public void observe(double now) {
                battery.observe(now);
            }

            @Override
            public CheckResult evaluate(double now) {
                return RuntimeInputs.isSimulation()
                        ? CheckResult.skip("simulated battery")
                        : battery.evaluate(now);
            }
        };
    }

    private static DeviceHealthCheck can(Swerve swerve) {
        DeviceHealthCheck can = new DeviceHealthCheck("CAN devices");
        String drivetrain = CanBuses.label(CanBuses.DRIVETRAIN);
        var in = swerve.getInputs();
        for (int i = 0; i < SwerveAlignment.MODULE_NAMES.length; i++) {
            final int m = i;
            // Logs from before ModuleConnected was recorded have none: the swerve treats those as
            // connected, and so does this.
            can.add(
                            drivetrain,
                            SwerveAlignment.MODULE_NAMES[i] + " module",
                            () -> in.moduleConnected.length != 4 || in.moduleConnected[m])
                    .resets(() -> swerve.moduleRebootedThisLoop(m));
        }
        can.add(drivetrain, "Pigeon", () -> in.gyroConnected).resets(swerve::gyroRebootedThisLoop);
        for (Mechanism m : Mechanism.all()) {
            can.addMechanism(m, CanBuses.label(m.config.getId().getBus()));
        }
        return can;
    }

    private static HomingCheck homing(Hood hood) {
        HomingCheck homing = new HomingCheck();
        for (Mechanism m : Mechanism.all()) {
            if (m instanceof Hood) {
                if (hood.isAttached()) {
                    homing.add(hood.getName(), hood::getHomingCheckState);
                }
            } else {
                homing.addMechanism(m);
            }
        }
        return homing;
    }

    /**
     * A camera the robot fuses must answer; one that is only logged may be missing. FM's Limelights
     * fuse by default; the SystemCore cameras and the Orin only when switched on.
     */
    private static DeviceHealthCheck coprocessors(Vision vision) {
        DeviceHealthCheck co = new DeviceHealthCheck("coprocessors");
        for (PoseSource s : vision.getMt1Sources()) {
            co.add("Limelight", s.getName().replace("/MT1", ""), s::isConnected)
                    .missingIs(() -> s.isEnabled() ? Status.FAIL : Status.WARN);
        }
        for (PoseSource s : vision.getScMt1Sources()) {
            co.add("SystemCore camera", s.getName().replace("/MT1", ""), s::isConnected)
                    .missingIs(() -> s.isEnabled() ? Status.FAIL : Status.SKIP);
        }
        Jetson jetson = vision.getJetson();
        // Registered from the config alone, so a replay registers what the robot did; whether it
        // must answer comes from the logged mode (the simulator has no Jetson).
        if (vision.getConfig().isJetsonInstalled()) {
            List<PoseSource> orin = vision.getOrinSources();
            co.add("Jetson", "Jetson", jetson::isResponding)
                    .missingIs(
                            () ->
                                    !jetson.isExpected()
                                            ? Status.SKIP
                                            : orin.stream().anyMatch(PoseSource::isEnabled)
                                                    ? Status.FAIL
                                                    : Status.WARN);
            List<Jetson.Camera> cameras = jetson.getCameras();
            for (int i = 0; i < cameras.size() && i < orin.size(); i++) {
                Jetson.Camera c = cameras.get(i);
                PoseSource s = orin.get(i);
                if (!c.installed()) {
                    continue;
                }
                co.add("Jetson", c.name(), () -> c.source().getCameraInputs().connected)
                        .missingIs(
                                () ->
                                        !jetson.isExpected()
                                                ? Status.SKIP
                                                : s.isEnabled() ? Status.FAIL : Status.WARN)
                        .problem(
                                () ->
                                        jetson.timestampsLookWrong(c.name())
                                                ? "timestamps far from now (clock sync?)"
                                                : null);
            }
        }
        return co;
    }

    /**
     * The Jetson's field map against the robot's.
     *
     * @param expected whether the Jetson should be answering ({@link Jetson#isExpected})
     * @param robotHash the canonical hash of the map the robot solves with
     * @param jetsonHash the hash the Jetson reports for the map it loaded, empty when it reports
     *     none
     * @param jetsonError why the Jetson could not load the robot's map, or empty
     * @return PASS when they match, FAIL when they differ, WARN when the Jetson reports none, SKIP
     *     when no Jetson is expected
     */
    static CheckResult jetsonFieldMap(
            boolean expected, String robotHash, String jetsonHash, String jetsonError) {
        if (!expected) {
            return CheckResult.skip("no Jetson");
        }
        if (jetsonHash.isEmpty()) {
            return CheckResult.warn(
                    "Jetson not reporting its map"
                            + (jetsonError.isEmpty() ? "" : " (" + jetsonError + ")"));
        }
        if (!jetsonHash.equals(robotHash)) {
            return CheckResult.fail(
                    "Jetson field map differs (Jetson "
                            + jetsonHash
                            + ", robot "
                            + robotHash
                            + ")");
        }
        return CheckResult.pass("Jetson's map matches");
    }

    private static String reasonIf(CheckResult r, Status status) {
        return r.status() == status ? r.reason() : null;
    }

    private static SettingsCheck settings(Swerve swerve, Vision vision) {
        Jetson jetson = vision.getJetson();
        Set<Integer> defaultExcluded =
                Arrays.stream(vision.getConfig().getOrinExcludedTags())
                        .boxed()
                        .collect(Collectors.toCollection(TreeSet::new));
        // Vision solves with Event.APRILTAG_FIELD (in replay, the match's field), so the robot's
        // own map is the event's by construction. The Jetson reports the hash of the map it
        // loaded.
        Supplier<CheckResult> jetsonMap =
                () ->
                        jetsonFieldMap(
                                jetson.isExpected(),
                                Vision.getTagLayoutHash(),
                                jetson.getInputs().fieldMapHash,
                                jetson.getInputs().fieldMapError);
        SettingsCheck settings =
                new SettingsCheck()
                        .info(
                                () ->
                                        "field "
                                                + Vision.getTagField().name()
                                                + (jetsonMap.get().status() == Status.PASS
                                                        ? ", Jetson's map matches"
                                                        : ""))
                        .add(() -> reasonIf(jetsonMap.get(), Status.FAIL), Status.FAIL)
                        .add(() -> reasonIf(jetsonMap.get(), Status.WARN), Status.WARN)
                        .add(
                                () ->
                                        jetson.isPracticeFieldProfile()
                                                        && !RobotState.isFMSAttached()
                                                ? "Orin practice-field profile ON"
                                                : null,
                                Status.WARN)
                        .add(
                                () ->
                                        jetson.isPracticeFieldProfile()
                                                        && RobotState.isFMSAttached()
                                                ? "Orin practice-field profile ON in a match"
                                                : null,
                                Status.FAIL)
                        .add(
                                () ->
                                        new TreeSet<>(jetson.getExcludedTags())
                                                        .equals(defaultExcluded)
                                                ? null
                                                : "Orin excluded tags " + jetson.getExcludedTags(),
                                Status.WARN)
                        .add(
                                () -> {
                                    Set<Long> extra = new TreeSet<>();
                                    for (long id : jetson.getInputs().excludedTagsActive) {
                                        if (!jetson.getExcludedTags().contains((int) id)) {
                                            extra.add(id);
                                        }
                                    }
                                    return extra.isEmpty()
                                            ? null
                                            : "Jetson also excludes tags " + extra;
                                },
                                Status.WARN)
                        .add(
                                () -> vision.isChassisUseMt2() ? null : "MegaTag2 fusion OFF",
                                Status.WARN)
                        .add(
                                () ->
                                        swerve.isTiltScaledOdometry()
                                                ? null
                                                : "tilt-scaled odometry OFF",
                                Status.WARN);
        for (PoseSource s : vision.getAllSources()) {
            settings.add(
                    () ->
                            s.isEnabled() == s.isEnabledByDefault()
                                    ? null
                                    : s.getName() + " fusion " + (s.isEnabled() ? "ON" : "OFF"),
                    Status.WARN);
        }
        addParams(settings);
        return settings;
    }

    /**
     * Params: tuning mode, and the hard stops (swerve encoder offsets, soft limits). A hard stop
     * from the pit's local file or a live change was not reviewed in git; one changed since boot is
     * not what the motors run on until a reboot; one a file tried to set but was refused is not
     * what someone meant.
     */
    private static void addParams(SettingsCheck settings) {
        List<Param<?>> hardStops = hardStopParams();
        settings.add(() -> TuningMode.enabled() ? "tuning mode ON" : null, Status.WARN)
                .add(() -> notFromGit(hardStops), Status.WARN)
                .add(
                        () ->
                                SwerveOffsetParams.changedSinceBoot()
                                        ? "swerve offsets changed since boot, reboot to apply"
                                        : null,
                        Status.FAIL)
                .add(PreMatch::softLimitsChanged, Status.FAIL)
                .add(() -> refused(hardStops), Status.FAIL);
    }

    private static String notFromGit(List<Param<?>> hardStops) {
        List<String> out = new ArrayList<>();
        for (Param<?> p : hardStops) {
            if (p.source() == ParamSource.LOCAL || p.source() == ParamSource.LIVE) {
                out.add(p.path() + " (" + p.source().name().toLowerCase(Locale.ROOT) + ")");
            }
        }
        return out.isEmpty() ? null : "hard stops not from git: " + String.join(", ", out);
    }

    private static String softLimitsChanged() {
        List<String> out = new ArrayList<>();
        for (Mechanism m : Mechanism.all()) {
            if (m.softLimitsNeedReboot()) {
                out.add(m.getName());
            }
        }
        return out.isEmpty()
                ? null
                : String.join(", ", out) + " soft limits changed since boot, reboot to apply";
    }

    private static String refused(List<Param<?>> hardStops) {
        List<String> out = new ArrayList<>();
        for (Param<?> p : hardStops) {
            if (Params.refusedInFile(p)) {
                out.add(p.path());
            }
        }
        return out.isEmpty()
                ? null
                : "a params file's value was refused for hard stops " + String.join(", ", out);
    }

    /** The swerve encoder offsets and every mechanism's soft limits, as declared at boot. */
    private static List<Param<?>> hardStopParams() {
        List<Param<?>> out = new ArrayList<>();
        for (String key : SwerveOffsetParams.KEYS) {
            Param<?> p = Params.find(SwerveOffsetParams.GROUP + "/" + key);
            if (p != null) {
                out.add(p);
            }
        }
        for (Mechanism m : Mechanism.all()) {
            for (Param<?> p :
                    Arrays.asList(
                            m.config.getForwardSoftLimitParam(),
                            m.config.getReverseSoftLimitParam())) {
                if (p != null) {
                    out.add(p);
                }
            }
        }
        return out;
    }
}
