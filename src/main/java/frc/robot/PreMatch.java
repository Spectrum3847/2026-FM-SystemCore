package frc.robot;

import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveAlignment;
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
import java.util.Arrays;
import java.util.List;
import java.util.OptionalDouble;
import java.util.Set;
import java.util.TreeSet;
import java.util.stream.Collectors;
import org.wpilib.driverstation.RobotState;
import org.wpilib.vision.apriltag.AprilTagFieldLayout;
import org.wpilib.vision.apriltag.AprilTagFields;

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
     * fuse by default; the SystemCore cameras, the Orin and the Quest only when switched on.
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
        if (jetson.isExpected()) {
            List<PoseSource> orin = vision.getOrinSources();
            co.add("Jetson", "Jetson", jetson::isResponding)
                    .missingIs(
                            () ->
                                    orin.stream().anyMatch(PoseSource::isEnabled)
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
                        .missingIs(() -> s.isEnabled() ? Status.FAIL : Status.WARN)
                        .problem(
                                () ->
                                        jetson.timestampsLookWrong(c.name())
                                                ? "timestamps far from now (clock sync?)"
                                                : null);
            }
        }
        PoseSource quest = vision.getQuestSource();
        co.add("Quest", "Quest", quest::isConnected)
                .missingIs(() -> quest.isEnabled() ? Status.FAIL : Status.SKIP);
        return co;
    }

    /**
     * Whether the tag layout vision loaded is this event's field.
     *
     * @param loaded the layout vision loaded
     * @param loadedField the field it was loaded as
     * @param expected this event's field ({@link Event#APRILTAG_FIELD})
     * @return what differs, or null when it is the event's layout
     */
    static String fieldMismatch(
            AprilTagFieldLayout loaded, AprilTagFields loadedField, AprilTagFields expected) {
        if (loaded != null && loaded.equals(AprilTagFieldLayout.loadField(expected))) {
            return null;
        }
        if (loadedField == expected) {
            return "AprilTag layout differs from WPILib's " + expected.name() + ", this event's";
        }
        return "AprilTag layout "
                + loadedField.name()
                + ", but this event's field is "
                + expected.name()
                + " (Event.APRILTAG_FIELD)";
    }

    private static SettingsCheck settings(Swerve swerve, Vision vision) {
        Jetson jetson = vision.getJetson();
        Set<Integer> defaultExcluded =
                Arrays.stream(vision.getConfig().getOrinExcludedTags())
                        .boxed()
                        .collect(Collectors.toCollection(TreeSet::new));
        // Judged once: neither layout changes while the code runs.
        String fieldMismatch =
                fieldMismatch(Vision.getTagLayout(), Vision.TAG_FIELD, Event.APRILTAG_FIELD);
        SettingsCheck settings =
                new SettingsCheck()
                        .info(() -> "field " + Event.APRILTAG_FIELD.name())
                        .add(() -> fieldMismatch, Status.WARN)
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
        return settings;
    }
}
