# Pre-match checks and event prep

## Pre-match checks

While disabled the robot checks itself and publishes one GO / NO-GO
([PreMatchChecks](../src/main/java/frc/spectrumLib/checks/PreMatchChecks.java); FM's list is
[PreMatch](../src/main/java/frc/robot/PreMatch.java)). The checks only report: they never command
anything or stop the robot from enabling. A FAIL makes it NO-GO and raises a HIGH alert, a WARN a
MEDIUM one; both clear on enable.

| Check | FAIL | WARN |
|---|---|---|
| CAN | a device silent for 0.5 s, or never seen | a device rebooted in the last 10 min |
| Homing | a homing attempt failed (hood timed out) | a soft-limited mechanism lost its position to a reboot |
| CameraHeading | two cameras, or a camera and the seeded heading, more than 15 deg apart (TXMAN1 Q30) | more than 6 deg |
| GyroMount | a gyro's yaw sense is reversed, so the heading turns the wrong way | another gyro mount setting looks wrong (the reason names the setting) |
| Auto | no auto file, no paths, or 0.5 m / 10 deg off the start for 1 s | Do Nothing, no alliance, or the pose not seeded |
| Battery | under 11.8 V at rest for 3 s, or the same battery as last match | under 12.3 V at rest for 3 s |
| Coprocessors | a camera the robot fuses not answering | an unfused one missing, or Orin timestamps off |
| Settings | the Jetson's field map hash differs from the robot's, the Orin practice profile is on with the FMS attached, swerve offsets or soft limits changed since boot, or a hard-stop params file was refused | the Jetson reports no field map, the practice profile is on, excluded tags or dashboard switches differ from their defaults, tuning mode is on, or a hard stop is not from git |

On NetworkTables (Elastic, the pit app): `/AdvantageKit/RealOutputs/PreMatch/Go`, `.../Reason`
(one line), `.../Summary` (one line per check) and `.../Checks/<name>/Status` and `Reason`.
Elastic shows the verdict on the Pre-Match tab and every check's line on the Pre-Match Checks
tab. `python tools/wpilog_summary.py <log>` lists every change of verdict. The same-battery check
needs a battery ID reader, which FM doesn't have, so it is skipped.

The field-map check compares the Jetson against the robot. If `Event.java` names the wrong field,
both use it and the check still passes.

Before each event, set the event's field (AndyMark or Welded) in
[Event.java](../src/main/java/frc/robot/Event.java). Vision solves with it, and the Limelights and the
Jetson must match it. Its name and canonical hash
([FieldMapHash](../src/main/java/frc/spectrumLib/vision/FieldMapHash.java)) are logged inputs
(`Vision/FieldMap/Name`, `Hash`), so a replay re-solves the Orin frames with the match's field even
after `Event` has changed. A log without them (before October 2026) replays with `Event`'s field and
raises a warning that the replay may differ; `Vision/FieldMap/Solving` shows which field a run used.

## Before the event: calibrate and check

- [ ] **CAN wiring** matches `CanBuses` (`USE_CANIVORE` and the [port table](systemcore-port.md#what-changed-for-systemcore)). With a CANivore:
      `canivore-usb` installed on the SystemCore. `SystemCoreCAN<n>/StatusOK` (and
      `CANivore/StatusOK`) in the log, and nonzero `BusUtilization` on every port in use.
- [ ] **Swerve encoder offsets**: FM's 2026 values are in `params/robots/FM2026/swerve.json`
      (defaults in `FM2026`); re-check with the alignment page.
- [x] **Limelight mounts**: pushed from code (`pushLimelightMounts`); positions from CAD, angles solved from Q13 rewinds
- [ ] **Orin camera mounts** (`VisionConfig.orinRobotToCamera`): position and yaw from CAD, pitch
      from the Q17 Jetson video. Check height, pitch and roll against `Vision/Orin/<camera>/Mount/*`
      with 2+ tags in view. See [Orin and the Jetson](localization.md#orin-and-the-jetson).
- [ ] **SystemCore camera mounts** (`VisionConfig.systemCoreCameras`): placeholders.
- [ ] **Rio.java**: add the SystemCore's serial so it selects `FM2026` by identity (it defaults to FM anyway).
- [ ] **Hood homing**: the [bench checklist](shooting.md#hood-homing-drivers-no-more-pushing-the-hood-down).
- [ ] **2026 NI Driver Station** (the event's FMS needs it) and a SystemCore image that matches
      WPILib alpha-6. Run the [controller bench test](controls.md#bench-test-ni-ds-about-5-minutes) on it.
