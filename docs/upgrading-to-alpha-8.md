# Moving to WPILib 2027 alpha-8

`main` is on alpha-6. The long-lived branch `upgrade/alpha-8` (PR #139) compiles and runs on
alpha-8: the build, every unit test and the drive, shoot and auto sims pass. It lands on `main`
only when the robot runs it ([checklist](#checklist)).

## Target versions

Checked 2026-10-30.

| Piece | Now | Target | Notes |
| --- | --- | --- | --- |
| WPILib / GradleRIO | 2027.0.0-alpha-6 | **2027.0.0-alpha-8** | Released 2026-10-30. Takes alpha-7 vendordeps (except PathPlanner C++). Needs SystemCore image 14+ and the alpha-7 Driver Station. |
| SystemCore image | alpha 13 | **alpha 18** (2026-10-28) | FM is on the "alpha" hardware line. Image 14+ requires WPILib alpha-7 or later. |
| PhotonLib | v2027.0.0-alpha-2 | **dev build** from PhotonVision `main` | No tagged alpha-7 release; `main` builds against alpha-7. Dev builds are on `maven.photonvision.org/snapshots` (latest checked: `dev-v2027.0.0-alpha-2-88-g14cc2298`). The robot's PhotonLib and the Jetson's PhotonVision must come from the same commit. |
| PathPlannerLib | 2027.0.0-alpha-3 | **2027.0.0-alpha-4.1** | Built for alpha-7 (its `2027` branch). No alpha-8 build yet; WPILib says Java works on alpha-8. |
| AdvantageKit | 27.0.0-alpha-4 | **27.0.0-alpha-7** | Includes `LoggedDashboardChooser`. |
| Phoenix 6 | 26.50.0-alpha-1 | **26.70.0-alpha-2** | Every CTRE device needs **26.70.x firmware** (26.50 took any 26.x). |
| CommandsV2 | 1.0.0 | the copy in the alpha-8 install | |

Alpha-7 and alpha-8 vendordeps come from `wpilibsuite/vendor-json-repo/2027_alpha7`. Nothing else
in `build.gradle` depends on the WPILib version (Jackson, Error Prone, JUnit).

## Blocked on

- **PhotonVision on the Jetson** moving to the alpha-7 PhotonVision (SpectrumJetson, in progress).
  The robot can't move first: PhotonLib and PhotonVision have to match.
- **A PhotonVision build for the SystemCore backup camera** (`photonvision-sc`) from the same commit.

## What changed

Each item is on the branch. The ones marked **robot** still need checking on FM.

- **Packaging.** GradleRIO alpha-8 dropped the fat jar. `build.gradle` uses Gradle's `application`
  plugin (`deployArtifact.configureApplication`, `wpi.java.configureApplication`): the robot jar
  plus one jar per library. Locally, `./gradlew installDist` puts them in `build/install/`;
  `tools/check.py` and CI run that. `debugJni` moved inside the wpilibJava artifact.
- **Vendordeps.** Alpha-7 ones, plus WPILib's own `CameraServer.json` and `AprilTag.json`: in 2027
  they are vendordeps, and PhotonLib's camera sim needs them (OpenCV).
- **Nanoseconds.** `RobotController.getTime()` and `HALUtil.getMonotonicTime()` return ns, and so
  does PhotonLib's capture time. Fixed in `Swerve` (odometry time offset) and `IntakeExtension`.
  The sims' fused-pose error is unchanged, which would not survive a 1000x mistake.
- **Log time.** AdvantageKit 27.0.0-alpha-7 logs `/Timestamp` in ns and marks it with
  `"unit":"nanoseconds"`; WPILOG record times stay µs. `wpilog_summary.py`, `trim_wpilog.py` and
  `ci_assert.py` read the mark, so they handle both.
- **Old logs don't replay.** AdvantageKit alpha-7 refuses any log without the mark ("incompatible
  timestamp unit"). CI's replay job is off on this branch until it has alpha-8 robot logs as
  fixtures. `TagField` still reads the alpha-6 field names, for when it does.
- **Alerts.** WPILib's `Alert` takes an id, unique within its group. Our `Alert` makes one per
  alert, since many share text.
- **SmartDashboard is gone.** Field2d, the scheduler and the Limelight views go through
  `org.wpilib.telemetry.Telemetry.log`, under the same `SmartDashboard/` paths, so Elastic's layout
  is unchanged. **Robot:** check that Elastic draws them.
- **Choosers.** `LoggedDashboardChooser` became `LoggedNetworkChooser`, same keys. A dashboard
  picks an option by writing `<key>/selected/tune` (the `auto` sim does).
- **AprilTag fields** are `org.wpilib.fields.Field` and `Fields`. The field map sent to the Jetson
  is WPILib 2027 Field JSON (`field-tags`, `field-dimensions`). Its hash is unchanged
  (AndyMark `2792cd4e868967b8`, welded `ca28a7c1139ed099`). **Jetson:** must parse the new format.
- **Bad Orin packets.** PhotonLib now decodes zeros past the end of a short packet instead of
  throwing, so a garbled result decoded as a real one. `OrinSolver` re-encodes each result and
  drops it (`DecodeFailures`) unless the bytes match.
- **CAN.** `CANBus(CANPort.CAN_Sn)` for SystemCore buses; swerve uses `withNetwork`.
- **Gamepads.** Face buttons are `FACE_DOWN/RIGHT/LEFT/UP`. **Robot:** alpha-8 needs the alpha-7
  Driver Station, not the NI one, so check every control on the WPILib DS before driving
  ([Controllers](../README.md#controllers-and-the-driver-station)).
- **Brownout.** `getBrownoutVoltage` is gone. `Robot` reads only `isBrownedOut()` and keeps the
  default voltages; `SystemStats/BrownoutVoltage` is no longer logged.
- **Smaller ones.** `RobotBase.startRobot(Robot::new)`, constants ALL_CAPS (`Rotation2d.ZERO`),
  `Translation2d.getAngle()` returns an `Optional`, `Rectangle2d` moved to `math.shape`.
- **Hand-stepped tests.** WPILib reports the robot mode as UNKNOWN until
  `DriverStationBackend.observeUserProgramStarting()`, which `startCompetition` calls. `SimRobot`
  calls it too, or `RobotModeTriggers.teleop()` never goes true.

PhotonLib is upstream's `dev-v2027.0.0-alpha-2-88-g14cc2298`, which resolves for CI. The team's
alpha-8 build (`...-94-g0859c272`) has the same photon-serde, so either talks to a Jetson built
from serde `14cc2298`.

## Brownouts

SystemCore browns out in hardware, independent of WPILib: below 6.75 V it zeros the digital
outputs, sets PWM outputs to neutral and blinks the red LED fast, and it recovers above 7.5 V
(Limelight's release notes, image 4 on). It reports brownouts in its fault table, which AdvantageKit
logs as `SystemStats/Faults/Brownout` and `FaultCounts/Brownout`; `MatchSummary` counts those.

- **On alpha-6 the brownout API is a stub.** `setBrownoutVoltage`, `getBrownoutVoltage` and
  `isBrownedOut` all fail with HAL -1098 on SystemCore, so FM has always run at the default above
  (the old 6.0 V request never applied) and `SystemStats/BrownedOut` isn't logged.
- **Alpha-7 implements it** ([allwpilib#9238](https://github.com/wpilibsuite/allwpilib/pull/9238)):
  `isBrownedOut()` reads the real state, and `setBrownoutVoltages(brownout, recovery)` replaces the
  setter: brownout 5-8 V, recovery at most 8.5 V and at least 0.5 V above it, anything else
  rejected ([allwpilib#9227](https://github.com/wpilibsuite/allwpilib/issues/9227), which cites
  batteries killed by low roboRIO settings). `getBrownoutVoltage()` is gone.
- **Open: does the fault table see a brownout?** TXMAN1 Q6 logged `SystemStats/BatteryVoltage`
  under 6.75 V about ten times (down to 6.53 V), yet `Faults/Brownout` never went true and
  `FaultCounts/Brownout` stayed 0. Short dips may be filtered, or the trip may use another
  measurement. Bench test with an adjustable supply: brownout trips at the set voltage, shows in the
  fault table and in `isBrownedOut()`, and the motors stop.
- **Open: CAN motor controllers.** The roboRIO sends them an explicit disable in a brownout;
  Limelight's notes for SystemCore name only digital and PWM outputs. Check in the same bench test.

On the branch: the default voltages, `getBrownoutVoltage` and its log gone. Still to do: log
`isBrownedOut()` every loop beside the fault count. If the default ever changes, use
`setBrownoutVoltages`, never below 6 V.

The upgrade also fixes a mismatch alpha-6 has on FM's image: the image publishes
`/diagnostics/canbusinfo` as `struct:CanBusInfoEntry[]`, alpha-6's `RobotController.getCANStatus()`
(called by AdvantageKit every loop) wants `double[]`, so NT disables that subscription at boot and
AdvantageKit's `SystemStats/Network/CAN*/Available` and `InterfaceUp` read false on a working bus.
Nothing here reads them; our CAN health comes from Phoenix (`CAN/<bus>/*`).

## Checklist

On `upgrade/alpha-8`. Each step ends with `python tools/check.py` passing unless it says otherwise.

**Code (no robot needed)**

- [x] GradleRIO `2027.0.0-alpha-8`, `settings.gradle` on the alpha-8 install, `Robot::new`.
- [x] Vendordeps: AdvantageKit 27.0.0-alpha-7, PathPlannerLib 2027.0.0-alpha-4.1, Phoenix 6
      26.70.0-alpha-2, CommandsV2 from alpha-8, PhotonLib dev build, CameraServer, AprilTag.
- [x] Compiles; timestamps settled.
- [x] `python tools/check.py --sims drive,shoot,auto` (with `simTest`) passes.
- [ ] Replay a real match log: blocked until there is an alpha-8 log (old logs don't replay).
- [ ] Brownout: log `isBrownedOut()` every loop.
- [x] Log key guide regenerated; README's version table and this doc updated.

**Robot**

- [ ] CTRE devices to 26.70.x firmware (Phoenix Tuner X).
- [ ] Reflash the SystemCore to image alpha 18. Then: static IP 10.85.15.2; CANivore driver from
      CTRE's packages for image 15+ (FM's `canivore-usb` 1.16 is for image 13); reinstall
      `photonvision-sc` built from the matching PhotonVision. The next deploy turns Limelight's vision
      back off (`tools/systemcore_services.sh`).
- [ ] Jetson on the matching PhotonVision build.
- [ ] Alpha-7 (or later) Driver Station on the DS laptop.
- [ ] Deploy, then [bench-checks.md](bench-checks.md): every-deploy checks, CAN, Jetson and cameras,
      pre-match GO.
- [ ] Brownout bench test with an adjustable supply.
- [ ] Record a few practice logs, make them CI's replay fixtures, and turn the replay job back on
      (`.github/workflows/replay.yml`).
- [ ] Merge `upgrade/alpha-8` to `main` through its PR.

**Way back:** reflash image alpha 13, reinstall `canivore-usb` 1.16 and the current `photonvision-sc`,
deploy `main`. Device firmware 26.70 should still run Phoenix 26.50 code (26.50 took any 26.x).
