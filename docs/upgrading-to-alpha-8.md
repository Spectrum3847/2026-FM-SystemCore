# Moving to WPILib 2027 alpha-8

This repo is on alpha-6 on purpose: alpha-7 and later reject every older vendordep, and PhotonLib
has no tagged alpha-7 release. The move happens on the long-lived branch `upgrade/alpha-8`, merged
from `main` as it goes, and lands only when the robot runs it (checklist below).

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

## Code changes

Counted 2026-10-30 across the repo's 210 Java files (`LimelightHelpers.java` excluded).

| Change | Where | Notes |
| --- | --- | --- |
| `Main`: `RobotBase.startRobot(Robot::new)` instead of `Robot.class` | 1 | |
| Constants and enum values ALL_CAPS (`Rotation2d.kZero` → `ZERO`, `k...` enums) | ~150 uses, ~30 files | Mechanical; VS Code's project import does some. |
| `SmartDashboard`, `SendableChooser`, `Sendable`, `Field2d` → Telemetry, Tunables, `Selectable` | 32 uses, 12 files | Real work. `TuneValue` and the Hub Model Chooser too. Alpha-8 has a shim so `Selectable` works with 2026 dashboards (Elastic). |
| `Alert` → wpiutil; `Preferences` → `preferences` | 14 files (no `Preferences`) | Import changes. |
| AprilTag and CameraServer → vendordeps; `AprilTagFields` → the built-in `Fields` | 74 uses, 14 files | Includes `Event.java` and the field map hashed for the Jetson (`FieldMapJson`, `FieldMapHash`): the hash must still match the Jetson's. |
| CAN devices take a `CANPort` enum | `CanBuses` | |
| Gamepad face buttons → `faceUp/Down/Left/Right`; axes get a default deadband | our `Gamepad` wrapper | Re-check driver deadbands: ours would stack on WPILib's. |
| **Raw integer timestamps µs → ns** | `Swerve.java` (`HALUtil.getMonotonicTime() * 1e-6`), `IntakeExtension.java` (`RobotController.getTime() / 1000.0`) | **Highest risk.** Wrong units silently shift every odometry sample and break pose fusion. Check each call's units in the alpha-8 Javadoc, then prove it with replay. |
| Brownout API | `Robot` | See [Brownouts](#brownouts). |
| Vendor API changes | PhotonLib 6 files, PathPlanner 3, Phoenix 25, AdvantageKit 41 | Unknown until it compiles. |
| `LimelightHelpers.java` | Limelight's file | No 2027 version; Limelights aren't in the 2027 plan. Keep if it compiles, else drop with the Limelight sources. |

Alpha-8 itself (over alpha-7) removes `Talon` and `VictorSP` (we only mention them in comments),
renames more "speeds" to "velocities", and simplifies WPILib's trajectory classes. If
PathPlannerLib alpha-4.1 calls one of those it fails at run time, not compile time; the `auto` sim
and `simTest` would catch it. Fallback: stay on alpha-7, which image 18 also runs.

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

After the upgrade: keep the default or set it explicitly with `setBrownoutVoltages`, never below
6 V; drop `getBrownoutVoltage` from the availability probe and the `SystemStats/BrownoutVoltage`
log in `Robot`; log `isBrownedOut()` every loop beside the fault count.

The upgrade also fixes a mismatch alpha-6 has on FM's image: the image publishes
`/diagnostics/canbusinfo` as `struct:CanBusInfoEntry[]`, alpha-6's `RobotController.getCANStatus()`
(called by AdvantageKit every loop) wants `double[]`, so NT disables that subscription at boot and
AdvantageKit's `SystemStats/Network/CAN*/Available` and `InterfaceUp` read false on a working bus.
Nothing here reads them; our CAN health comes from Phoenix (`CAN/<bus>/*`).

## Checklist

On `upgrade/alpha-8`. Each step ends with `python tools/check.py` passing unless it says otherwise.

**Code (no robot needed)**

- [ ] Install the WPILib `2027.0.0-alpha-8` ISO. `build.gradle`: GradleRIO `2027.0.0-alpha-8`;
      `settings.gradle`: the alpha-8 install folder. Main: `Robot::new`.
- [ ] Vendordeps: AdvantageKit 27.0.0-alpha-7, PathPlannerLib 2027.0.0-alpha-4.1, Phoenix 6
      26.70.0-alpha-2, the alpha-8 CommandsV2, and PhotonLib from the dev build the Jetson uses
      (add `maven.photonvision.org/snapshots` if the vendordep doesn't).
- [ ] Get it compiling, in the table's order. Keep each kind of change its own commit.
- [ ] Timestamps: settle both conversions from the alpha-8 Javadoc, before anything else runs.
- [ ] `python tools/check.py --sims drive,shoot,auto` (with `simTest`) passes.
- [ ] Replay a real match log (`--log`, and `tools/replay_regression.py` against `main`): fused
      pose and shot gate unchanged except where the code meant to change.
- [ ] Brownout: the alpha-7 API, per [Brownouts](#brownouts).
- [ ] Log key guide regenerated; README's version table and this doc updated.

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
- [ ] Merge `upgrade/alpha-8` to `main` through its PR.

**Way back:** reflash image alpha 13, reinstall `canivore-usb` 1.16 and the current `photonvision-sc`,
deploy `main`. Device firmware 26.70 should still run Phoenix 26.50 code (26.50 took any 26.x).
