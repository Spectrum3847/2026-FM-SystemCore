# Moving to WPILib 2027 alpha-7

This repo is on alpha-6 on purpose: alpha-7 rejects every older vendordep, and as of 2026-10-29
PhotonLib has no alpha-7 build (latest `v2027.0.0-alpha-2`, for alpha-6). PathPlannerLib
(`2027.0.0-alpha-4.1`), AdvantageKit (`27.0.0-alpha-7`) and Phoenix 6 (`26.70.0-alpha-2`) do.

Staying on alpha-6 costs two things on FM's SystemCore (image alpha 13). The brownout API, below.
And the image publishes
`/diagnostics/canbusinfo` as `struct:CanBusInfoEntry[]`, alpha-6's `RobotController.getCANStatus()`
(called by AdvantageKit every loop) wants `double[]`, so NT disables that subscription at boot with
a type-mismatch warning. AdvantageKit's `SystemStats/Network/CAN*/Available` and `InterfaceUp` then
read false on a working bus. Nothing here reads them; our CAN health comes from Phoenix
(`CAN/<bus>/*`).

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

## Steps

When the vendors catch up:

1. Install the WPILib `2027.0.0-alpha-7` ISO; reimage the SystemCore to **image 14**; use the alpha-7
   Driver Station.
2. `build.gradle`: GradleRIO `2027.0.0-alpha-7`; `settings.gradle`: the alpha-7 install folder.
3. Replace every vendordep from `wpilibsuite/vendor-json-repo/2027_alpha7`.
4. Phoenix 26.70 wants **26.70.x device firmware** (26.50 accepted any 26.x).
5. Known alpha-7 breaks (from its release notes), roughly in the order they will show up:
   - `Main`: `RobotBase.startRobot(Robot::new)` instead of `Robot.class`.
   - Constants and enum values are ALL_CAPS everywhere (`Rotation2d.kZero` → `ZERO`, …).
   - `SmartDashboard` / `SendableChooser` / `Sendable` replaced by the Telemetry and Tunables APIs
     (`SendableChooser` → `Selectable`). `TuneValue`, `Field2d` use, and the Hub Model Chooser need
     porting.
   - Gamepad face buttons renamed to `faceUp/Down/Left/Right`; axes get a default deadband.
   - AprilTag and CameraServer moved to vendordeps; `AprilTagFields` → the integrated `Fields`.
   - CAN device classes take a `CANPort` enum (see `CanBuses`).
   - Raw integer timestamps are nanoseconds, not microseconds (NT and DataLog still µs on the wire).
   - `Alert` moved to wpiutil; `Preferences` moved to `preferences`.
   - `RobotController.getBrownoutVoltage` removed; `setBrownoutVoltage` → `setBrownoutVoltages`
     (see [Brownouts](#brownouts)).
