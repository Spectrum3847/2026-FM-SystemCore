# Moving to WPILib 2027 alpha-7

This repo is on alpha-6 on purpose: alpha-7 rejects every older vendordep, and as of 2026-10-29
PhotonLib has no alpha-7 build (latest `v2027.0.0-alpha-2`, for alpha-6). PathPlannerLib
(`2027.0.0-alpha-4.1`), AdvantageKit (`27.0.0-alpha-7`) and Phoenix 6 (`26.70.0-alpha-2`) do.

Staying on alpha-6 costs one thing on FM's newer SystemCore image: the image publishes
`/diagnostics/canbusinfo` as `struct:CanBusInfoEntry[]`, alpha-6's `RobotController.getCANStatus()`
(called by AdvantageKit every loop) wants `double[]`, so NT disables that subscription at boot with
a type-mismatch warning. AdvantageKit's `SystemStats/Network/CAN*/Available` and `InterfaceUp` then
read false on a working bus. Nothing here reads them; our CAN health comes from Phoenix
(`CAN/<bus>/*`).

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
