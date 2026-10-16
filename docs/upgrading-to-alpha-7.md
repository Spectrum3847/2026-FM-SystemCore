# Moving to WPILib 2027 alpha-7

This repo is on alpha-6 on purpose: as of 2026-09-22, PathPlannerLib and PhotonLib have no
alpha-7 builds, and alpha-7 rejects every older vendordep. AdvantageKit (`27.0.0-alpha-5`) and
Phoenix 6 (`26.70.0-alpha-2`) do support alpha-7.

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
