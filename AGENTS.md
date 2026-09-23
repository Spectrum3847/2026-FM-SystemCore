# Agent notes

FM (Spectrum 3847's 2026 robot) on SystemCore, competing as 8515. See README.md first, then
docs/pose-sources.md for the localization testbed.

## Build

- Java 25: `export JAVA_HOME=/c/Users/Public/wpilib/2027_alpha5/jdk` (the WPILib 2027 JDK).
- `./gradlew build` compiles, applies Spotless (Google Java Format AOSP, 4 spaces) and runs tests.
- `FM_SIM_SCRIPT=drive ./gradlew simulateJava -PnoSimGui` runs the scripted sim; then
  `python tools/wpilog_summary.py logs/<log>.wpilog`.
- The WPILib API is 2027 alpha-6 (`org.wpilib.*`). Do not guess 2026 names: check the jars
  (`javap`) or the existing code. Common moves: `DriverStation` → `MatchState` / `RobotState` /
  `DriverStationErrors`; `ChassisSpeeds` → `ChassisVelocities`; `Timer.getFPGATimestamp` →
  `Timer.getTimestamp`; test mode → utility mode.

## AdvantageKit rules

- `Logger`/`Telemetry` is main-thread only. Never log from CTRE's odometry thread, a Notifier, or a
  vision callback.
- Logic must read logged inputs (`MotorInputs`, `SwerveInputs`, `PoseSourceInputs`,
  `LoggedNetwork*`), never outputs, or replay diverges.
- The robot clock (`Timer`, `RobotController.getTime`) is pinned per cycle and frozen during init.
  Anything that measures elapsed wall time (timers, CAN config budgets) must use `System.nanoTime`.

## Conventions

- Hardware constants are FM's from 2026-Spectrum `main`; do not copy the offseason bot's.
- Vision thresholds carry their 2026 rationale in comments; change them only with log evidence,
  and prove it with replay (`tools/replay_compare.py`).
- One feature per PR.
