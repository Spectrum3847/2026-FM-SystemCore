# Porting FM to SystemCore

## Where things came from

The 2026 code lives in [Spectrum3847/2026-Spectrum](https://github.com/Spectrum3847/2026-Spectrum).

| Area | Source |
|---|---|
| FM's robot code (subsystems, SuperStructure, bindings, autos, `frc.rebuilt`) | `main` (the competition FM) |
| `spectrumLib` (Mechanism, telemetry, CAN budget, battery logger, system load, sims) | `2026-offseason-bot` |
| Swerve logging and robustness, Vision gates and seeding, Robot loop order, CAN/match logging, start-pose checks | `2026-offseason-bot`, adapted to FM |
| FM physical constants (gear ratios, wheel radius, module positions, CAN IDs, encoder offsets, camera mounts) | `main`, because the offseason bot is a different drivetrain |
| Not brought over | dye rotor, feeder, turret, launcher tower (the offseason bot's, which stays on a roboRIO) |

## What changed for SystemCore

- **API**: everything moved from `edu.wpi.first.*` to the 2027 `org.wpilib.*` packages
  (`DriverStation` split into `MatchState`/`RobotState`/`DriverStationErrors`, `ChassisSpeeds` →
  `ChassisVelocities`, test mode → *utility* mode, …). Gamepads read raw indices through a
  Driver Station layout table, see [Controllers](controls.md#controllers-and-the-driver-station).
- **CAN** ([CanBuses.java](../src/main/java/frc/spectrumLib/hardware/CanBuses.java)): one switch,
  `CanBuses.USE_CANIVORE`, picks the wiring. **FM runs with it `true`: drivetrain, shooter and mechanisms on the
  CANivore, only the intake rollers on `systemcore:0`.**

  | Port | Devices | With `USE_CANIVORE = true` |
  |---|---|---|
  | `systemcore:0` | intake rollers (5, 6) | same |
  | `systemcore:1` | swerve: 8 TalonFX, 4 CANcoders, Pigeon (its own bus for 250 Hz odometry) | CANivore |
  | `systemcore:2` | launcher (46–49), hood (15) | CANivore |
  | `systemcore:3` | intake extension (4, 5), indexer bed (8, 9), indexer tower (51, 52), CANdle (1) | CANivore |
  | `systemcore:4` | spare | |

  Bus health is logged as `SystemCoreCAN<n>/…` (and `CANivore/…` with it on). After flipping the
  switch, rerun `python tools/make_elastic_layout.py`; the dashboard's main-bus widgets follow it.
  A CANivore on SystemCore needs CTRE's `canivore-usb` package (wpilibsuite/SystemcoreTesting
  `CTR-Phoenix.md`). *Check the port assignment against the wiring.*
- **Logging**: AdvantageKit. `Telemetry.log(...)` keeps its 2026 API and records AKit outputs.
  Logs go to a USB stick (`/U/logs`) if one is inserted, else `/home/systemcore/logs`.
- **Replay**: every motor's status signals are AKit inputs ([MotorInputs](../src/main/java/frc/spectrumLib/mechanism/MotorInputs.java)),
  the drivetrain's odometry samples are AKit inputs ([SwerveInputs](../src/main/java/frc/robot/subsystems/swerve/SwerveInputs.java)),
  every pose source's raw frames are AKit inputs, and the robot pose is computed on the main loop
  from them, so a match log can be replayed through changed code.
- **Swerve**: CTRE's `SwerveDrivetrain` still does control and 250 Hz odometry. Its odometry samples
  are copied off its thread and fused on the main loop by [PoseFusion](../src/main/java/frc/spectrumLib/localization/PoseFusion.java);
  CTRE's own estimate is kept aligned for its field-centric requests.
- **Sim**: maple-sim has no 2027 build, so the drivetrain sim is CTRE's. It keeps a perfect *truth*
  pose and feeds the robot a version with 2% wheel-radius error and 0.05°/s gyro drift, so vision
  has something to correct. Cameras are PhotonLib's `VisionSystemSim`.

## Known gaps

- **Offseason mechanism behaviour is not ported, only its infrastructure.** Every FM mechanism runs
  on the offseason `Mechanism` (status-signal rates, per-loop batched reads, CAN budget, follower
  health alerts, 10 Hz diagnostics) and AKit inputs, but FM keeps its own state machines and tuning.
  The offseason intake is different hardware (extension 3.5:1 over 3.65 rot vs FM's 11.25:1
  over 2.8; a kicker roller FM does not have; a hood at 59.4:1 on a different CAN ID). Candidates to
  bring over with bench time: extension tooth-skip RESYNC, clamp-past-max and hold-when-retracted
  (`b25ec09`, `d3bd875`), current-aware agitate and unjam modes (`5b6cfca`), and the hood's
  stop-at-home-rest fix for the 75 A stall (offseason `Hood.homeRestToleranceDegrees`; FM now rests
  on the stop in brake after each stow re-home, which covers the same case).

- Offseason Vision's placement-heading vote and two-camera consensus heading correction are not
  ported (they are tied to that bot's camera layout).
- maple-sim has no 2027 build, so there are no field collisions and `RobotBumpSim` is unused.
- The scripted sim changes the sim Driver Station mid-loop, so replaying a *scripted* run is exact
  except on the loop where the robot enables. Real Driver Station logs do not have this.
- Log archiving and the other 2026 repo tooling were not brought over. The robot app was (see
  [Robot app](robot-app.md)), without the parts that edited Java or synced logs.
