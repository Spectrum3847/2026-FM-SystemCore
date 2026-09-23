# 2026-FM-SystemCore

FRC Team 3847 Spectrum's 2026 robot **FM** ("Final Machine"), converted to run on **SystemCore**,
for an October 2026 offseason event where it competes as **8515**.

It is also a testbed: FM carries several independent robot-pose sources — Limelights, a Jetson Orin
running PhotonVision, a Meta Quest running QuestNav, and the SystemCore's own cameras — and every one
of them is gated on its own, logged before and after gating, and given its own "shadow" pose track,
so the question *which source do we actually trust, when, and how do we know* can be answered with
logs rather than opinions.

| | |
|---|---|
| WPILib | `2027.0.0-alpha-6` (GradleRIO `org.wpilib.GradleRIO`), Java 25 |
| Logging / replay | AdvantageKit `27.0.0-alpha-4` |
| Motors | Phoenix 6 `26.50.0-alpha-1` (needs 26.x device firmware) |
| Autos | PathPlannerLib `2027.0.0-alpha-3` |
| Orin cameras | PhotonLib `v2027.0.0-alpha-2` |
| QuestNav | hand-written NT4 protobuf client (QuestNavLib has no 2027 build) |
| Team number | 8515 (`.wpilib/wpilib_preferences.json`) |

Why alpha-6 and not alpha-7: PathPlanner and PhotonLib have no alpha-7 builds yet, and alpha-7
breaks every older vendordep. When they ship, see [docs/upgrading-to-alpha-7.md](docs/upgrading-to-alpha-7.md).

## Quick start

Java 25 is required. The JDK that ships with WPILib 2027 works:

```bash
export JAVA_HOME=/c/Users/Public/wpilib/2027_alpha5/jdk
```

Build, format (Spotless, AOSP style) and run the tests:

```bash
./gradlew build
```

Run the simulator with the sim GUI (connect the 2027 Driver Station, or use the GUI's DS controls):

```bash
./gradlew simulateJava
```

Or run the scripted demo drive — no driver needed. It seeds the pose while disabled, re-origins the
QuestNav, then drives a lap of straights, spins and strafes in teleop:

```bash
FM_SIM_SCRIPT=drive ./gradlew simulateJava -PnoSimGui
```

Open AdvantageScope (2027), connect to `localhost` (or open the `logs/akit_*.wpilog` the sim wrote),
and install the FM model from [advantagescope-custom-assets/Robot_FM](advantagescope-custom-assets/Robot_FM).

Deploy to the robot (USB: SystemCore at `172.26.0.1`):

```bash
./gradlew deploy
```

## Where things came from

The 2026 code lives in [Spectrum3847/2026-Spectrum](https://github.com/Spectrum3847/2026-Spectrum).

| Area | Source |
|---|---|
| FM's robot code (subsystems, SuperStructure, bindings, autos, `frc.rebuilt`) | `main` — the competition FM |
| `spectrumLib` (Mechanism, telemetry, CAN budget, battery logger, system load, sims) | `2026-offseason-bot` |
| Swerve logging and robustness, Vision gates and seeding, Robot loop order, CAN/match logging, start-pose checks | `2026-offseason-bot`, adapted to FM |
| FM physical constants (gear ratios, wheel radius, module positions, CAN IDs, encoder offsets, camera mounts) | `main` — the offseason bot is a different drivetrain |
| Not brought over | dye rotor, feeder, turret, launcher tower (the offseason bot's, which stays on a roboRIO) |

## What changed for SystemCore

- **API**: everything moved from `edu.wpi.first.*` to the 2027 `org.wpilib.*` packages
  (`DriverStation` split into `MatchState`/`RobotState`/`DriverStationErrors`, `ChassisSpeeds` →
  `ChassisVelocities`, gamepads → `CommandGamepad` face buttons, test mode → *utility* mode, …).
- **CAN** ([CanBuses.java](src/main/java/frc/spectrumLib/hardware/CanBuses.java)): FM's CANivore is
  kept — it works on SystemCore once CTRE's `canivore-usb` package is installed on it (see
  wpilibsuite/SystemcoreTesting `CTR-Phoenix.md`). The devices that were on the roboRIO's own bus
  (the intake rollers, IDs 5 and 6) now go on **SystemCore CAN port 0**. *Check this against the
  wiring.*
- **Logging**: AdvantageKit. `Telemetry.log(...)` keeps its 2026 API and now records AKit outputs.
  Logs go to a USB stick (`/U/logs`) if one is inserted, else `/home/systemcore/logs`.
- **Replay**: every motor's status signals are AKit inputs ([MotorInputs](src/main/java/frc/spectrumLib/mechanism/MotorInputs.java)),
  the drivetrain's odometry samples are AKit inputs ([SwerveInputs](src/main/java/frc/robot/subsystems/swerve/SwerveInputs.java)),
  every pose source's raw frames are AKit inputs, and the robot pose is computed on the main loop
  from them — so a match log can be replayed through changed code.
- **Swerve**: CTRE's `SwerveDrivetrain` still does control and 250 Hz odometry. Its odometry samples
  are copied off its thread and fused on the main loop by [PoseFusion](src/main/java/frc/spectrumLib/localization/PoseFusion.java);
  CTRE's own estimate is kept aligned for its field-centric requests.
- **Sim**: maple-sim has no 2027 build, so the drivetrain sim is CTRE's. It keeps a perfect *truth*
  pose and feeds the robot a version with 2% wheel-radius error and 0.05°/s gyro drift, so vision
  has something to correct. Cameras are PhotonLib's `VisionSystemSim`.

## The pose-source testbed

Design and rationale: [docs/pose-sources.md](docs/pose-sources.md).

Sources (names are the log paths under `Localization/Sources/`):

| Source | Device | Fuses by default |
|---|---|---|
| `LL-Back/MT1`, `LL-Left/MT1`, `LL-Right/MT1` | FM's Limelights, MegaTag1 | yes, until the seed is confirmed |
| `LL-Back/MT2`, `LL-Left/MT2`, `LL-Right/MT2` | FM's Limelights, MegaTag2 | yes, once the seed is confirmed |
| `SC0/MT1`, `SC0/MT2`, `SC1/…` | cameras plugged into the SystemCore (its built-in Limelight stack) | no |
| `orin-front`, `orin-left`, `orin-right` | Jetson Orin + PhotonVision | no |
| `Quest` | Meta Quest + QuestNav | no |

**Every source runs all the time.** "Fuses" only decides whether its accepted measurements move the
robot's pose. Toggle any of them live from a dashboard at
`/Localization/Sources/<name>/Enabled` (an AdvantageKit dashboard input, so replay knows too).

What gets logged, per source:

| Key | Meaning |
|---|---|
| `Localization/Sources/<name>/*` (inputs) | the raw frames: poses, timestamps, tag counts, distances, ambiguity, tracking — **before** any gate |
| `…/Verdicts` | per frame: the trust tier if accepted, the rejection reason if not |
| `…/RejectionCounts/<reason>` | running count per reason |
| `…/AcceptedPoses`, `…/RejectedPoses` | for plotting on the field |
| `…/ErrorVsFusedMeters`, `…/HeadingErrorVsFusedDeg` | disagreement with the fused pose at each frame's capture time |
| `…/ErrorVsTruthMeters` | sim only: error against the true pose |
| `Localization/Shadow/<name>/Pose` | **what the robot would have believed with only this source** |
| `Localization/FusedPose`, `Localization/OdometryPose` | the robot pose, and wheels+gyro only |

After a match (or a sim run), summarise it:

```bash
python tools/wpilog_summary.py logs/akit_XXXX.wpilog
```

Replay a log through the current code (edit a gate, replay, compare the shadow tracks):

```bash
FM_REPLAY=1 AKIT_LOG_PATH=/path/to/log.wpilog ./gradlew simulateJava -PnoSimGui
```

```bash
python tools/replay_compare.py /path/to/log_sim.wpilog
```

The replay log (`*_sim.wpilog`) holds both `RealOutputs` (what the robot did) and `ReplayOutputs`
(what the edited code would have done).

### What the sim shows today

From the scripted drive (see [docs/pose-sources.md](docs/pose-sources.md#simulation-results) for the table):
odometry alone drifts to ~0.16 m while the fused pose averages ~0.014 m error. MegaTag2's shadow
tracks come out slightly worse than MegaTag1's, because MegaTag2 inherits the simulated gyro drift
through the heading the robot pushes to the camera — exactly the kind of finding the testbed exists
to surface, and worth checking on the real robot.

## Hardware notes (SystemCore bench unit, 2026-09-22)

Deployed to the bench SystemCore (no CAN devices, two cameras):

- Boots in ~11 s with no CAN devices at all; loop period 20.0 ms, ~2 ms of user code, 0% overruns.
- The SystemCore's own cameras connect straight to this program as `limelightsc0/1` — no NT bridge
  is needed when the SystemCore is the robot controller. (Deploying this code replaced the
  SystemCoreVision bridge program on that unit; redeploy that repo to get it back.)
- System CPU read ~83% with two cameras attached; probably the Limelight vision stack. Watch
  `System/CpuPercent`.
- `RobotController.setBrownoutVoltage` fails (HAL -1098) on that image; the code carries on
  without it.

Read the robot's console over the deploy connection (it restarts the program afterwards):

```bash
./gradlew deployrobotLogsystemcore -ProbotLog --info
```

Read its NetworkTables from a laptop:

```bash
./gradlew ntDump -Phost=172.26.0.1 -Pprefix=/AdvantageKit/RealOutputs/Localization
```

## Before the event — CALIBRATE / check

- [ ] **CAN wiring**: CANivore present and `canivore-usb` installed on the SystemCore; intake rollers
      (5, 6) on SystemCore port 0. `CANivore/StatusOK` and `SystemCoreCAN0/StatusOK` in the log.
- [ ] **Swerve encoder offsets**: FM's 2026 values are in `FM2026`; re-check with the alignment page.
- [ ] **Limelight mounts**: FM's 2026 code never pushed mounts from code, so `pushLimelightMounts` is
      off and the cameras' own flash is trusted. Verify, then turn it on.
- [ ] **Orin camera names and mounts** (`VisionConfig.orinCameraNames` / `orinRobotToCamera`) — placeholders.
- [ ] **QuestNav mount** (`VisionConfig.robotToQuest`) — placeholder; check the protocol against the
      headset's QuestNav version ([QuestNavProtocol](src/main/java/frc/robot/subsystems/vision/QuestNavProtocol.java)).
- [ ] **SystemCore camera mounts** (`VisionConfig.systemCoreCameras`) — placeholders.
- [ ] **Rio.java**: add the SystemCore's serial so it selects `FM2026` by identity (it defaults to FM anyway).
- [ ] **Driver Station 2027 alpha** and a SystemCore image that matches WPILib alpha-6.

## Known gaps

- **Offseason mechanism behaviour is not ported, only its infrastructure.** Every FM mechanism runs
  on the offseason `Mechanism` (status-signal rates, per-loop batched reads, CAN budget, follower
  health alerts, 10 Hz diagnostics) and AKit inputs, but FM keeps its own state machines and tuning.
  The offseason intake is different hardware now (extension 3.5:1 over 3.65 rot vs FM's 11.25:1
  over 2.8; a kicker roller FM does not have; a hood at 59.4:1 on a different CAN ID). Candidates to
  bring over with bench time: extension tooth-skip RESYNC, clamp-past-max and hold-when-retracted
  (`b25ec09`, `d3bd875`), current-aware agitate and unjam modes (`5b6cfca`), and the hood's
  stop-at-home-rest fix for the 75 A stall (offseason `Hood.homeRestToleranceDegrees`).

- Offseason Vision's placement-heading vote and two-camera consensus heading correction are not
  ported (they are tied to that bot's camera layout).
- maple-sim's field collisions and `RobotBumpSim` are gone with maple-sim.
- The scripted sim changes the sim Driver Station mid-loop, so replaying a *scripted* run is exact
  except on the loop where the robot enables. Real Driver Station logs do not have this.
- The 2026 `tools/robot-app`, log archiving and other repo tooling were not brought over.
