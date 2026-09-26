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

Or run one of FM's PathPlanner autos (any name from the Auto Chooser), e.g.:

```bash
FM_SIM_SCRIPT="auto:TBTB Left" ./gradlew simulateJava -PnoSimGui
```

Or the scripted shooting run: aim and launch standing, then aim and launch while strafing, with
fuel put in the hopper before each launch (it exercises everything under [Shooting](#shooting)):

```bash
FM_SIM_SCRIPT=shoot ./gradlew simulateJava -PnoSimGui
```

Add `FM_SIM_EXIT=1` to any scripted run to have the program close its log and exit when the script
ends, so the run is one command.

All 32 autos and 26 paths from 2026 `main` are in `src/main/deploy/pathplanner`, unchanged. The Auto
Chooser is now an AdvantageKit `LoggedDashboardChooser` (same dashboard key), so the selected auto
is logged and replays.

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
- **CAN** ([CanBuses.java](src/main/java/frc/spectrumLib/hardware/CanBuses.java)): one switch,
  `CanBuses.USE_CANIVORE`, picks the wiring. **This branch sets it to `false`: no CANivore, only the
  SystemCore's native CAN FD ports.**

  | Port | Devices | With `USE_CANIVORE = true` |
  |---|---|---|
  | `systemcore:0` | intake rollers (5, 6) | same |
  | `systemcore:1` | swerve: 8 TalonFX, 4 CANcoders, Pigeon (its own bus for 250 Hz odometry) | CANivore |
  | `systemcore:2` | launcher (46–49), hood (15) | CANivore |
  | `systemcore:3` | intake extension (4, 5), indexer bed (8, 9), indexer tower (51, 52), CANdle (1) | CANivore |
  | `systemcore:4` | spare | — |

  Bus health is logged as `SystemCoreCAN<n>/…` (and `CANivore/…` with it on). After flipping the
  switch, rerun `python tools/make_elastic_layout.py`; the dashboard's main-bus widgets follow it.
  A CANivore on SystemCore needs CTRE's `canivore-usb` package (wpilibsuite/SystemcoreTesting
  `CTR-Phoenix.md`). *Check the port assignment against the wiring.*
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

## Shooting

### Aim feedforward

While aiming (`PILOT_AIM_AT_TARGET`: pilot X, every RT launch, and every auto `launch()`), the
heading request now gets a feedforward: how fast the bearing to the hub is turning, `-v_tangential
/ distance` (581's form), in rad/s counter-clockwise, clamped to +/-2 rad/s (2910's clamp). The
heading PID alone only turns once it has fallen behind. In the `shoot` sim, strafing past the hub
at about 1 m/s from 2.5 m, the heading error went from a steady 4-5 deg to about 1 deg.

`ShotCalculator` used to differentiate its own drive angle for this, in rotations per second (2 pi
too small; nothing read it). Fixed to rad/s and used, that made the aim oscillate (6-9 deg mean
error): it differentiates the shoot-on-move yaw offset too, and that follows the measured velocity,
which the turning itself disturbs. Logged as `Swerve/Aim/TargetRateFeedforward` (deg/s) and
`Swerve/Aim/HeadingErrorDeg`.

### Shot readiness gate (drivers: this changes how RT feels)

**FM on 2026 `main` fed the moment RT was pulled**, whether or not the flywheel was up to speed,
the hood had arrived or the robot had finished turning to the hub. Now the indexers hold the fuel
(tower stopped, bed slow-indexing as when intaking) until the shot is ready, then feed at full
speed. Drivers should practise with it: the first ball can come a few tenths later than they are
used to, and pre-spinning with X (track target) before RT takes most of that away. Logic in
[ShotGate](src/main/java/frc/robot/subsystems/ShotGate.java), from the offseason bot's feed gate
(dcb88dc, 067af02, ddd564a) with the drivetrain's heading in place of its turret.

| | Start a volley | Keep feeding |
|---|---|---|
| Flywheel | within 150 RPM of target (`Launcher.onTargetToleranceRPM`) | above 75% of target |
| Hood | within 1 deg, and moving under 20 deg/s | within 3 deg |
| Heading (while the drivetrain aims) | within `atan((hub radius - ball) / distance)` (4499), 2-10 deg; 10 deg for feed shots (9470) | twice the start tolerance |
| Range | distance inside the model's fit, only while vision is under 3 s old | not checked |
| All of it | held for 0.04 s | |

- **Timeout**: held for 1.0 s in a launch state, it feeds anyway for the rest of that launch, so a
  stuck sensor or an unreachable target can delay a shot by a second but never stop one. This is
  what keeps autos safe (their `launch()` window is 2.5 s).
- **Bypass**: **operator Y** (without LB; LB+Y is still the intake reset) feeds while held.
- Leaving the launch state resets it: the next volley re-earns the start window.

On the dashboard (Match and Shooting tabs): `Shot/Ready` (also lit while tracking with X, before
RT) and `Shot/BlockedReason`: `None`, `NoShot`, `FlywheelNotReady`, `HoodNotReady`, `NotAimed`,
`InvalidShot` or `Settling`. Also logged: `Shot/Feeding`, `Shot/FeedReason`
(`Ready`/`Override`/`Timeout`/`Held`), the errors against each target, and running counts.
`tools/wpilog_summary.py` prints each launch, how long it was held and by what.

In the sim the flywheel runs 124 RPM over target (kS 20 A against no friction, kP 10 A/rps), which
is why the flywheel tolerance is 150 and not the unused 100; the hood overshoots on Motion Magic,
which is why its speed is checked. **CALIBRATE all of it from the first practice log**
(`Shot/FlywheelErrorRPM`, `Shot/HoodErrorDeg`, `Shot/HeadingErrorDeg`, `Shot/SecondsHeld`).

### Set shots (fixed shots for when the pose is gone)

From the offseason bot (463c465, fa1376b), without its turret and without its hub-face shot (FM
cannot make it). Park at a known spot, point the **back** of the robot (FM's launcher side) at the
hub, and hold the chord: the hood and flywheel go to FM's own hub model evaluated at that spot's
range standing still, with the operator's hood trim, and the gate feeds once flywheel and hood are
there. Nothing reads the pose and the aim is not checked: as on the offseason bot, **the driver
aims by parking**, with translation slowed to the launch states' 10%. Everything is in
`ShotCalculator.SetShot` (spots per alliance from `Field`, rotated about the field centre for red).

| Chord (pilot, teleop) | Spot (blue; red is rotated) | Heading | Range | Hood / RPM (no-ceiling model, trim -2) |
|---|---|---|---|---|
| **LB + A** | Tower: intake against the tower's field-facing wall on its centreline, (1.525, 3.746) m | 185.3 deg: square to the wall, then about 5 deg counter-clockwise | 3.11 m | 14.7 deg / 2074 RPM |
| **LB + X** | Left trench: in the lane, just clear of the trench on the alliance side, (3.609, 7.430) m | 106.7 deg | 3.54 m | 16.4 deg / 2129 RPM |
| **LB + B** | Right trench: the mirror, (3.609, 0.639) m | -106.7 deg | 3.54 m | 16.4 deg / 2129 RPM |

Hood and RPM follow the Hub Model chooser and the hood trim live; the table is the default model.
`Shot/SetShotHeadingErrorDeg` logs the fused heading against the spot's (information only).
**CALIBRATE** before relying on them: FM's bumper-to-centre (`FM_HALF_LENGTH_METERS`, 0.42 m from
PathPlanner's 0.84 m frame), and where the tower's centreline really is (`TOWER_CENTRE_Y_METERS`:
the offseason code says it follows tag 31, FM's `Field` says the field centreline; the range barely
changes, the heading does). In the `shoot` sim: tower 5 of 8, each trench 7-8 of 8.

### Shot log

One row per volley under `ShotLog/`, written on the loop the gate opens the feed: distance, radial
and tangential velocity, hood and RPM target and actual, heading error and tolerance, the gate's
reason (and what an override or timeout bypassed), how long it waited, pose trust
(`SecondsSinceFusedEstimate`, seed confirmed), set shot or calculated, alliance, mode and match
time; an end row gives the volley's length and its flywheel dips (roughly, balls; calibrate first).
From the offseason bot's shot record (eb2d86e). Schema: [docs/shot-log.md](docs/shot-log.md).

```bash
python tools/shot_log.py logs/akit_XXXX.wpilog > shots.csv
```

### Bindings

Pilot (port 0):

| Input | Does |
|---|---|
| RT | Launch (squeeze after 1 s); RT + LT launch without squeeze; LT released with RT held, launch with no delay. All behind the shot gate |
| LT | Intake; LT + LB eject |
| X (without LB) | Track target: aim and spin up, no feed |
| A (without LB) | Unjam |
| **LB + A / LB + X / LB + B** | **Set shot: tower / left trench / right trench** |
| Select | Force home |
| LB + D-pad | Reorient forward / left / back / right |
| A / B, disabled | Coast / brake the intake extension and hood |

Operator (port 1):

| Input | Does |
|---|---|
| **Y (without LB)** | **Hold to feed regardless of the shot gate** |
| LB + X | Re-origin the QuestNav on the robot pose |
| LB + Y | Intake extension: reset position to max |
| Select | Force home |
| D-pad up / down | Hood trim +/- 0.1 deg |
| D-pad left / right | Aim trim +/- 1 deg |
| A / B, disabled | Coast / brake |

Release order on the chords: let go of the face button first (or both together). Letting go of LB
first with X or A still held starts track target or unjam for the rest of the press, as on the
offseason bot.

## Loop rate, logging and the dashboard

- **100 Hz.** `Constants.LOOP_PERIOD_SECONDS = 0.01` (FM ran 50 Hz on the roboRIO). Anything that
  was "every N loops" is now `RobotLoop.everySeconds(...)`, so slow-tier logging stays at 10 Hz / 1 Hz
  whatever the period.
- **The main thread is real-time** (`Constants.MAIN_THREAD_RT_PRIORITY = 1`, set at the end of
  robot init, real robot only). SystemCore's cores are ~85% busy with its own camera servers and
  services, and a normal-priority loop waits behind them. Bench unit, 2026-09-23, 1 s samples: the
  worst loop per sample went from 11–46 ms to about 10.2 ms (one sample in 20 hit 15 ms), and late
  loops (over 12.5 ms) from 1.7% to 0.1%. System CPU stays ~88%.
  - **Why 1 and not higher:** Phoenix's CAN threads run at real-time 1–3 on SystemCore, and 1 is
    the lowest real-time level. The 2026 offseason bot ran its main thread at 99 on the roboRIO
    and starved Phoenix's frame dispatch ("CAN message is stale", `WaitForAll -1003`), and that
    was reverted (2026-Spectrum `704030d`). Priority 15 measured the same here as 1, so there is
    nothing to gain by outranking Phoenix.
  - **Before the event:** with the swerve powered, watch for stale-signal warnings and check
    `Swerve/FailedDaqs`.
  - **Inheritance:** threads the main thread starts afterwards inherit the priority (AdvantageKit's
    radio logger does). `System/TopThreads` marks real-time threads `[rt N]`, and
    `./gradlew deployrobotLogsystemcore -ProbotLog -ProbotThreads --info` lists them all.
  - **To turn it off:** set the constant to 0.
- **The log gets everything; NetworkTables gets the dashboard.** [DashboardReceiver](src/main/java/frc/spectrumLib/telemetry/DashboardReceiver.java)
  sits in front of AdvantageKit's NT publisher and passes only dashboard keys (`Telemetry.logDash`
  keys, plus every key [elastic-layout.json](src/main/deploy/elastic-layout.json) reads) at 50 Hz (every 20 ms, as before). The sim mirrors everything by default.
  This is the offseason rule (it stopped mirroring the whole log to NT because of the CPU cost). Skipped
  cycles are merged, not dropped, so a value logged once a second still reaches the dashboard.
- **`/SmartDashboard/Telemetry/MirrorLogsToNT`** (Diagnostics tab) sends *every* output to NT for a
  live AdvantageScope session on the bench. It is ignored with the FMS attached. Measured on the bench
  unit: 154 topics become 966, and AdvantageKit's receiver thread goes from ~11% to ~14% of one core.
  The main loop doesn't move, because the publisher runs off it.
- **Alerts** are logged (`Alerts/Errors|Warnings|Infos`) and republished to `/SmartDashboard/Alerts`
  for Elastic by [Alert](src/main/java/frc/spectrumLib/telemetry/Alert.java). WPILib alpha-6 no longer
  does either. Use `frc.spectrumLib.telemetry.Alert`, not WPILib's.
- **Elastic layout**: generated by [tools/make_elastic_layout.py](tools/make_elastic_layout.py). It
  has six tabs (Pre-Match, Match, Shooting, Localization, Power, Diagnostics) on a **14 × 6 grid
  (1792 × 768 px)**, so nothing sits behind the Driver Station window. The generator asserts every
  widget is in-grid and that none overlap. Edit the script, not the JSON:

  ```bash
  python tools/make_elastic_layout.py
  ```

## Hardware notes (SystemCore bench unit, 2026-09-22)

Deployed to the bench SystemCore (no CAN devices, two cameras):

- Boots in ~11 s with the CANivore layout and no CANivore: the missing CANivore spends the CAN
  config budget at once. The native layout on an empty bench takes ~58 s, most of it (~26 s) in the
  swerve constructor timing out on devices that aren't there. With the devices wired, this should
  be much shorter.
- At 100 Hz: `robotPeriodic` is ~1 ms and the mean period is 10.0–10.7 ms. 2–6% of loops run
  over 12.5 ms, and the worst are 20–45 ms.
- **Why it jitters on the bench: the controller is out of CPU.** It reads 80–90% busy, from two causes:
  - The SystemCore's own Limelight vision servers for its two cameras use ~55% of the machine.
  - **With `USE_CANIVORE = true` and no CANivore plugged in, one Phoenix native thread spins a
    full core.** It starts as the first device on the `*` bus is constructed. With the native
    layout nothing spins: the program uses ~18% of the machine instead of ~35%, and the
    busiest thread is the main loop at ~25% of one core. Not yet checked: whether the spin also
    happens with a real CANivore attached. **Check `System/TopThreads` on the real robot.**
  - Native layout, bench unit: mean period 10.1–10.3 ms, 1–5% of loops over 12.5 ms, worst
    14–30 ms, versus 20–45 ms before.
  - Unplug or disable the SystemCore cameras when they aren't being tested.
- `System/TopThreads` lists this program's busiest threads from `/proc`, native ones included. A
  native thread that never named itself shows as `java/<tid>`. The boot console prints
  `[Threads] <step>: <new tids> | cpu ticks: ...` to tie each tid to what created it.
  `./gradlew deployrobotLogsystemcore -ProbotLog -ProbotThreads --info` samples them from the
  robot itself ([tools/robot_threads.sh](tools/robot_threads.sh)).
- The SystemCore's own cameras connect straight to this program as `limelightsc0/1` — no NT bridge
  is needed when the SystemCore is the robot controller. (Deploying this code replaced the
  SystemCoreVision bridge program on that unit; redeploy that repo to get it back.)
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

`-Pset=/SmartDashboard/Telemetry/MirrorLogsToNT=true` sets a boolean first; `-PwaitMs=` waits
longer. From Git Bash, prefix with `MSYS_NO_PATHCONV=1` or the leading `/` is turned into a
Windows path.

## Before the event — CALIBRATE / check

- [ ] **CAN wiring** matches `CanBuses` (`USE_CANIVORE` and the port table above). With a CANivore:
      `canivore-usb` installed on the SystemCore. `SystemCoreCAN<n>/StatusOK` (and
      `CANivore/StatusOK`) in the log, and nonzero `BusUtilization` on every port in use.
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
