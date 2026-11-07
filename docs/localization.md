# Localization and vision

## The pose-source testbed

Design and rationale: [pose-sources.md](pose-sources.md).

Sources (names are the log paths under `Localization/Sources/`):

| Source | Device | Fuses by default |
|---|---|---|
| `LL-Back/MT1`, `LL-Left/MT1`, `LL-Right/MT1` | FM's Limelights, MegaTag1 | yes, until the seed is confirmed |
| `LL-Back/MT2`, `LL-Left/MT2`, `LL-Right/MT2` | FM's Limelights, MegaTag2 | yes, once the seed is confirmed |
| `SC0/MT1`, `SC0/MT2`, `SC1/…` | cameras plugged into the SystemCore (its built-in Limelight stack) | no |
| `Orin-Left`, `Orin-Right` | Jetson Orin + PhotonVision (`orin-left`, `orin-right`) | no |

**Every source runs all the time.** "Fuses" only decides whether its accepted measurements move the
robot's pose. Toggle any of them live from a dashboard at
`/Localization/Sources/<name>/Enabled` (an AdvantageKit dashboard input, so replay knows too).
Two exceptions use MegaTag1 results even when that source is turned off. Seeding and the
gross-heading correction use the best of FM's Limelights. The LL-Back heading correction
(`Vision/BackHeadingCorrections`) uses `LL-Back/MT1` in teleop, and in auto while the path asks for
pose updates or the robot is launching.

**All-camera EKF** (`Localization/EKF/Pose`): every tag from both Orin cameras and the three
Limelights, one at a time, in one EKF. Logged next to the fused pose for comparison; it never moves
the robot. See [pose-sources.md](pose-sources.md#ekf-alongside-wpilib).

What gets logged, per source:

| Key | Meaning |
|---|---|
| `Localization/Sources/<name>/*` (inputs) | the raw frames: poses, timestamps, tag counts, distances, ambiguity, tracking, all **before** any gate |
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

The robot also summarises each run itself, under `Summary/...` in the log, and after an FMS match
in `<log name>.summary.json` next to the log: battery, brownouts, loop overruns, CAN drops and
reboots, Driver Station drops, vision counts, auto, alerts and peak motor temperatures. See
[match-summary.md](match-summary.md).

Replay a log through the current code (edit a gate, replay, compare the shadow tracks):

```bash
FM_REPLAY=1 AKIT_LOG_PATH=/path/to/log.wpilog ./gradlew run -PnoSimGui
```

```bash
python tools/replay_compare.py /path/to/log_sim.wpilog
```

The replay log (`*_sim.wpilog`) holds both `RealOutputs` (what the robot did) and `ReplayOutputs`
(what the edited code would have done).

### What the sim shows today

From the scripted drive (see [pose-sources.md](pose-sources.md#simulation-results) for the table):
odometry alone drifts to ~0.16 m while the fused pose averages ~0.014 m error. MegaTag2's shadow
tracks come out slightly worse than MegaTag1's, because MegaTag2 inherits the simulated gyro drift
through the heading the robot pushes to the camera. Check this on the real robot.

## Orin and the Jetson

The Orin cameras run PhotonVision on the Jetson ([SpectrumJetson](https://github.com/Spectrum3847/SpectrumJetson)).
This section is issue #10's robot side. Everything that talks to NetworkTables is in an IO class,
so replay stays clean.

**Cameras.** FM's Jetson has two cameras, `orin-left` and `orin-right`, logged as `Orin-Left` and
`Orin-Right`. Adding one means a name, mount, installed flag and factor in `VisionConfig`.

**Field map.** The robot's layout (the event's, see [Pre-match checks](pre-match.md#pre-match-checks)) is the
one source of truth. At boot it publishes it for the Jetson:
- `/SpectrumVision/FieldMap/Json`: the layout as WPILib 2027 Field JSON (`field-tags`,
  `field-dimensions`)
  ([FieldMapJson](../src/main/java/frc/spectrumLib/vision/FieldMapJson.java)).
- `/SpectrumVision/FieldMap/Hash`: its canonical hash
  ([FieldMapHash](../src/main/java/frc/spectrumLib/vision/FieldMapHash.java)).
- `/SpectrumVision/FieldMap/Name`: the WPILib field's name, or `custom`.

The Jetson reports the map it loaded at `/photonvision/jetson/FieldMap/Hash`, `Name`, `LoadedAt`
and `Error` (logged as `Vision/Jetson/FieldMap/*`). It hashes the layout it parsed from the
published JSON, so the robot publishes exactly the numbers it hashes. The Settings check fails when
the Jetson's hash differs from the robot's, and warns (with its `Error`) when it reports none.

**The whole Orin pipeline replays.**
- [PhotonOrinIO](../src/main/java/frc/robot/subsystems/vision/PhotonOrinIO.java) logs every PhotonLib
  result as raw bytes (`Vision/Orin/<camera>/Results`) while the robot is enabled. Empty results
  are included, so the frame rate and dropouts are in the log. While disabled it logs at most one
  result with targets every 0.1 s and drops empty ones, so a disabled stretch replays only those.
  It also logs the calibration and the Jetson's per-camera topics.
- [OrinSolver](../src/main/java/frc/robot/subsystems/vision/OrinSolver.java) solves poses from those
  inputs only, so in replay it runs again. Change a mount, the solver or a gate, replay a match, and
  the Orin poses move. The sim run was checked both ways: an unchanged replay matched exactly, and a
  0.5 m mount change moved only that camera's poses by 0.5 m.
- The solved observations are outputs now, at `Localization/Sources/Orin-<camera>/Observations/*`.
  Logs from before this change have no raw results, so their Orin data doesn't replay.

**How each frame is used** (6328 and AOS, issue #10 section 8b):
- **Multi-tag:** the Jetson's solve. Tag count and distance come from the tags it used, not from
  every tag in view (PhotonLib alpha-2 bug, section 10a).
- **Single tag:** the least ambiguous tag. Of its two candidate poses, the one whose heading is
  closer to the gyro wins.
- **Never used:** game pieces (`fiducialId < 0`), tags not in the layout, and excluded tags.
- **Trust:** std dev xy = 0.01 · d² / n², times the camera factor and an image-edge scale. A tag near
  the image edge gets up to 4x the std dev; it is not dropped. Heading stays with the gyro, as for
  every other source.
- **Extra rejections, each with its own counter:**
  - single-tag ambiguity ≥ 0.4
  - z outside −0.5 to 1.0 m
  - a single tag farther than 5 m
  - heading more than 20° from the fused heading
  - the first 2 s of auto
- A bad PhotonLib packet is counted and dropped (`DecodeFailures`), so it can't stop the loop
  (section 10b).

**Two more estimates per camera**, logged and shadowed. They are not fused unless their source is
turned on at `/Localization/Sources/<name>/Enabled`, and then only while the robot is enabled.
Both are built from the same logged results, so they replay too, and both need the pose seeded first:
- **`Orin-<camera>/Gyro`** (#10 section 3) solves on the robot with the heading held to the gyro at the frame's
  timestamp. With 2+ tags it uses PhotonLib's constrained SolvePnP, seeded from the fused pose. With
  one tag, or if the constrained solve lands more than 1 m from its seed, it uses the distance-trig
  solve instead. It runs on the newest frame per camera only. `Vision/Orin/<camera>/GYRO_PNP/SolveMicros` is the
  time to watch on the SystemCore. PhotonLib's native solver is loaded explicitly, so it also works in replay.
- **`Orin-<camera>/TxTy`** (#10 section 8c, 6328's final-alignment estimate) takes the closest tag's angles and
  solved distance, plus the gyro heading, and gives the robot's position.
- In the sim both come within 2–4 cm of the true pose, about the same as the main Orin source.

**Dashboard controls** (NetworkTables, recorded by AdvantageKit):

| Topic | What it does |
|---|---|
| `/Vision/Orin/ExcludedTags` | Comma list of bad tags, e.g. `7,12`. It is sent to the Jetson (multi-tag ignores them), and single-tag solves skip them too. Default: `VisionConfig.orinExcludedTags`. The Jetson's combined list is logged as `Vision/Jetson/ExcludedTagsActive`. |
| `/Vision/Orin/PracticeFieldProfile` | Off = pipeline 0 (event), on = pipeline 1 (practice field), on every camera. Default off, so a forgotten switch can't carry practice settings into a match. |
| `/Vision/Orin/Rewind/ManualRecord`, `.../ManualLabel` | Bench recordings without the FMS. |

**Rewind.** With the FMS attached, the Jetson records from enable until 10 s after the last disable.
That makes each match one recording (auto, the gap and teleop), named `<event>-Q12-r0`. Without
the FMS, the robot leaves `record` alone unless the manual switch changes. FM's Limelights still
save their own 165 s buffers at auto and teleop end.

**Clock.** The robot publishes its wall clock (`/photonvision/clock/unixMs`) every loop once the DS
has set it. The Jetson has no clock battery, and this gives it the right date offline. SystemCore
gets its clock from the DS like the roboRIO (`MrcCommDaemon` sets it when the DS connects), and the
Jetson's clock follows (checked on FM, 2026-10-29). Until a DS connects, both can be days off.

**Logged** under `Vision/Jetson/*` and `Vision/Orin/<camera>/*`:
- Jetson health: temperatures, fan, power, GPU load, throttle reason, over-current events, JPEG
  decoder checks and heartbeat.
- Rewind: `Recording`, `Session`, `FreeGB`, `FramesDropped`.
- Per camera: `Health/*` (fps, pipeline ms, latency, frames, decode failures, recoveries), `Mount/*`
  (the Jetson's estimate) and `SettingsJson` (full pipeline and calibration, written on change).
- Robot side: `ResultsPerSecond`, `LatencyMs` and `ResultAgeSeconds`.

**Alerts** (only for installed cameras, on the real robot):

| Condition | Level |
|---|---|
| Jetson heartbeat stale > 3 s | error |
| Camera not on NetworkTables (2 s), connected but no frames for 1.5 s, or 0 fps for 1 s | error |
| Camera under 100 fps for 2 s, once enabled for 3 s (disabled, the Jetson idles cameras at ~30 fps) | warning |
| Camera JPEG decode failures rising, or the camera was reset during the match | warning |
| Result timestamps more than 0.5 s from now for 2 s (time sync / timebase) | warning |
| Jetson tj temp > 75 °C, fan < 1000 rpm, throttling, over-current during the match, CPU JPEG fallback | warning |
| Rewind SSD < 20 GB, dropping frames while recording, or should be recording but isn't (FMS) | warning |
| Disabled and still, 60+ multi-tag samples: measured mount differs from config by > 1° or > 2 cm | warning |

A camera turned off with `setEnabled(false)` raises none of its alerts.

**During matches**, keep dashboards off the high-rate vision topics. PhotonVision publishes every
frame's result, a few Mbps for four cameras, and anything the Driver Station subscribes to crosses
the field radio. Subscribe only to the topics you display. Don't open the camera streams
(ports 1181+) or the PhotonVision UI (5800) from the DS during a match.

**Jetson uptime** is read from PhotonVision's metrics protobuf (`/photonvision//metrics/photonvision-3847`;
check the host name on the robot network). If uptime drops, the Jetson rebooted, and a "Jetson rebooted"
warning stays up until the next enable.

**Log size.** The raw Orin results are the biggest thing in the log:
- With the robot enabled, every result is kept. A 5-minute session with four cameras at 122 fps is
  about 160–330 MB, versus about 60 MB without the Orin.
- With the robot disabled, only the newest result with targets is kept, at 10 Hz per camera.
  `ResultCount` still records every frame, for the frame rate and the no-frames alert.
- The unused min-area-rect corners are dropped before logging.
- Use a USB stick at events: the internal-storage cap (2 GB) holds only about 6–12 such matches.
- If the log writer ever falls behind, AdvantageKit drops that loop's data and prints a warning. It
  never blocks the robot loop. Watch `Logger/QueuedCycles` on the robot.

**Replay is exact up to floating-point noise.** A replayed run can drift by a centimetre or so from
the original. Replay's first difference is in the last bit (about 1e-18) around seeding: Java's
`Math.sin`/`cos` may round differently in interpreted and JIT-compiled code, and a threshold then
amplifies it. Orin observations that don't depend on the fused pose replay bit-exact.

**Not done yet** (issue #10): a game-piece map (8e).

**To check on the robot:**
- the Jetson topic names (AdvantageScope on `/photonvision`)
- timestamps (spin in front of a tag and look for smear)
- NetworkTables reconnects (Jetson booting first, robot reboot, cable pulls)
- a multi-tag frame with an excluded tag in view

## Tilt and knocks (Pigeon)

The Pigeon's pitch, roll, pitch and roll rates, and 3-axis acceleration are logged inputs
(`Swerve/Gyro*`, `Swerve/Accel*G`). [TiltState](../src/main/java/frc/robot/subsystems/swerve/TiltState.java)
derives the following, logged under `Swerve/Tilt/*`:
- **`TiltDegrees`:** the angle from level, from pitch and roll.
- **`TiltRateDegPerSec`**
- **`ImpactG`:** how far the total acceleration is from 1 g. It stays near 0 whether the robot is still, tilted or driving smoothly, and spikes on a collision or a bump landing.
- **`BumpedRecently`:** an impact above 0.5 g in the last 0.5 s.
- **`Tilted`:** above 5°.

How they're used:
- **Odometry** (#10 section 8g, 6328): wheel travel is scaled by `OdometryScale`. The scale is 1 up to 2° of tilt,
  falls linearly to 0 at 25°, and is switched at `/Localization/TiltScaledOdometry` (on by default). On a bump the
  wheels slip, lift or climb, so they overstate travel across the floor. `Localization/OdometryPose` is never
  scaled, so replay can compare the two.
- **Vision:** "Robot Tilted Rejection" drops Limelight MegaTag2 and the Orin gyro and tx/ty frames while the
  robot is tilted more than 5°. Those take only the robot's yaw, so on a bump the camera isn't where they think.
  Full 3-D solves (MegaTag1, the Orin's main source) are unaffected.
- **Shooting:** `Swerve.isTilted()` and `Swerve.wasBumpedRecently()` are available to the shot readiness gate,
  but are not wired in yet. That's a driver-feel decision to make with logs.
