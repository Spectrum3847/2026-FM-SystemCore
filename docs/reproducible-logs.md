# Reproducible logs: code, configuration and calibration

AdvantageKit replay feeds a log's **inputs** (sensor readings, camera results) through **today's
code**. Every value that lives in code comes from today's version, not the robot's at match time:
camera mounts, wheel radius, steer offsets, TagEkf noise, PID gains, scoring heights. A replay of
an old log therefore silently mixes the match's sensor data with today's robot. That is sometimes
what you want ("what would the new calibration have done?") and sometimes wrong ("why did the robot
do that?"), and today the two can't be told apart.

Example: the TXMAN1 replays in `pose-improvements.md` ran with the current Orin mounts, not the
match's, and only `-Pmountfix` made that visible.

## What logs carry today

| Logged | As | Replay uses it? |
|---|---|---|
| Git SHA, branch, date, dirty flag (`RealMetadata/Git*`) | Metadata | No; check the commit out by hand |
| Robot Settings values and their source layer (`Params/<group>/<key>`, `Params/Sources/...`): swerve offsets, soft limits, camera settings | Input | **Yes**: every declared setting replays as the robot had it (not in the TXMAN1 logs, which predate it) |
| Field map (`Vision/FieldMap`) | Input | **Yes**: `TagField` reads it back, so replay solves with the match's field. The pattern to copy |
| Orin intrinsics (`Vision/Orin/<cam>/CameraMatrix`, `DistCoeffs`) | Input | Yes |
| Orin mount estimates from the Jetson (`Vision/Orin/<cam>/Mount/*`) | Input | Yes, where consumed |
| Steer offsets (`Swerve/Align/<module>/ConfigOffset`, `AppliedOffset`) | Output | No: recomputed from today's code |
| Shot offsets (`ShotCalc/DriveAngleOffsetDegrees`, `HoodAngleOffsetDegrees`, `YawOffsetDeg`) | Output | No |
| Robot-code camera mounts, wheel radius, ratios, module positions, IMU mounts, TagEkf settings, PID, current limits, mechanism setpoints | Nothing | Today's code |
| Limelight on-camera settings (Q30's flipped LL-Back mount lived there) | Nothing | n/a |

## Three tiers

| Tier | What | Needed for | How |
|---|---|---|---|
| 1. Code | Everything in the repo | Reproducing exactly | Git SHA, plus the diff when dirty |
| 2. Device configuration | What each motor and sensor actually ran with: PID, current limits, ramps, inversions, neutral mode, sensor offsets | Explaining behaviour; catching configs that differ from code | Read back from the devices at boot, logged as metadata |
| 3. Calibration | Values that turn inputs into robot state or that were measured on this robot | Replaying an old log with its own values without a checkout | Robot Settings: already logged as inputs and restored in replay |

### 1. Code: git, made complete

`build.gradle` already has AdvantageKit's `eventDeploy`: on a branch named `event…`, every deploy
commits all changes first. Gaps and fixes:

- **Only `event…` branches auto-commit.** TXMAN1 deployed from `robot/fm-bringup`; a dirty deploy
  from any other branch only logs "Uncommitted changes" and the changes are lost.
  Fix: auto-commit (or at least warn loudly) on a dirty deploy from any branch.
- **Embed the diff when dirty.** At build time, write `git diff HEAD` (usually a few KB) into
  `BuildConstants` and log it as metadata (`GitDiff`). Any log, dirty or not, is then reproducible:
  check out the SHA, apply the diff. This alone makes tier 1 complete.
- **Push.** An auto-commit exists only on the deploying laptop; nobody else can check out that SHA.
  Print the push command after an auto-commit, or push automatically.
- **Untracked files.** `git diff HEAD` misses new files; include `git status --porcelain` and the
  untracked files' contents, or `git add -N` before diffing.

### 2. Device configuration: read back from the device

Logged once at boot (and after any reconfigure), per device: the full config as the device
reports it (`getConfigurator().refresh(...)` for Phoenix 6), as JSON metadata under
`DeviceConfig/<name>`. About 100-150 values per TalonFX; 20 motors is a few KB against 120-200 MB
logs.

Read back rather than copied from code because the two can differ: a config call can fail (logs
already count `CANConfig/FailedCalls`), and settings changed in Phoenix Tuner stay on the device.
Compare against what the code requested and raise an alert on mismatch.

Replay doesn't need these: it uses measured motion, so PID gains don't change a replayed pose. They
matter for explaining behaviour and for sim.

### 3. Calibration: Robot Settings

The Robot Settings system (`frc.spectrumLib.params`, being renamed from Params; see
`plan-2027-restructure.md`) already does this for every value declared through it, so calibration
belongs there rather than in a separate input:

- **Layers:** declared default in code, then `deploy/params/<group>.json` (git), then
  `deploy/params/robots/<robot>/<group>.json` (git, per robot), then the robot-local file saved in
  the pit (survives deploys, **not in git**), then live dashboard changes (tuning mode, disabled
  only).
- **Logged:** every value and the layer it came from is an input (`Params/<group>/<key>`,
  `Params/Sources/...`). Replay restores them, so a replayed match already runs on the robot's own
  values, whatever the replaying PC's files say.
- **Hard-stop groups** (swerve offsets, soft limits) carry a `HardwareStamp`: a file saved on one
  robot isn't applied to another.
- **Pit-saved values exist only on the robot and in the log.** The log is their only record, which
  is why logging them is essential.

Groups today: `swerve` (encoder offsets), `soft-limits` (hood, intake extension), `cameras`.

**What to move into Robot Settings** (one group each, physical facts marked hard-stop where a wrong
value could damage something or the value is per-robot hardware):

- Drivetrain: wheel radius, per-module scale, module positions, drive, steer and couple ratios.
- IMUs: Pigeon and SystemCore IMU mount orientation, gyro scale.
- Cameras: each camera's robot-to-camera mount (today `Vision.VisionConfig.orinRobotToCamera` and
  the Limelight configs, in code), lag.
- Pose estimation: TagEkf noise, gates, tilt policy, field-prior, mount-estimator and
  self-checking settings.
- Mechanisms: zero and home offsets, encoder offsets, gear ratios (soft limits are already there).
- Scoring: setpoints and heights, shot calculator tables, shot offsets (`ShotCalc/*Offset*`, today
  outputs).

Also log a **settings hash** over all groups (and a version string), so two logs can be compared at
a glance.

**What Robot Settings doesn't do yet:**

- **Replay with today's values.** Replay always restores the logged values. For "what would the
  robot do now?" (what `TagEkfTuner` does), replay needs a switch to ignore the logged values for
  chosen groups and resolve from today's files instead (next section).
- **Values not declared through it** are still in code; only tier 1 (git + diff) covers them.

## Replay: logged values or today's?

Robot Settings replay is `logged` today. Add the other mode, chosen explicitly (e.g.
`-PreplaySettings=logged|current`, default `logged`):

| Mode | Question it answers | Use for |
|---|---|---|
| **logged** | Why did the robot do that? | Debugging a match; the replay should match the robot's own outputs (`tools/replay_compare.py`) |
| **current** | What would the robot do now with this data? | Testing a new calibration, noise model or scoring table against old matches (what `TagEkfTuner` does) |

A rule of thumb for values that aren't clearly one or the other:

- **Physical facts about the robot at that moment** (mounts, offsets, wheel radius, zeroes): the
  logged value is the truth for that log. Override only to test a correction, as `-Pmountfix` did.
- **Decisions and tuning** (setpoints, scoring heights, gates, noise, PID): `logged` to reproduce,
  `current` to evaluate a change.

Per group, since Robot Settings is already grouped (`-PreplaySettings=logged,cameras=current`), so
one change can be tested while everything else stays as it was. In `current` mode a group resolves
from the replaying PC's files exactly as on the robot, minus the robot-local layer (or with a
chosen robot-local file).

Replay should log which mode and which hash it ran with, so a replay log says what it reproduced.

## Order of work

1. Embed the git diff when dirty, and warn on dirty deploys from any branch. Small, and makes every
   future log reproducible.
2. Device config read-back with mismatch alerts. Catches real faults on its own.
3. Move calibration into Robot Settings, starting with camera mounts, drivetrain, IMUs and pose
   estimation (what replay of pose needs), then mechanisms and scoring. Add the settings hash.
4. The `current` replay mode, per group.
