# Development: sim, replay and CI

## Build and run

Java 25 is required. The JDK that ships with WPILib 2027 works: point `JAVA_HOME` at the `jdk`
folder of the WPILib install that matches `build.gradle` (on Windows it is under
`C:\Users\Public\wpilib\`).

Build, format (Spotless, AOSP style) and run the tests:

```bash
./gradlew build
```

Run the simulator with the sim GUI (use the GUI's DS controls; an Xbox controller dragged onto
Joystick 0 with "Map gamepad" on reads correctly, see [Controllers](controls.md#controllers-and-the-driver-station)):

```bash
./gradlew run
```

Or run the scripted demo drive, which needs no driver. It seeds the pose while disabled, then drives a lap of straights, spins and strafes in teleop:

```bash
FM_SIM_SCRIPT=drive ./gradlew run -PnoSimGui
```

Or run one of FM's PathPlanner autos (any name from the Auto Chooser), e.g.:

```bash
FM_SIM_SCRIPT="auto:TBTB Left" ./gradlew run -PnoSimGui
```

Or the scripted shooting run: aim and launch standing, then aim and launch while strafing, with
fuel put in the hopper before each launch (it exercises everything under [Shooting](shooting.md)):

```bash
FM_SIM_SCRIPT=shoot ./gradlew run -PnoSimGui
```

Or the chord run: the pilot's LB + face-button chords pressed and let go in sloppy orders, read in
`SuperStructure/WantedSuperState`:

```bash
FM_SIM_SCRIPT=chords ./gradlew run -PnoSimGui
```

A scripted run closes its log and exits when the script ends, so the run is one command. Add
`FM_SIM_EXIT=0` to keep it running afterwards (to keep looking at it live in AdvantageScope). Add `FM_SIM_MATCH=Q12` to have the Driver Station report an FMS
match, so the end of teleop writes the match summary file into `logs/`.

All 32 autos and 26 paths from 2026 `main` are in `src/main/deploy/pathplanner`, unchanged. The Auto
Chooser is an AdvantageKit `LoggedNetworkChooser` (same dashboard key as 2026), so the selected
auto is logged and replays.

Open AdvantageScope (2027), connect to `localhost` (or open the `logs/akit_*.wpilog` the sim wrote),
and install the FM model from [advantagescope-custom-assets/Robot_FM](../advantagescope-custom-assets/Robot_FM).

Deploy to the robot (USB: SystemCore at `172.26.0.1`):

```bash
./gradlew deploy
```

## CI

Every PR runs, in parallel:

| Job | What fails it |
| --- | --- |
| `build` | Spotless, a compile error (Error Prone's ERROR checks included) or a unit test, including the CAN config invariants over every robot config (`ConfigInvariantsTest`: unique IDs per bus, followers on their leader's bus, soft limits around every home position, no motor in two mechanisms). Error Prone warnings print but don't fail; the style-only checks are off in `build.gradle`. |
| `build` (`simTest`) | `BindingsSmokeTest`: the whole robot booted headless in simulation, every pilot and operator input pressed alone, with LB, and LT with RT, disabled and in teleop. Fails if a press throws, if a binding in its table doesn't schedule its command or reach its wanted state, or if a lone button schedules something the table doesn't list. About 30 s; run it alone with `./gradlew simTest`. |
| `sim (drive)`, `sim (shoot)`, `sim (auto)` | the scripted sim (`auto` is `TBTB Left`) crashing, not finishing its script, an auto path stopping short of its end before autonomous ends (being cut off by the end of autonomous is fine), the fused pose more than 0.15 m from the sim's truth at p95 (0.3 m worst) once the seed is confirmed, or more than 5% of loops overrunning by 25% ([tools/ci_assert.py](../tools/ci_assert.py)) |
| `replay` | the PR moving the fused pose (5 cm, 2°) or the shot gate's verdicts (10 loops) in any replay fixture, against the base branch replaying the same fixtures ([tools/replay_regression.py](../tools/replay_regression.py)) |
| `app` | the robot app's tests |

Sim logs and replay logs are uploaded as artifacts.

To check locally in one command, use [tools/check.py](../tools/check.py). It builds once, then runs the
tests, scripted sims and replays in parallel, and prints one PASS/FAIL summary (full output in
`build/check/`):

```bash
python tools/check.py --quick --log robotlogs/X.wpilog   # ~80 s: unit tests, drive sim, a replay
python tools/check.py --sims drive,shoot,auto            # ~2.5 min: everything CI's build and sims run
```

Loop overruns only warn there, since the jobs share the laptop's CPU; CI judges loop timing.
`check.py` does not convert old logs: a log recorded before AdvantageKit alpha-7 (microsecond
timestamps) needs `python tools/akit_log_to_ns.py --in-place <log>` before `--log` can replay it.
A `--log` replays only the enabled part of the log (10 s before the first enable to 5 s after the
last disable; `--full-log` for all of it), since a replay costs about 16 s plus 0.1 s per logged
second. The comparison with the robot then skips the cut's first 2 s, raw odometry and the shadow
tracks, which build up from wherever they start; the fused pose replays exactly. A log from older
code differs where the code changed: that is the replay's answer, not a failure.

When a PR is meant to change the replay (a gate, a std-dev tier), add the `replay-change-expected`
label.

The replay fixtures are windows of real match logs, listed in
[fixtures.json](../src/test/resources/replay/fixtures.json) with what each covers. The logs themselves
are not committed: [tools/fetch_replay_fixtures.py](../tools/fetch_replay_fixtures.py) downloads the
release asset each comes from (the redacted `TXMAN1-logs` copies), checks the log's SHA-256 and cuts
the window with `trim_wpilog.py`. CI caches the cut windows until the list or those tools change.
The cache is saved by the Replay workflow's runs on main (each push, and weekly), since a cache a PR
saves can be restored only by that PR.
Today: TXMAN1 Q24 180-215 s (pre-match through the vision seed in auto, two auto launches, the start
of teleop), Q32 290-335 s (seven teleop volleys, two fed by the gate timeout) and Playoff 20
350-395 s (the event's last code, one volley fed while the pose was untrusted).

The fixtures were recorded in microseconds, so they need the conversion to nanoseconds that the
Replay workflow also runs. To add one, pick a window with the pose seed or some shots (`tools/shot_log.py` lists them), add an
entry with the full log's SHA-256, and replay it locally:

```bash
python tools/fetch_replay_fixtures.py build/replay
python tools/akit_log_to_ns.py --in-place build/replay/*.wpilog
FM_REPLAY=1 AKIT_LOG_PATH=build/replay/NAME.wpilog ./gradlew run -PnoSimGui
```

Each fixture adds about 30 s to both replay jobs. Logs from another event go in a release of their
own, with the radio's keys redacted the way `TXMAN1-logs` was.

## Reading the robot from a laptop

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
