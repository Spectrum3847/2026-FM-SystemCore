# 2026-FM-SystemCore

Robot code for FM, FRC Team 3847 Spectrum's 2026 robot, moved from the roboRIO to SystemCore. At
offseason events FM competes as team 8515.

The repo is also a testbed for robot pose sources. FM has Limelights, a Jetson Orin running
PhotonVision, and SystemCore's own cameras. The code gates and logs each source on its own, so the
match logs show which source to trust and when.

## What you need

- Java 25. The JDK that ships with WPILib 2027 works.
- [AdvantageScope](https://github.com/Mechanical-Advantage/AdvantageScope) 2027, to watch the sim
  and read logs.
- For the robot, a SystemCore image that matches the WPILib version in `build.gradle`. See
  [Before the event](docs/pre-match.md#before-the-event-calibrate-and-check).

Library versions are in `build.gradle` and `vendordeps/`.

## Quick start

Point `JAVA_HOME` at the `jdk` folder of your WPILib 2027 install (on Windows it is under
`C:\Users\Public\wpilib\`). Then:

```bash
./gradlew build                                # compile, format and run the unit tests
./gradlew run                                  # simulator with the sim GUI
FM_SIM_SCRIPT=drive ./gradlew run -PnoSimGui   # scripted drive, no driver needed
python tools/check.py                          # tests and the drive sim in one run
```

To deploy, connect to the robot over USB and run `./gradlew deploy`.

## Docs

The docs explain how the robot works and why. They can fall behind the code. When a doc and the
code disagree, the code is right.

| Doc | Read it for |
| --- | --- |
| [controls.md](docs/controls.md) | Pilot and operator bindings, and controllers on the NI Driver Station. Drivers start here. |
| [shooting.md](docs/shooting.md) | Aiming, the shot readiness gate, hood homing and set shots. |
| [pre-match.md](docs/pre-match.md) | The robot's GO / NO-GO checks and the before-event checklist. |
| [bench-checks.md](docs/bench-checks.md) | What to test on the robot before an event. |
| [development.md](docs/development.md) | Sim scripts, replay, CI, and reading the robot from a laptop. |
| [localization.md](docs/localization.md) | Pose sources, the Orin and Jetson, and tilt handling. |
| [pose-sources.md](docs/pose-sources.md) | Why the pose-source testbed is built the way it is. |
| [robot-runtime.md](docs/robot-runtime.md) | Loop rate, logging, the dashboard, storage and SystemCore hardware notes. |
| [params.md](docs/params.md) | Tuned values kept in JSON instead of Java. |
| [robot-app.md](docs/robot-app.md) | The pit web app the robot serves. |
| [systemcore-port.md](docs/systemcore-port.md) | Where the code came from, what changed for SystemCore, and known gaps. |
| [match-summary.md](docs/match-summary.md), [shot-log.md](docs/shot-log.md) | The match summary and shot log formats. |
| [log-keys.md](docs/log-keys.md) | Every logged key. Generated from the code. |

## Contributing

- Branch from `main`, one feature per PR, and fill in the PR template.
- Compiling is not testing. A change to how the robot behaves needs a scripted sim run, a replay of
  a real log, or a bench test, described in the PR.
- Run `python tools/check.py` before you push. Add `--sims drive,shoot,auto` to run every sim CI
  runs.
