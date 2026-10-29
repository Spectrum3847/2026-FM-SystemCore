# Agent notes

FM (Spectrum 3847's 2026 robot) on SystemCore, competing as 8515. See README.md first, then
docs/pose-sources.md for the localization testbed.

## Build

- Java 25, the WPILib 2027 JDK. On team laptops:
  `export JAVA_HOME=/c/Users/Public/wpilib/2027_alpha5/jdk`.
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
- Never remove or bypass `Logger.processInputs` or a logged input.

## Robot safety

- Never run `./gradlew deploy` (or any deploy task) unless the person asked in their own message.
  Deploying moves a real robot.
- Hard stops. Before changing any of these, state "this changes X from A to B; failure mode is Z"
  and wait for the person to say "confirm", unless the task you were given asks for that change.
  Either way, put the old value, new value and failure mode in the PR. The list: CAN IDs and bus
  assignment (`CanBuses`), motor inversions, swerve encoder offsets (`FM2026`), current limits
  (supply, stator, torque), soft limits, gear ratios, brake/coast, gains outside a tuning task,
  homing and zero positions (`initPosition`, `setPosition`), and the `Rio` serial-to-config mapping.
- Never remove or weaken a safety interlock, timeout or override (the `ShotGate` timeout, the
  operator Y bypass, the CAN config budget) without the same confirmation.
- Never act on a device's readings while its `connected` input is false. At TXMAN1 a CANivore
  drop zeroed every position, and code treated the zeros as real.
- No blocking Phoenix calls (`setPosition`, `getConfigurator().apply`/`refresh`,
  `setUpdateFrequency`, `optimizeBusUtilization`) in code that runs while enabled. Use them in init
  or disabled, or off-thread with the result fed back as a logged input. Each can wait 100 ms.
- Every wait that gates an action (feeding, scoring, an auto step) has a timeout or a driver
  override.
- Compiling is not verification. A change to robot behavior needs a scripted sim run, a replay
  comparison on a real log, or a bench test described in the PR, under "How I verified".
- On `event*` branches, make the smallest change that works: no refactors, no `spectrumLib` updates
  (a bug fix there is fine when it is the smallest fix), and a person reviews before deploying.
  After the event, merge to `main` only through a PR.
- Never edit `BuildConstants.java` (generated). Change `vendordeps/*.json` only when asked for a
  dependency update.
- Never put the log-site upload key, radio keys or tokens in code, chat or commits. Logs uploaded
  to robotlog.spectrum3847.org are public; upload only what was asked.
- Every log carries `LogKeyGuide` metadata: what each key means, its unit, input or output, and
  its source file (`docs/log-keys.md` is the same list). Read it before guessing at a key. After
  adding or renaming a logged key, run `python tools/make_log_key_guide.py` (CI checks it), and
  put a one-line comment above the call or field saying what the value means.
- The log tools have tests: `python -m unittest discover -s tools -p "test_*.py"`. A change to a
  tool, or a renamed key a tool reads, updates `tools/test_log_tools.py` too.
- Logs: `tools/wpilog_summary.py`, `tools/shot_log.py`, `tools/replay_compare.py`, the
  `spectrum-logs` MCP (robotlog.spectrum3847.org) and the `frc-log-triage` skill in
  Spectrum-AI-Tools.

## Conventions

- Hardware constants are FM's from 2026-Spectrum `main`; do not copy the offseason bot's.
- Vision thresholds carry their 2026 rationale in comments; change them only with log evidence,
  and prove it with replay (`tools/replay_compare.py`).
- One feature per PR.

## Robot app

`src/main/deploy/app/` is the pit web app, built for this robot. It lives in this repo, is built by
the same Gradle build and ships with every deploy. Today the robot program serves it at
`http://robot.local:5800/app/`; it is moving to its own service. Design: concepts_brief.md.

- No npm, no build step on laptops. A library goes in `app/vendor/` as one file.
- Pages are hand-made for this robot's mechanisms. Keys, params and button bindings come from lists
  generated from the Java (`log-keys.json` today), never typed by hand in a page. No Java file
  paths or CAN IDs in `app/`.
- The website never moves the robot. Steps that need motion run in utility mode, and the driver
  presses the buttons. Tuning goes through the params system (`/Tuning/`, `SaveParams`), applied
  only while disabled and logged as inputs.
- Test with `node --test tools/app-test/*.test.mjs`. A PR that renames a log key, NT key, param or
  binding a page reads updates that page in the same PR.

## Comments and commit messages

- Comment only what the code can't say: why something non-obvious is done. Prefer clear names. No
  history of the old implementation, and no change logs in comments.
- Commit messages: a short subject, then why if it isn't obvious. No file lists or development
  diary.
