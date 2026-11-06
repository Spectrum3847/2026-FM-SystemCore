# Robot app

The robot app is a small pit web app. It is plain HTML and JavaScript in
[`src/main/deploy/app/`](../src/main/deploy/app/), so every `./gradlew deploy` ships the app that
matches the code. There is no Node, npm or build step.

**On FM the app is not reachable yet.** The robot program serves it on port 5800 (`/app/`), but on
FM's SystemCore the built-in vision service already holds 5800. A separate app service on a free
port is planned and not built yet; see [robot-app-plan.md](robot-app-plan.md#check-on-the-bench-first).

| Page | What it does |
| --- | --- |
| Logs | Opens a `.wpilog` from the laptop: run metadata, mechanisms, CAN buses, keys no page shows. |
| Power | Pack current and sag, pack resistance, main breaker heat, current and energy per mechanism. |
| CAN Bus | Utilization and error counters per bus, leader and follower dropouts, alerts, loop time. |
| Swerve Align | Live CANcoder readings over NetworkTables; prints the `configEncoderOffsets(...)` line. |
| Cameras | Each Limelight's stream, mount (accelerometer and tag solve) and image tuning. |

The app only reads from the robot. Swerve Align and Cameras print the Java line to paste; Cameras
can also save a mount or image setting to the camera itself.

To add a page, make `pages/<name>/index.html` and `main.js`, and add it to `PAGES` in
`lib/ui.js`. Libraries go in `vendor/` as single files. Tests run with
`node --test tools/app-test/*.test.mjs`, in CI and locally.
