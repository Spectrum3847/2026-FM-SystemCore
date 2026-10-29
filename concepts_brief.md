# FM Pit App on SystemCore

* **Author:** (pit app brainstorm, EGM)
* **Date:** 2026-10-29

## Overview

A web app that pit students and programmers open from a laptop or phone to run and fix FM. It shows how to drive the robot (controller layouts), guides tuning (swerve module offsets, gyro scale), lists the devices on the robot network with links to their own config pages, and downloads logs in one click. It keeps working when the robot program has crashed, because that is when people need logs and diagnostics most. An AI session, directed by a student or coach, uses the same app through a JSON API. A person must confirm any change to a hard-stop value (AGENTS.md).

The app is a separate small service on the SystemCore, installed by the normal `./gradlew deploy`. Its pages stay plain HTML and JavaScript. Live robot data goes from the browser straight to the robot's NetworkTables server; the service handles files, status and the API.

It is the one place a student starts. For every pit job it either does the job itself or says which tool to use and links to it, so nobody has to learn where each tool lives.

## Analysis

Six reviewers looked at the concept without seeing this conversation. Their findings, merged:

**1. The current first version fails the crash requirement (all six reviewers).** `Robot.java:294` starts `WebServer.start(5800, ...)` inside the robot program, and only when `hasHardware()`. When the robot program crashes or restarts, the app, log download and diagnostics go down with it. The fix is a separate process managed by systemd (`Restart=always`). `build.gradle` already runs `systemctl` and `journalctl` against the `robot` unit through a GradleRIO `CommandArtifact`, so a deploy can install and restart a second unit the same way.

**2. The front end was judged sound; the hosting was not.** Five of six said to keep the plain HTML/JS pages with no build step, single-file libraries in `vendor/`, and the rule that the app keeps no copies of robot facts. That rule is what fixes the old `tools/robot-app` failures. The 524 KB `app/` folder and its `node --test` suite can be served by the new service unchanged. Starting over on the front end would cost work and fix nothing.

**3. The service must stay light on CPU.** The robot loop comes first. The SystemCore has 4 GB of RAM, so memory is not the limit; CPU is what to protect. Requirements:
- the service does almost nothing when idle: no log parsing or analysis on the robot; the browser does that work;
- a systemd unit with `Nice=10` and a CPU quota;
- a measured budget before adoption (for example under 3% CPU idle, under 10% while a log downloads), using the existing `-ProbotTop` task. No current CPU figure is assumed; measure it.

**4. Service language: Java.** Reviewers split between Python, Java and Go. With the facts now known:
- *Java, a second small program in the same Gradle build:* the Java runtime is already on every SystemCore, because the robot program needs it. Uses the JDK's built-in `com.sun.net.httpserver` and WPILib's own NT4 client, with no new dependencies. Tested with JUnit inside `./gradlew build`, same as the robot code, so agents check it with tools they already use. Its 60–100 MB of RAM is small against 4 GB. Cost: some CPU while the JVM starts up after a deploy. Run it at a lower priority and cap its heap so its garbage collector stays small.
- *Python:* can be installed on the SystemCore, but it is not there by default. Every new or reflashed SystemCore would need a setup step, often with no internet in the pit. A deploy that silently loses its app after a reflash is the kind of failure this project is trying to remove.
- *Go:* lightest on the robot, but adds a cross-compile toolchain that agents and laptops must keep working. Not worth it with 4 GB of RAM.

**5. Changes to the robot must go through the robot program (five reviewers).** The service must never write robot config or files the robot reads. Changes go as NetworkTables commands that the robot's main loop applies only while disabled and logs as inputs, so AdvantageKit replay matches. A crashed robot program therefore means read-only mode, and that is acceptable.

**6. Hard-stop confirmation has to be something an agent cannot do itself.** If the API caller can also send "confirm", an agent can confirm its own change. The design that survived: the agent *proposes* a change; the service holds it as pending (old value, new value, failure mode, short expiry); a person clicks Confirm on a page; the robot logs both the proposal and the confirmation with a request ID, so a log shows which session caused which change. The robot publishes which values count as hard stops; the app keeps no list of its own.

**7. One place to start, replacing other tools where it can (team decision, against the reviewers).** All six reviewers said to link to existing tools rather than rebuild them. The team overruled this: having to know AdvantageScope, Elastic, Phoenix Tuner X, WinSCP and each device's web page is hard to teach. The app must:
- for every common pit job, say which tool to use, why, and link to it (a "How do I…" page);
- do the job itself when the common case is small: download logs, check a device is reachable, see live motor and CAN status, set swerve offsets, see why the robot program is down.

The cost the reviewers raised still holds: every replaced feature is code that agents must keep working when the robot code changes. To keep that cost down:
- replace the common, simple case and link out for the deep one (for example, a live current and temperature table instead of Tuner X for checking motors, but Tuner X for firmware updates and configs);
- every page reads robot facts from the manifest, NetworkTables or the log, never a copy;
- every page states in its "How do I…" entry when to switch to the full tool.

Candidates to replace or partly replace: WinSCP (log download and file view), the Limelight and radio pages for status (link out for configuration), Tuner X for motor health checks, AdvantageScope for a quick look at the last match (power and CAN summaries). AdvantageScope stays the tool for detailed log analysis.

**8. Version mismatch needs a visible check.** The service and the robot program can be from different deploys (the service survives a crash; the browser may cache pages). The robot publishes a manifest with a schema version and git SHA. It goes on NetworkTables and also to a JSON file on disk, so the service can still show controls and log paths while the robot program is down. Pages show a banner when the SHAs differ. `lib/log-model.js` copies key names by hand today; generate that table from the `LogKeyGuide` output, which CI already checks.

**9. Log download details.**
- Use `LogStorage` paths (`/U/logs` or `/home/systemcore/logs`), not hardcoded ones.
- Stream files with HTTP Range support.
- Mark the log being written now as "in progress".
- Zip a day's logs on the fly.
- Slow downloads while the robot is enabled.
- Works on phone browsers, where the file goes to the share sheet.

**10. Other details.**
- *Port:* `LimelightHelpers` forwards 5800–5809 for USB Limelights, and a SystemCore teardown lists a built-in service on 5800. Pick a port that is free and allowed by the field (check the game manual).
- *Shortcut:* don't patch SystemCore's built-in web page; a system image update erases it. Use `robot.local:<port>`, a bookmark, and a QR code sticker on the robot.
- *System files:* read-only, limited to an allowlist (logs, deploy, selected config), with a path-traversal test.
- *Service health:* the service can't use AdvantageKit (separate process). It writes to journald and offers `/api/health` (version, uptime, robot unit state, NetworkTables connected).

## Goals vs. Non-Goals

* **In-Scope Goals:**
  * A separate service on the SystemCore that survives robot program crashes, installed and restarted by `./gradlew deploy`, adding under 10–15 s (about 1 s when unchanged).
  * One-click log download from laptop and phone.
  * Status page: robot program up or down since when, versions, last log.
  * Network page: devices found from NetworkTables and the neighbour table, reachable or not, links to their config pages.
  * Controls guide built from descriptions next to the bindings in Java.
  * Guided tuning procedures (swerve offsets, gyro scale), applied through logged NetworkTables commands while disabled.
  * A JSON API with a self-describing index, used by pages, `curl` and AI sessions; hard-stop changes need a person's click.
  * Read-only file viewer for an allowlist of folders.
  * A "How do I…" page that names the tool for every common pit job and links to it.
  * Replacements for the common case of other tools: log download and file view (WinSCP), device status (Limelight, radio, Jetson pages), motor health (Tuner X), a quick last-match power and CAN summary (AdvantageScope).
* **Out-of-Scope Non-Goals:**
  * Detailed log analysis and plotting; AdvantageScope stays the tool, linked from the app.
  * Rebuilding full camera, radio, Jetson or motor-controller configuration; the app shows status and links to them.
  * Changing robot behaviour while enabled, or writing robot files directly from the service.
  * Changing SystemCore's built-in web page.
  * Agents changing hard-stop values without a person.

## Alternatives Weighed

* **Keep serving from the robot program's `WebServer` (the current first version).** Rejected: goes down when the robot program crashes, which breaks a hard requirement.
* **An app installed on each laptop.** Rejected: drifts from the robot code, and needs per-laptop setup. This is how the old `tools/robot-app` went stale.
* **Node, npm or Vite front end.** Rejected: laptop toolchain and cached old builds; the old app's documented failures.
* **Off-the-shelf servers (Cockpit, filebrowser).** Rejected: need ARM builds on the device, Cockpit uses noticeable CPU, and neither offers hard-stop confirmation.
* **The service sends live data to browsers itself (one robot connection, shared out).** Not chosen for now: it saves NetworkTables load with many phones but adds CPU on the robot. Revisit if many clients become a problem.
* **Link to existing tools instead of replacing any (all six reviewers).** Rejected by the team: too many places to learn. Kept in part: the app links out for the deep cases.
* **Python service.** Rejected: not installed by default, so every new or reflashed SystemCore needs a setup step, and a missing install silently takes the app down.
* **Agent access only through a laptop-side command-line tool or MCP server (SSH plus existing tools).** Partly kept. One reviewer argued an on-robot API is unneeded because agents already have SSH, NetworkTables, `tools/*.py` and the `spectrum-logs` MCP. Others answered that pages need the same API anyway, so an MCP wrapper should be a thin layer over it, not a separate system.
* **Go binary for the service.** Rejected: needs a cross-compile toolchain, and 4 GB of RAM removes the reason to use it.

## Key Decisions & Staging for Next Steps

**Decided:**
1. Separate systemd service, installed by the normal deploy through a `CommandArtifact` that restarts it only when its files changed.
2. Keep the plain HTML/JS front end, the no-copied-facts rule and the `node --test` suite. Remove `WebServer.start` from `Robot.java` in the same change.
3. The browser reads live data from NetworkTables directly; the service does files, status, network and the API.
4. Writes only through logged NetworkTables commands while disabled; hard-stop changes are proposed by the API and confirmed by a person's click.
5. Build log download and the status page first; they are the clearest pain today.
6. The app is the single starting point: a "How do I…" guide for every pit job, and its own pages for the common case of each tool.
7. The service is a second small Java program in the same Gradle build.
8. The existing Power, CAN and Cameras pages are rebuilt, not kept as they are: each covers the common pit case, gets robot facts from the robot, and says when to switch to AdvantageScope, Tuner X or the camera's own page. The current code is a reference, not a base.

**Check on the bench before `/egm-draft`:**
1. CPU use with vision running, and with the service idle and serving a download.
2. Java service start-up time after a deploy, and its CPU while starting.
3. Can a deploy install and restart a systemd unit (sudo or `systemctl --user` with lingering)?
4. A free port that the field allows.
5. Which log folder is used, with and without a USB stick.
6. Does killing the robot program leave the service up?

**Undecided:**
- Whether the API needs any login beyond "confirm comes from a page click".
