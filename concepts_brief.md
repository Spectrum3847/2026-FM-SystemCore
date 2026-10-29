# FM Pit App on SystemCore

* **Author:** (pit app brainstorm, EGM)
* **Date:** 2026-10-29

## Overview

A web app that pit students and programmers open from a laptop or phone to run and fix FM. It shows how to drive the robot (controller layouts), guides tuning (swerve module offsets, gyro scale), lists the devices on the robot network with links to their own config pages, and downloads logs in one click. It keeps working when the robot program has crashed, because that is when people need logs and diagnostics most. An AI session, directed by a student or coach, uses the same app through a JSON API. A person must confirm any change to a hard-stop value (AGENTS.md).

The app is a separate small service on the SystemCore, installed by the normal `./gradlew deploy`. Its pages stay plain HTML and JavaScript. Live robot data goes from the browser straight to the robot's NetworkTables server; the service handles files, status and the API.

It is the one place a student starts. For every pit job it either does the job itself or says which tool to use and links to it, so nobody has to learn where each tool lives.

## Analysis

Six independent reviews (second round, run without steering), merged by question. Options are not chosen; the team decides.

Reviewers: Design Critic (DC), Implementation Readiness (IR), Zoom Out (ZO), Comprehension/UX (CX), Structural (ST), Codebase Details (CD). All six completed. This document lays out the options for each question and does not choose answers. The team decides.

### The point all six agree on

Every reviewer found the same blocker: the current app is served by `WebServer.start(5800, ...)` inside the robot program (Robot.java:294, and only when `hasHardware()` is true). So it goes down when the robot program crashes, which breaks a decided requirement. All six want the app to run as a separate process, managed by the OS, that deploy installs or restarts. CX and ST put it this way: "robot program up" should be a data source the app can lose. The app splits into a crash-proof layer (static pages, logs, files, network, controls guide, robot-program status) and a live layer that needs the robot program (NT data, tuning). CX adds that the UI should always show which layer is up and why a feature is greyed out.

---

### 1. Service language/runtime and page approach

**Service options**

| Option | Backed by | Strongest argument | Strongest counter |
|---|---|---|---|
| Standalone Java service (second JVM, separate from the robot program) | ST (preferred); ZO (if python3 is absent); IR ("prefer a runtime already on the device") | The JDK is already on the device and the team knows Java, so there's no new toolchain or runtime to keep alive across image updates. | A second JVM costs about 100-200 MB of RAM plus startup and GC jitter (DC, CX). This hasn't been measured. |
| Python | ZO (only if a real SystemCore has python3) | Easiest for AI and students to maintain; no build step. | python3 may not be installed. Installing it or packages at deploy time breaks the 10-15 s budget and adds a dependency outside our control (DC, IR, CX, ST). |
| Static Go binary | CX (scored it best); DC and ST name it as the counter-option | About 10 MB, no runtime dependency, low CPU and RAM, survives crashes. | New language for students. Weaker fit for AI maintainers (DC). Adds a cross-compile toolchain (ST). |

The reviewers agree on the selection criteria: nothing to install on laptops, no package manager on the robot or laptops at deploy (no npm, no pip), deploy within 15 s, CPU use measured, maintainable by AI. CX wants each candidate scored against these explicitly.

**Page options**

All reviewers who addressed pages agree (ZO, CX, ST, plus IR on build-step cost): plain JS with no build step. ZO and CX suggest JSDoc with `// @ts-check` and `node --test` to get some type safety. ZO says keep the existing pieces (`lib/nt4.js`, `lib/wpilog.js`, `log-loader.js`, Chart.js, the existing pages, `tools/app-test`) rather than rewrite them.
- Strongest argument: no deploy cost, and it avoids the stale-build problem the old Node/Vite app had.
- Strongest counter: plain JS has no real type checking for AI maintainers. JSDoc covers most of it, not all.

**Facts to check:** whether python3 is present on a stock SystemCore; idle CPU and RAM headroom with the robot loop running; the second JVM's actual RSS and its effect on loop jitter (measure with `tools/robot_top.sh`); clean and incremental deploy times for each candidate.

### 2. Shortcut on SystemCore's built-in web page

| Option | Backed by | Argument | Counter |
|---|---|---|---|
| Add a link if the built-in page supports extensions | ZO, CX (check first) | One obvious entry point for students. | It may not support extensions at all. |
| Patch the built-in page's files and reapply on every deploy | IR (optional) | Works even with no extension mechanism. | OS or firmware updates undo it. ZO says don't patch firmware files. |
| Don't depend on it: bookmark plus QR sticker, with an IP fallback | DC, ZO, IR, CX | Always works. | Not discoverable from the place students already look. |

CX adds that `.local` (mDNS) names fail on some laptops and phones. Publish one URL plus an IP fallback.

**Facts to check:** whether the built-in page can be extended; whether it owns port 80; whether it survives OS updates; whether `.local` resolves for phones on the radio network.

### 3. File transfer

There is broad agreement on the shape: read and download from an allowlist of folders (logs, deploy, system logs); no write access to the deploy jar, code, config or the service itself; any upload goes only to one staging or scratch folder (DC, IR, ZO, CX, ST).

| Variant | Backed by | Argument | Counter |
|---|---|---|---|
| Read-only in v1, no upload at all | ZO | This is what WinSCP is actually used for today, and write access would bypass the hard-stop rules. | Quick pit edits aren't possible. ZO's answer: those should go through deploy. |
| Upload to a staging folder only; disabled or gated while the robot is enabled; delete only while disabled; every write logged | DC, IR, CX, ST | Covers real transfers while keeping the risk bounded. | More surface area. Also, a file written by the service doesn't appear in replay unless it is also logged (IR). |

Additional requirements raised:
- Throttle downloads while the robot is enabled to protect disk I/O and radio bandwidth (DC).
- Show file size and hash, support resumable and range downloads, and mark the log currently being written (CD).
- Show free disk space (CX).
- ST: the service may write only inert files in an allowlisted folder, which the robot reads at init.

**Facts to check:** the disk layout and log path (e.g. /U/logs); which folders the systemcore user can write (logs, override folder); disk I/O cost of a download while the robot is enabled.

### 4. Where the AI interface lives, hard-stop enforcement, login

| Option | Backed by | Argument | Counter |
|---|---|---|---|
| One API on the robot side, used by both the UI and AI; the laptop MCP or CLI is a thin client over it | ST, DC, CX | Hard-stop enforcement lives in one place and can't drift between two paths. | It's a new robot-side surface to build and secure. |
| No separate AI API: AI uses the same read-only HTTP endpoints and NT4 that the browser uses, plus the existing laptop tools (spectrum-logs MCP, tools/*.py, frc-log-triage) | ZO | Least to build, and it reuses what already works. | It's unclear how AI would request changes and how confirmation would be enforced for them. |

**Hard-stop confirmation.** All reviewers agree it must be enforced in code, not left to an AI's prompt. CX notes an AI could simply type "confirm" itself. The proposed mechanism (DC, CX, IR, ST, CD):
- The robot side returns a pending change showing old value, new value and failure mode.
- A person approves it in the browser with a step an agent cannot complete (a token or a PIN).
- Changes apply only while disabled, through NT commands handled on the main thread, and are logged as inputs.
- DC adds that hard-stop values should then be written to repo JSON or a PR, never applied live.

**Login.**

| Option | Backed by | Argument | Counter |
|---|---|---|---|
| No login | ZO | The robot side already enforces safety, and the service is read-only. | Anyone on the field network can use the app. |
| Shared pit PIN for anything that writes | DC | Cheap protection against stray clients. | A shared secret leaks easily and has to be kept out of code (see AGENTS.md). |

CD wants every API or MCP call tagged with a session or client ID and logged (no secrets in the log), with separate endpoints for reads and changes.

**Facts to check:** whether a browser-only approval step can be made that an agent driving a browser cannot complete.

### 5. Live data: direct NT or relay

| Option | Backed by | Argument | Counter |
|---|---|---|---|
| Browsers connect directly to NT4 (`nt4.js` already does this); add a relay only if measurement shows a problem | ZO | Exists already; nothing new to build. | Each browser adds load to the NT server inside the robot JVM, which hurts the robot loop most (DC). |
| The service holds one NT subscription and relays throttled updates (10-20 Hz), with a cap on clients | DC; CD wants the relay rate exposed if a relay is built | Load on the robot stays the same however many phones, laptops or AI sessions connect. The relay can also say clearly when the robot program is down. | Extra code, plus CPU in the service. Premature without measurement. |

CD and CX both want every live value to show its data age, and diagnostic widgets to grey out when the device's `connected` input is false (the TXMAN1 lesson).

**Facts to check:** NT server CPU cost per client in the robot JVM; how many clients typically connect in the pit.

### 6. How far to replace each tool

**The team's decision:** replace other tools where possible, not only link to them.

**Dissent from reviewers:** DC, ZO and ST (ST mildly, as a nit) recommend linking out by default. In that view the app replaces only WinSCP-style jobs (log download, file view) and the device-link pages, and launches AdvantageScope, Elastic, Tuner X and the Limelight, radio and Jetson pages with a "which tool for which job" line.
- Their argument: replacing those tools is open-ended, other people maintain them, and a launcher plus how-to meets the teaching goal at about 5% of the cost (ZO).
- Counter (ZO's own, and the team's position): students lose context every time they switch tools, and having to know many tools is the teaching problem the team wants to solve.

Points that hold under either approach:
- CX: the home page should be a list of pit jobs, not a list of tools. Each job leads to an in-app page or to the outside tool, with one line saying why. Every tool that isn't replaced gets an explicit "use X for this" entry.
- ST: list out-of-scope items by name.
- IR: when Power, CAN and Cameras are rebuilt, run the old and new versions side by side until the new ones match. Remove the old pages in a separate PR, and never do the cutover on an `event*` branch.

**Facts to check:** for each tool, which pit jobs students actually use it for, and where it fails in practice. Replacing a tool is justified most clearly where it fails (WinSCP, per ST).

### 7. Source of the controls guide

There is near agreement (DC, ZO, CX, ST): generate the guide from the code's own button bindings. Two ways to do that:
- **The robot publishes its bindings at runtime in a manifest** (DC, CX, ST). This always matches the running code. The counter is that it is unavailable when the robot program is down, so ST adds a deploy-time snapshot as a fallback.
- **Generate the guide at build time, with a CI check** (ZO). This works even when the robot program is down. The counter is that it may need parsing Java, which the old app did and which went stale. ZO says never parse Java in the browser.

ZO also offers writing the guide once by hand. That is the simplest option, and the most likely to go stale.

**Facts to check:** whether the binding code can enumerate its own bindings with useful descriptions.

### 8. How changes reach the robot safely (AdvantageKit rules)

All reviewers who addressed this agree on the rule: changes that alter robot behavior go through NT commands handled on the robot program's main thread, only while disabled, logged as inputs (IR, ZO, ST, DC, CD). This matches docs/robot-app-plan.md §4. The service never writes the deploy folder or code directly.

Additional requirements raised:
- **Where app-written data lives (IR):** outside the deploy tree, so a deploy doesn't overwrite it. Each file carries a schema version and the git SHA that wrote it. Code rejects unknown schemas and falls back to compiled defaults (logged as an input). A Gradle "pull" task brings values into git, and deploy warns about values not yet pulled.
- **Visible tuning state (CX):** each value shows whether it is applied, pending until disabled, or saved on the robot but not in git. A persistent banner reads "N robot-only changes not in repo".
- **Audit trail (CD):** every change, including rejected requests and the confirmation itself, is recorded twice: as an AdvantageKit input (key, old value, new value, origin, confirmation ID) and in an append-only service journal that survives crashes.
- **Undo (IR):** every change has one.

**Open tension:** the AdvantageKit rules require inputs to be logged from the robot's main thread, yet the service journal sits outside the robot program. Decide which record counts for replay (it should be the AK input) and what the journal is for.

### 9. Facts to check on a real SystemCore before design

All six asked for these:
- Whether python3 is installed by default.
- Whether the systemcore user can register a systemd unit (system or user), and whether that unit survives reboot, `./gradlew deploy`, robot program updates and OS updates. Also whether OS updates wipe `/home/systemcore`.
- Free ports. In particular, port 5800 collides with Limelight USB port forwarding at 5800-5819 (`LimelightHelpers.java:2411-2424`; DC, ZO). Also the ports used by NT (5810) and the built-in page (80?).
- Whether the built-in web page can be extended.
- A CPU, RAM and loop-jitter baseline with the robot enabled, then the same with each candidate service running (`tools/robot_top.sh`).
- Log paths (/U/logs?) and write permissions.
- Whether `.local` resolves on phones and laptops on the radio network.
- Whether journald or syslog persists across reboots, and a writable persistent path for the service journal (CD).
- Deploy time on a clean device and incrementally, before and after adding the service (IR, DC).

---

### Issues raised outside the nine questions

- **Port collision (blocker, DC):** port 5800 overlaps Limelight USB forwarding. Covered under Q9, and it needs a decision regardless.
- **App and robot code drifting apart (IR, ST, CD):** the service reports its build hash and the robot's `BuildConstants` git SHA, dirty flag and build time. The UI shows any mismatch prominently. Deploy restarts the service when its version changes (`CommandArtifact` pattern), reinstalls it if it is missing, and fails loudly if the restart fails. Behavior on a never-deployed SystemCore needs defining. CD adds showing when the code was deployed, by which laptop and from which branch.
- **Robot liveness and diagnosis (CD blocker):** the service shows the robot program's state (systemd status, last exit code, time since restart) and the tail of its journal. This lets students tell apart "crashed", "never deployed" and "app down".
- **Robot facts copied into the app (ST blocker):** one source of truth per fact. A manifest the code publishes (plus a deploy-time snapshot) for robot facts; LogKeyGuide for log keys; discovery for network devices; the hard-stop list in one machine-readable file that both AGENTS.md and the enforcement code reference. DC warns specifically against network help turning back into a hard-coded IP list.
- **Log and NT key contract (IR):** the robot publishes a versioned key manifest built from LogKeyGuide. Pages read keys only through `lib/log-model.js`, tolerate missing keys and list unknown ones. A CI test runs the pages against committed logs from earlier builds.
- **Client architecture (ST):** a single client data layer in lib/ with three adapters (NT, service, device proxy). Pages use only the adapters. Test fixtures replay recorded logs.
- **Resource budget (DC, CD):** set a hard budget, for example under 5% of one core and under 150 MB. Enforce it with nice and cgroups. The service reports its own CPU, RSS and client count, and these are correlated with loop overruns.
- **Network page (CD, CX):** discover devices with mDNS, ARP and probes. Keep a timestamped, exportable history of reachability and latency. List each device as found or not found, with its address.
- **Home page (CD, CX):** show what is available right now: robot code up, NT connected, log folder present, each camera reachable.
- **Glossary (CX, nit):** terms like "hard-stop values", "gyro scalar", "TXMAN1" and "LogKeyGuide" need definitions, or links to AGENTS.md and docs/log-keys.md.

## Goals vs. Non-Goals

* **In-Scope Goals:**
  * A separate service on the SystemCore that survives robot program crashes, installed and restarted by `./gradlew deploy`, adding no more than 10–15 s.
  * One-click log download from laptop and phone.
  * Status page: robot program up or down since when, versions, last log.
  * Network page: devices found from NetworkTables and the neighbour table, reachable or not, links to their config pages.
  * Controls and button layouts, and how to use the robot.
  * Guided tuning procedures (swerve offsets, gyro scale), applied through logged NetworkTables commands while disabled.
  * An API that AI sessions directed by a person can use; a person confirms any hard-stop change.
  * A shortcut from SystemCore's built-in web page, if possible.
  * Viewing system files and transferring files.
  * A "How do I…" page that names the tool for every common pit job and links to it.
  * Replacements for the common case of other tools: log download and file view (WinSCP), device status (Limelight, radio, Jetson pages), motor health (Tuner X), a quick last-match power and CAN summary (AdvantageScope).
* **Out-of-Scope Non-Goals:** none set yet; see Open.

## Alternatives Weighed

* **Keep serving from the robot program's `WebServer` (the current first version).** Rejected: goes down when the robot program crashes, which breaks a hard requirement.
* **Node, npm or Vite front end.** Reviewers advised against it: laptop toolchain and cached old builds were the old app's documented failures. Not yet decided by the team (see item 4).
* **Off-the-shelf servers (Cockpit, filebrowser).** Rejected: need ARM builds on the device, Cockpit uses noticeable CPU, and neither offers hard-stop confirmation.
* **Link to existing tools instead of replacing any (all six reviewers).** Rejected by the team: too many places to learn. Kept in part: the app links out for the deep cases.

## Key Decisions & Staging for Next Steps

**Decided by the team:**
1. The app runs as its own service on the SystemCore, keeps working when the robot program crashes, and is installed by every robot deploy, adding no more than 10–15 s.
2. The app is the single starting point: it explains which tool to use for every pit job and replaces other tools where it can.
3. The existing Power, CAN and Cameras pages are rebuilt, with the current code as a reference, not a base.
4. AI sessions are directed by a person; a person confirms any hard-stop change.

**Reviewer recommendations, not yet approved by the team:**
1. The service runs under systemd, installed through a GradleRIO `CommandArtifact` that restarts it only when its files changed.
2. Remove `WebServer.start` from `Robot.java` when the service replaces it.
3. The browser reads live data from NetworkTables directly; the service does files, status, network and the API.
4. Writes only through logged NetworkTables commands while disabled; hard-stop changes are proposed through the API and confirmed by a person's click on a page.
5. The app keeps no copies of robot facts; a robot manifest with a git SHA, plus a mismatch banner.
6. Build log download and the status page first.

**Open (for the new review, then the team):**
1. Service language (Python, Java or Go) and page approach (plain JS or a build step).
2. Whether the API needs any login beyond "confirm comes from a page click".
3. Whether a shortcut can be added to SystemCore's built-in web page, and how.
4. File transfer: which directions, which folders, read-only or not.
5. What is out of scope (log analysis, device configuration, changes while enabled, others).
6. Where the agent interface lives: an API on the robot, a tool on the laptop, or both.
7. Whether the service relays live data to browsers, or browsers connect to NetworkTables directly.
8. How far the app replaces each other tool.
9. Where the controls guide gets its content.
10. Which bench checks are needed.

**Bench checks:** see Open item 10.
