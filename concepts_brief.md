# FM Pit App on SystemCore

* **Author:** (pit app brainstorm, EGM)
* **Date:** 2026-10-29

## Overview

A web app that pit students and programmers open from a laptop or phone to run and fix FM. It shows how to drive the robot (controller layouts), guides tuning (swerve module offsets, gyro scale), lists the devices on the robot network with links to their own config pages, and downloads logs in one click. It keeps working when the robot program has crashed, because that is when people need logs and diagnostics most. An AI session, directed by a student or coach, uses the same app through a JSON API. A person must confirm any change to a hard-stop value (AGENTS.md).

The app is a separate small service on the SystemCore, installed by the normal `./gradlew deploy`. Its pages stay plain HTML and JavaScript. Live robot data goes from the browser straight to the robot's NetworkTables server; the service handles files, status and the API.

It is the one place a student starts. For every pit job it either does the job itself or says which tool to use and links to it, so nobody has to learn where each tool lives.

## Verified facts (2026-10-29)

From Limelight's public SystemCore OS repo ([systemcore-os-public](https://github.com/LimelightVision/systemcore-os-public): README release notes and `package_examples/`). The image itself was not downloaded; release files are blocked from the session that checked.

- **Python is on the image.** Since OS alpha 11 / beta 11: "Add venv support and pip support to system python" and "Add common python package dependencies (gunicorn, flask-sqlalchemy, requests)". The app targets a future 2027 robot, so what matters is the latest SystemCore OS (alpha 18 / beta 18 at the time of writing), which includes it.
- **SystemCore has its own package format for add-on services.** An `.ipk` package with a `control` file, a systemd unit and a `postinst` script that enables and starts it. Control fields include `X-Port`, `X-Has-UI`, `X-Auto-Start` and `X-Icon-Path`; the built-in web UI shows installed packages as tiles. Both official examples are Python `http.server` services with `Restart=always`. This may answer the shortcut question (a package tile with a UI link) without patching SystemCore's page; how the tile links to the UI is not yet confirmed.
- **Installing a package stops the robot program** and restarts it after (alpha 18 notes). Effect on deploy time unmeasured.
- **The built-in web UI already covers some pit jobs:** per-service "Download Log", a support bundle with optional robot logs, CAN bus utilisation and warnings, a cameras portal (`robot.local/cameras`, `robot.local/camerastreams`), ports and firewall views, storage use, and a live robot program log.
- **Known ports** (from a third-party teardown): 4800 web UI, 4803 package manager, 4804 update manager, 4900 VS Code server, 4901 web terminal, 5800 CAN sniffer, 5812 NT4.
- SystemCore has 4 GB of RAM (team).

## Params system (verified in this repo, 2026-10-29)

From `src/main/java/frc/spectrumLib/params/`, `spectrumLib/README.md` and `README.md` ("Params").

- **Layers, later wins:** the default declared in code (`Params.group(...)`), then `src/main/deploy/params/<group>.json` (git), then `params/robots/<robot>/<group>.json` (git, per robot), then the robot-local `/home/systemcore/params/<robot>/<group>.json` (survives deploys), then live changes.
- **Live changes:** only with `Constants.TUNING_MODE` on, under NT `/Tuning/`, applied on the main thread only while disabled with no FMS attached; a change made on the field waits (`Params/PendingLive`). `TUNING_MODE` is `false` today and a test fails a PR to `main` that leaves it on.
- **Saving:** the `/Tuning/SaveParams` button writes values that differ from git to the robot-local folder, disabled only, on a background thread.
- **Replay:** every value and the layer it came from are logged inputs (`Params/Values/*`, `Params/Sources/*`).
- **Hard stops:** the `swerve` (CANcoder offsets, read once at boot, so a change needs a save and a reboot) and `soft-limits` groups carry a `$hardware` stamp; a file from another chassis is refused and git values are used.
- **Groups today:** `cameras`, `swerve` (per robot), `soft-limits`.
- **Not built:** nothing pulls saved values from the robot back into git. `pullTuning` and the deploy check exist only in `docs/robot-app-plan.md`. Saved values stay on the robot until someone copies them by hand.

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

### Round 3: language, packaging, shortcut, built-in UI overlap

Six new reviews with the verified facts above. Options are not chosen; the team decides.

Reviewers: Comprehension/UX (CX), Codebase Details (CD), Implementation Readiness (IR), Zoom Out (ZO), Structural (ST), Design Critic (DC). All six completed. The brief was concept3.md, which added these facts: system Python is on the image, SystemCore uses an .ipk package format that shows as tiles, installing a package stops the robot program, the built-in web UI on port 4800 has its own features, there is a known port list, and the target is the latest OS on a 2027 robot. This document lays out the options and does not choose answers. The team decides.

### Agreement carried over from round 2 (still unanimous or nearly so)

- **Crash survival:** the app must run as its own systemd service, not inside the robot program (Robot.java:294). CX, ZO, ST, CD and DC say this outright.
- **Pages:** plain JS modules with no build step. Keep the existing `lib/` (NT4, wpilog, log-model) and `tools/app-test` as the shared layer (CX, ZO, ST, DC).
- **Changes:** go through NT commands handled on the robot's main thread, only while disabled, logged as inputs. Hard-stop changes need a person to confirm, showing old value, new value and failure mode. The service never writes robot config (all six).
- **AI parity:** every capability is a documented HTTP/JSON (or NT) endpoint, and the pages are clients of that same API. ZO says no separate MCP layer for now (CX, CD, ZO, ST, DC).
- **Version check:** the service reports its own SHA next to the robot code's SHA, and the UI shows a loud mismatch banner (CX, CD, IR, ST, DC).
- **CPU budget:** measure loop overruns with and without the service, then set a budget and enforce it with systemd limits (Nice, CPUWeight, MemoryMax, IOSchedulingClass=idle) (CX, CD, IR, DC, ZO).

### What changed since round 2

- **Python:** in round 2 its availability was unknown and Python was a minority choice. Now that system Python is on the image, Python is the leading option: ZO, ST, DC and CX lean to it. Java has fallen to the counter-option, and Go is mentioned only by DC as a counter.
- **Port:** the clash is now known to be three-way. The CAN sniffer also uses 5800, alongside the first-cut app and Limelight forwarding. Every reviewer calls a new port a blocker or major issue.
- **Packaging:** a new question. Reviewers converge on installing the service rarely and copying the page files on every deploy.
- **Overlap with the built-in UI:** in round 2 the "link out" advice was aimed at AdvantageScope, Tuner X and similar tools. In round 3 the same lean is aimed at the 4800 UI: all six default to linking to it.
- **Live data:** DC now explicitly says browsers connect straight to NT4. No one argued for a relay this round.

---

### Q1. Service language/runtime and page approach

| Option | Backed by | Strongest argument | Strongest counter |
|---|---|---|---|
| System Python, standard library only (or the Flask already on the image); no pip or venv at deploy | ZO, ST, DC; CX leans this way | It's on the image and matches the official package examples. About 20-30 MB of RAM and millisecond startup (ZO). It can reuse `tools/*.py`. | Limelight controls the Python version, and an OTA update could change it, so pin and test (DC). If pip/venv were ever needed, an OS update could break them (ST). CX notes it needs pip/venv handling if you go beyond the standard library. |
| Second JVM (Java) | Counter-option only (ZO, ST, CX) | Same language as the robot code. Could reuse WPILib's wpilog and NT classes (ST). | Costs 100-200 MB of RAM and seconds of startup. ST adds that log parsing already happens in the browser, so the reuse gains little. |
| Go static binary | Counter only (DC, ZO) | Doesn't depend on the OS's Python. | Adds a toolchain. |

**Pages:** all reviewers agree on plain JS with no build step. DC raises TypeScript, for type safety with AI maintainers, as the counter.

**Facts to check:** the Python version on alpha/beta 18, and whether it changes across OTA updates; whether the standard library (or the bundled Flask) covers range requests and streaming; the service's RSS and CPU on a real device.

### Q2. Packaging and install on each deploy

| Option | Backed by | Strongest argument | Strongest counter |
|---|---|---|---|
| **Split by what changes.** Install the .ipk (service plus systemd unit) only when the service's hash changes. Every deploy copies page and data files with no restart. | IR (option a), ST, DC, CX | An ordinary deploy pays no package-manager time and causes no extra robot restart. The tile and the package manager know about the app. | Two install paths can drift: an old service serving new pages (DC). It also ties the team to the alpha package-manager API on port 4803 (ST). |
| **Gradle-installed systemd unit.** One more `CommandArtifact` copies `app/` and the unit file and runs `systemctl enable --now` or `restart`. Build an .ipk only if the tile needs one. | ZO; IR (option b: install the unit once, then replace files and restart only that service) | Simplest. Versions match because the same deploy ships both. ZO points out that GradleRIO already stops the robot program before any artifact runs (build.gradle:290), so the restart that package install causes adds nothing during a deploy. | A hand-installed unit is outside the package manager: an OS reflash may wipe it, and the built-in UI won't know about it (ZO's own counter). No tile. |
| .ipk installed on every deploy | No one backs it | One mechanism. | Restarts the robot program, includes postinst sleeps and daemon-reload, and adds unmeasured time. A failed install leaves versions mismatched (IR, ST, DC). |

**ZO's point and the round-3 consensus.** ZO's observation (build.gradle:290) weakens the main argument against using .ipk at deploy time. The consensus is still not to reinstall the package on every deploy.

**Shared requirements under any option:**
- Stamp the service and the pages with the same build ID. Deploy prints both IDs, says whether it reinstalled and how long it took, and fails loudly if the app install fails (CD, CX, IR).
- Keep the previous app version on disk and swap a symlink for rollback (IR).
- Set StartLimitBurst to stop crash loops (DC).
- Deploy checks the OS version and fails clearly, or falls back, if the OS is too old (IR).

**Facts to check:**
- Deploy time on a real SystemCore when nothing changed and when the service changed (IR).
- What package-manager install costs.
- Whether a reflash or OTA removes `/etc/systemd/system` units and installed packages (ZO, DC).
- Whether GradleRIO's stop of the robot program really covers the package install.

### Q3. Shortcut from the built-in web UI

There is near agreement: the `X-Has-UI` and `X-Port` fields strongly suggest the package tile links to host:port, but nobody has confirmed it (ZO, CX, ST, DC, IR).
- **Way to settle it:** install the official hello-world-autostart .ipk on a bench SystemCore running alpha/beta 18 and see whether the tile links (ZO, DC).
- **Related fields:** CX and ST note the README also documents `X-Https`, which may change the link scheme or break iframes, and `X-Services`, which adds restart and stop buttons to the tile.
- **Fallbacks:** a QR code, a bookmark, a fixed URL such as robot.local/<name>, and a link from the Elastic dashboard (CX, ZO, DC, ST).
- **Strongest argument for the tile:** it is the official entry point and needs no patching.
- **Counter:** it is unverified, and it requires the .ipk route from Q2.

**Port:** all six agree on a dedicated port outside 5800-5819 and the other known ports, checked with `ss -ltnp` on the latest OS and declared in `X-Port`. Nobody types it: students reach it only through the tile or shortcut (CX). Pages use relative URLs (ST). Redirect the old URL for one season and update the docs that name it in the same PR (IR).

**Facts to check:** whether the tile links, and to which scheme, host and port; what `X-Https` and `X-Services` do; which ports are free on alpha/beta 18.

### Q4. Overlap with the built-in SystemCore UI

**The team's decision:** the app is the single starting place and replaces tools where possible.

**Dissent:** all six reviewers default to linking to the built-in features rather than replacing them. Their reasons:
- Limelight owns these features and they change with each OS release (DC).
- The built-ins keep working when the robot program is down (CD).
- Rebuilding them is the biggest YAGNI risk (ZO).

DC states the counter itself: a task index that links out still shows students more than one UI, which is what the team wants to avoid.

**Embedding (iframes):** CX and DC advise against it, citing cross-port auth, X-Frame-Options, `X-Https` and OTA changes. ZO and ST would embed only if a check confirms the pages can be framed. Nobody recommends embedding by default.

**Replace, link or embed, per feature, as raised:**

| Built-in feature | What reviewers suggest | Notes |
|---|---|---|
| Per-service "Download Log", support bundle | Link to them (CD, ZO). The app also logs to stdout/journald so the bundle includes the app's log (CD). | |
| Robot log download (wpilog) | Replace, which is the unanimous exception (CX, IR, ZO, ST, DC). Add range/resume, checksum, size, a mark on the active log, no zipping of the active log while enabled, and show free storage (CD, DC). | ZO estimates about 30 lines of Python. Use documented OS paths with a version check (IR). |
| CAN utilisation and warnings | Rebuild the team's CAN page with AdvantageKit keys and FM's own device mapping. Link to the built-in utilisation view (ZO, ST, CD). | |
| Cameras portal | Link to it. The team's Cameras page adds calibration and mounts (ZO). | |
| Ports and firewall, storage | Link to them (ZO). Show free storage on the log page (CD). | |
| Live robot program log | Link to it from the robot-program status banner on every page (CX, ZO, CD). | |
| Web terminal (4901), VS Code (4900) | Link to them for file viewing and arbitrary writes (ZO, ST). | |
| Elastic package | Link to it, and link back to the app from Elastic (ZO, DC). | |

**Structure for the starting place:** CX (blocker) and ST want one routing table, kept as data, mapping each pit job to where it is done (an app page, a deep link into 4800, or an external tool) and why. The landing page is generated from that table. Jobs the app doesn't cover still get a row with a link and one sentence.

**Facts to check:** whether the 4800 pages can be framed; whether stable deep-link URLs into 4800 exist; which 4800 features have actually failed for the team, since replacing a feature is justified most clearly where it fails (CX, IR).

---

### Other issues raised

- **Crash-time feature set (DC, CX):** state explicitly what works while the robot program is down: logs, files, network, service status, program log. Live tuning does not. NT pages fall back to log and file views. Every page shows "robot program: running / crashed / not deployed".
- **Health endpoint (CD, DC):** `/health` reports app SHA, robot SHA, whether they match, uptime, restart count, robot program state and NT4 state. A compact status strip on phones (CD).
- **File security (DC, ST):** the official example runs as root. Run the service as the systemcore user or a dedicated user instead. Allowlist reads (logs, deploy, /var/log) and writes (the tuning folder only). Refuse writes while enabled or attached to the FMS.
- **Network discovery (CD, ZO):** for each device show last seen, address, discovery method, latency and last error. Bound and cache the probes, and pause them while enabled. ZO would keep it to a page of links plus one ping/mDNS scan.
- **Single key source (IR, ST):** key names come from LogKeyGuide / docs/log-keys.md, read at runtime. State how the CI check of `make_log_key_guide.py` covers the app. One published contract: the robot manifest over NT, plus LogKeyGuide, plus the HTTP API.
- **Rebuilt pages state data source and age (CD):** CAN, Power and Cameras pages show where each value comes from and how stale it is.
- **Thin service (ST):** keep the service "dumb". Logic lives in the page files, which are replaced on every deploy.
- **Never write Phoenix configs directly (ZO).**
- **Download load (DC):** measure loop overruns during a 200 MB download, and stream files in chunks.
- **Glossary (CX):** define hard-stop values, gyro scalar, SystemCore, Elastic, package manager on port 4803, and .ipk.

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
11. How the app edits params: through the existing `/Tuning/` NT transport and `SaveParams` (which needs `TUNING_MODE` on, today blocked on `main`), or another path; and how the pit gets saved values back into git automatically (no pull step exists yet).

**Bench checks:** see Open item 10.
