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

## Current position (final check round, with the full picture)

Six reviews that re-examined every earlier conclusion against all team decisions and the robot-specific framing. Where this conflicts with the rounds below, this section is current. Options are not chosen; the team decides.

Reviewers: Design Critic (DC), Codebase Details (CD), Zoom Out (ZO), Comprehension/UX (CX), Implementation Readiness (IR), Structural (ST). All six completed.

The brief was concept7.md, a final check. It re-examines earlier conclusions with the complete picture: the app is specific to one robot, its pages are hand-made, and it lives in the same repo and build as the robot code (decision 12). Round 1 had also been given a wrong CPU figure. This document lays out the reviewers' positions and does not choose answers. The team decides.

---

### 1. Safety problems (stated plainly)

#### S1. A page click does not prove a person confirmed a hard-stop change (blocker)

**Raised by:** DC, ZO, CX, IR, ST (five of six; all call it a blocker or a concrete safety problem).

**The problem.**
- NT and HTTP are unauthenticated (decision 5).
- An AI session can drive a browser and click "Confirm", and any NT client can publish the confirmation.
- So an agent can complete the whole hard-stop path alone, which breaks AGENTS.md's "a person confirms" and the intent of decision 4.
- ST adds that the tuning-mode button is unauthenticated too.
- CX notes this was raised as a fact to check in round 2 and never answered.

**Proposed fix (all five).** The page shows old value, new value and failure mode. The robot applies the change only after a physical driver-controller button (DC suggests a chord), pressed while disabled and read on the main thread as a logged input.
- An agent can't do it, it needs no secrets, it is visible in replay, and it reuses the driver-button model of decision 11.
- All five say decision 4 stands. Only how it is enforced changes.

**Dissent (CD).** CD keeps a page-only confirmation step that the robot checks and logs as an input, with a confirmation ID, old value, new value and failure mode. CD didn't address the browser-driving agent.

**Related point (ST).** Soft-limit capture (decision 11) writes a hard-stop group, so it needs this confirmation path too.

#### S2. Stale robot-local hard-stop values override newer git values (major)

**Raised by:** CX, IR.

**The problem.** The robot-local layer beats git (decision 7), and deploys only warn (decision 8).
- Suppose a swerve module is replaced and new offsets are committed to git. A stale robot-local offset still wins on the same chassis, because the `$hardware` stamp only rejects files from a different chassis.
- The warning scrolls past, and the robot drives with wrong offsets.

**Proposed fix.** Neither reviewer reopens the decisions.
- The robot logs at boot, as an input, whenever a local hard-stop value overrides a git value that changed after the save.
- A persistent banner shows this on the app and on the driver dashboard.
- IR notes this needs the per-key base value recorded, even though decision 8 never merges automatically.

**Conflict.** ZO says decision 8 makes three-way merge and base fields moot, so record only the git SHA. That conflicts with IR's need for a per-key base.

#### S3. Capture validity checked only on the page

**Raised by:** CD, as the only point where it sees a real safety risk.

A capture gate that runs only on the page could act on zeros after a CANivore drop (the TXMAN1 case) and leave no record in the log. All six now recommend the gate on the robot (see section 3).

**Related blocker (CD).** `nt4.js` passes `timestamp` to `onValue` (line 329) but doesn't store it per topic. So today no page can show how old a value is or reject stale data, and the TXMAN1 rule can't be enforced on the page side. ZO agrees the fix is to add per-topic last-update time to `nt4.js`.

#### S4. Other safety-adjacent points

- **Device config is read-only (DC, major).** Any "Tuner X replacement" must be read-only for device config. Phoenix config writes are blocking and close to hard-stop values.
- **The service must survive robot-code crashes (DC, CX, IR, ST).** If the service is Java, it must not depend on robot classes, the HAL, JNI or Phoenix. A robot-code bug or a bad native library must not take it down. ST wants a build check that enforces this.
- **Log tuning arm and disarm (CD, major).** Decision 5 has no timer, so log each arm and disarm (who armed it, why it turned off) as an input and show it in a banner. Today `TuningMode` is set only from the constant (Robot.java:173).

**No other safety problems.** CD and ST state explicitly that they found none in decisions 1-12.

---

### 2. Earlier conclusions: current verdicts

Legend: **H** = holds, **HC** = holds with changes, **SC** = should change.

#### 2.1 Service language and runtime (rounds 3-5 leaned to Python stdlib)

| DC | CD | ZO | CX | IR | ST |
|---|---|---|---|---|---|
| SC → Java | SC → lean Java, or keep it open and measure | HC (keep Python stdlib) | SC → Java | SC → Java | SC → Java leads, Python the counter |

**Tally:** 5 should change (4 firmly to Java, CD leaning Java), 1 holds with changes.

**Why it changes.** The round 4 objection was that sharing code would tie the service's release cycle to the robot's. Decision 12 already ties them, so the objection no longer applies (DC, CD, CX, IR, ST). Further reasons:
- **Shared code:** Java can share ParamStore JSON parsing, the robot-vs-git diff, the hardware-stamp rule, key constants, `BuildConstants` and the wpilog reader, rather than keeping copies in Python.
- **Version control:** Limelight's OS updates control Python's version. IR: an OTA update could break the service in a way no deploy can roll back. The team pins the JDK.
- **Languages for AI maintainers:** Python would be a third language alongside Java and JS.
- **RAM:** 100-200 MB out of 4 GB is about 3-5%, and that was never measured.

**ZO's dissent.** The robot-specific framing makes the service *thinner*, so it gains little from shared types.
- The only real pull to Java is decision 10's diff while the robot program is down. ZO's alternative: the robot writes the finished diff file on every `SaveParams`, and the service serves it as a static file. That keeps one parser.
- Keep to the standard library (no Flask), and add a bench smoke test for each OS release.

**Agreed by all six:**
- CPU and GC jitter are the real constraint, and neither runtime has been measured. Measure with `robot_top.sh` and `FullCycleMS` before choosing (CD, ST, CX).
- Go drops out (DC, CD, ZO; IR keeps it only as a fallback if JVM jitter turns out to be real).

**Changes asked for if Java is chosen:**
- **Separate jar (DC, CX, IR, ST).** Its own source set or subproject and its own jar, depending only on the JDK (`com.sun.net.httpserver`) plus a pure shared module (params schema, key constants, stamp rule). No robot classes, HAL, JNI or Phoenix, enforced by a build check (ST).
- **JVM caps.** `-Xmx64m` (DC, CX) or `-Xmx96m` (IR), `-XX:+UseSerialGC`, `-XX:TieredStopAtLevel=1`, plus systemd `Nice` and `CPUQuota`. DC adds `CPUAffinity` if jitter is measurable.
- **Stable JDK path (CX, ST, DC).** The launcher must find the JDK without hard-coding a version path that GradleRIO or OS updates would break.
- **Reproducible jar (DC).** Set `preserveFileTimestamps = false` and `reproducibleFileOrder = true`, and keep `BuildConstants` out of the service jar. Otherwise the jar's hash changes on every build and the service restarts on every deploy.
- **Self-reporting (CD).** The service reports its CPU, RSS, GC pauses and client count on `/health`.

#### 2.2 Pages: plain JS, no build step

| DC | CD | ZO | CX | IR | ST |
|---|---|---|---|---|---|
| HC | HC | H | HC | HC | HC |

**Tally:** 1 holds, 5 hold with changes. No laptop JS toolchain and no npm (all six).

**Changes asked for:**
- **Generated key module (DC, CX, IR, ST).** Gradle generates a JS module, or JSON files, from the Java declarations: keys, params with bounds and `requiresReboot`, procedure ids, button bindings, and the hard-stop list with failure-mode text. Pages import it, so a rename breaks `node --test`. CX says "no build step" should now mean "no laptop JS toolchain", not "no generation at all".
- **Type checking (all six).** `// @ts-check` plus JSDoc, checked in CI (CD). A per-page `KEYS` declaration (CD).
- **CAD assets.** Static SVG or PNG exported once per mechanism, in a folder per page (for example `app/pages/<page>/assets/`), with a size budget so the deploy stays within 10-15 s (DC, IR, ST, ZO).
  - Don't render CAD live in the page (ZO).
  - CX: if a renderer is used, it goes in `app/vendor/` as one file. Define an export procedure and who updates the assets.
  - CD: record the Onshape source and version next to each asset (`tools/onshape` exists).

#### 2.3 "No copies of robot facts / robot-published runtime manifest"

| DC | CD | ZO | CX | IR | ST |
|---|---|---|---|---|---|
| HC (narrowed) | HC | SC | SC | SC | SC |

**Tally:** 4 should change, 2 hold with changes. The substance is the same across all six.
- A hand-made page is a copy of robot facts by design.
- The new rule: no hand-typed *values* (keys, bounds, bindings, CAN or device lists, failure-mode text). Those come from generated files, and CI fails on drift, closed rather than open. Hand-written structure, step text and graphics are allowed.
- The runtime manifest drops to the SHA and version-mismatch banner, for partial deploys and after a crash.
- IR adds that a list generated at compile time also works when the robot program has crashed; a runtime manifest does not.
- The hard-stop list stays in one machine-readable place (ST).

**Drop the cross-repo machinery:** versioned contract documents and a schema-versioned shell solved skew between repos that no longer exists (ZO). CX and ST: rewrite or withdraw reviewer recommendation 5 in "Key Decisions".

#### 2.4 Linking to versus replacing the built-in UI and other tools

All earlier reviewers said "link". The team decided the app replaces tools where possible.

| DC | CD | ZO | CX | IR | ST |
|---|---|---|---|---|---|
| SC (earlier advice) | SC | HC | HC | SC (earlier objections lose weight) | SC |

**Tally:** 4 say the earlier "link" advice should change, 2 hold it with changes. In substance all six now agree with the team's decision.
- **Replace robot-specific jobs.** CAN and power by mechanism, mechanism health, this robot's cameras, procedures, log download.
- **Keep links** (deep links through the routing table) to Limelight-owned generic OS features: support bundle, service logs, ports and firewall, storage, web terminal, OS update. Rebuilding those adds no robot knowledge and ties the app to the alpha OS (ST, ZO, CX).
- **No dissent remains on this question.**

**Changes asked for:**
- Every replaced feature shows its data source and age, and links to the built-in view as a fallback, so the app isn't a single point of failure (CD).
- The QR sticker or bookmark keeps a fallback list of the built-in tools, in case the app itself is down (IR).
- Device config stays read-only (DC; see S4).

#### 2.5 Install (decision 6: launcher-only .ipk installed once, Gradle owns the folder, restart on hash change)

| DC | CD | ZO | CX | IR | ST |
|---|---|---|---|---|---|
| H (one gap) | HC (gaps) | H (missing items) | H | H (ordering blocker) | H |

**Tally:** all six hold the decision. Gaps raised:
- **Decide the runtime first (IR, blocker).** The launcher "never changes", so its interface (interpreter, path to `current/`, arguments, environment file) must be fixed before the first .ipk ships. Have the launcher read its exec line from a file in the Gradle-owned folder (IR), or run `current/run` (DC), so the runtime choice stays with Gradle.
- **Java jar hashes (DC).** The jar must be built reproducibly (see 2.1), or every deploy restarts the service.
- **Deploy output (CD).** Print the installed hash, whether the service restarted, and how long it took. `/version` reports the launcher version, the folder hash and the robot SHA. The launcher logs to journald why it failed to start.
- **Robot without the launcher (ZO, IR).** Print one line on how to install it, and continue.
- **Wording (ZO).** Decision 1 says "installed by every deploy" but decision 6 installs once. Reword it to "updated by every deploy".
- **Pages without restarts (CX).** Changed pages are served from disk with no restart.
- **Bench checks still gate the design (CD, CX, ST).** Can the deploy user run `systemctl restart` and `opkg`? Does the tile link to the app?
- **Rollback.** Deploying an older commit restores the matching app (ST, IR).

#### 2.6 AI interface (one documented API for pages and AI; a person confirms hard stops)

| DC | CD | ZO | CX | IR | ST |
|---|---|---|---|---|---|
| H + safety blocker | H | HC + safety blocker | H + safety blocker | H + safety problem | HC + safety blocker |

**Tally:** all six hold "one API". Five find the confirmation mechanism unsafe (S1).

**Changes asked for:**
- **Build "propose, then confirm" once** (DC).
- **Simplify (ZO).** The API is the service's read-only HTTP endpoints plus the `/Tuning/` NT topics the pages already write. No separate AI write path and no MCP layer.
- **Logging (CD).** Log each call with a client ID.
- **Failure-mode text (CD).** It still has no source, which blocks the swerve page. DC and the generated module (2.2) would take it from the param declaration.
- **API schema (IR).** A Java service could generate its schema from the shared records.

#### 2.7 Other earlier conclusions

| Item | Verdict and who | Change |
|---|---|---|
| NT relay (round 2) | Moot or drop: DC, ZO | Use direct NT4. |
| Controls guide | Holds, made clearer: DC, ZO | Generate it at build time from a binding registry or `Gamepad`. The same table drives the procedure pages' "press A" (ZO). |
| Network discovery with mDNS/ARP/latency history (round 2) | Should change: ZO | Over-built for one robot. Use one page listing the devices this robot's code logs, with reachability. |
| Three-way merge, base fields, `pending.json` schema (rounds 4-5) | Should change: ZO (record the git SHA only) | Conflicts with IR's per-key base, needed for S2. |
| Resource budget "5% / 150 MB" (round 2) | Should change: ZO | Came from the wrong CPU figure. Drop the numbers; measure, then set `Nice` and `CPUQuota`. |
| Round 2 client data layer with three adapters | Should change: ZO (YAGNI) | `nt4.js` and `log-loader.js` already are the adapters. Add per-topic timestamps only. |
| Round 4 ST "service and robot share no code" | Should change: DC, CD, IR, ST | Superseded by decision 12. |
| Python pip/venv drift, "reuse tools/*.py" | Unnecessary: IR | Only relevant if Python stays. |
| "Land the shared framework before the season" (round 6) | Holds with changes: DC | Conflicts with "built once the robot exists". Land only the core (service, log download, status, generator) before the season. Pages come later. |
| Retire the 5800 `WebServer` pages | Missing: IR, CD | Redirect, keep the old pages side by side until each new page passes its sim, then delete. |
| Event branches | Missing: IR | The smallest-change and no-spectrumLib rules now apply to app code too. Keep the shared framework outside spectrumLib so it can be fixed at an event. |
| What carries over to the next robot | Missing: ZO, CX | The service, `lib/` and the Java procedure and capture pieces carry over; pages are rewritten. CX: give it an owner and a trigger, e.g. "at kickoff, copy `app/core` and the service source set". |

---

### 3. The round-6 splits

#### 3.1 Where the shared part lives

**All six now say this repo, copied forward, extracted later.** That resolves round 6's three-way split. The remaining disagreement is where the **Java** half goes.

| Reviewer | Recommendation | Counter it gives |
|---|---|---|
| DC | `app/core` plus a Java `app-common` source set, not spectrumLib, because of the `event*` ban on spectrumLib edits. Extract when robot two exists. | Fixes won't reach the next robot. Mitigate with `VERSION` and a changelog. |
| CD | `app/core` plus spectrumLib Java, with `VERSION` and a contract document. | Copies drift. Acceptable for a season, since spectrumLib is already copied per repo. |
| ZO | `app/lib` plus `spectrumLib/checks`, next to `Check`/`PreMatchChecks`. Extract only when two robots are live at once. | Copies drift. Acceptable: robots are about yearly and pages are rewritten. |
| CX | `app/core` plus the service source set, with no mechanism names (test-enforced). | Fixes don't flow back. `VERSION` and a changelog of core fixes to port. |
| IR | A Java service-and-procedure package plus `app/core`, outside spectrumLib so it can be fixed at an event. | Drift, accepted under decision 12. |
| ST | `app/core` plus Java `procedures/`, with a CI purity test (no key literals or mechanism names in core). Move to spectrumLib off-season. | Drift. The purity test keeps later extraction cheap. |

**Split on the Java half:**
- **In spectrumLib now:** CD, ZO.
- **Outside spectrumLib during the season:** DC, IR (because of the event-branch ban), and ST (until the off-season).
- **In the service source set:** CX.

#### 3.2 Where capture validity is checked

**All six: on the robot.** This resolves round 6's split.
- **The gate:** a single `CaptureGate` (ZO: on the `Check` pattern). It produces a logged verdict and reason, computed from logged inputs: connected for the whole capture window, fresh, and steady. The page shows only the reason.
- **Why round 6's counter fell away (DC, CX, IR, ST):** under decision 12 a page fix and a Java fix ship in the same deploy, so iteration speed no longer favours the page.
- **Remaining counters and their mitigations:**
  - The reason text lives in Java, far from the page. Fix it with enum reason codes mapped to text in the generated module (DC).
  - A threshold tweak needs a redeploy. Make the thresholds params (ZO).
  - A bug in the gate blocks every procedure until the next deploy (IR).

#### 3.3 Who selects the utility-mode test

**Split 3 to 3.**

| Reviewer | Recommendation | Counter it gives |
|---|---|---|
| DC | **Driver buttons only.** The robot shows the selection on the dashboard and needs a second press to start. The page shows the expected button and accepts results only after `Seq` advances. | A wrong button runs the wrong test, limited by the two presses and timeouts. ZO's "requested test" field is the fallback if pit trials show errors. |
| ZO | **Driver buttons only.** Each test on a distinct, labelled button, with rumble and the test name shown at start. | The wrong mechanism moves. Add the logged "requested test" (read only while disabled, selects and never starts) if it happens in practice. |
| ST | **Driver buttons only for v1.** The page checks `Procedure/Active` after the test starts. | Wrong button. Same fallback. |
| CD | **The page requests a test while disabled, as a logged NT input.** The robot accepts it only while disabled with no FMS attached. Only the driver's button starts it. | Indirect selection of motion, but within decision 11. Buttons-only also works if the robot logs the test id and `Seq`. |
| CX | **The page selects while disabled, as a logged command.** One driver button runs only the selected test. The selection clears on leaving utility mode or when an FMS attaches. The driver must see it on the dashboard first. | Selecting from the web, or by AI through the API, is indirect control of motion. |
| IR | **The page selects while disabled, as a logged command that never starts.** Cleared on FMS attach or leaving utility mode. Dashboard display plus rumble. | Indirect control of motion. Kept in check by the button press, the visible selection and the timeout. |

**Round-6 option dropped:** "page-only arming", where nothing is sent to the robot, is no longer backed by anyone. IR moved from it to page-selects.

**Common ground:** all three page-select reviewers require that selection never starts motion. All three buttons-only reviewers accept page selection as a fallback.

**Interaction with S1 (DC):** with the physical hard-stop confirm, every action that changes the robot stays at the controller. That argues for buttons-only.

---

### 4. Document fixes the reviewers asked for

- **AGENTS.md "Robot app" section** (DC blocker; ST major). It says "find mechanisms, buses and cameras from the log or NT, never a list" and "no CAN IDs in app/", and it still names port 5800. This contradicts decision 12, and AI maintainers will refuse to build hand-made pages or will fight them. Rewrite it to allow generated lists and forbid hand-typed values.
- **Overview** (CX). Put the robot-specific framing in the first paragraph. Today a newcomer reads it as a generic app until decision 12.
- **Goals** (CD, ST). They still describe a generic, discovery-based network page and a "How do I" page. Align them with decisions 11-12, and say which jobs get robot-specific pages and which stay generic. Mention the hand-made procedure pages and CAD.
- **Glossary** (CX, ST; asked for since round 2). Define hard-stop, `$hardware` stamp, utility mode, eventDeploy, TXMAN1, LogKeyGuide and capture.
- **Pruning:**
  - Mark open item 1 (language) and the Alternatives "Node... not yet decided" as stale or under review (CX).
  - Prune open items 1, 7, 9 and 11, which decisions 5, 7 and 12 partly settle (ST).
  - Cut the "implementation details still undecided" list (ZO).
  - Mark round 4 ST's "share no code" as superseded (CD).
  - Withdraw or rewrite reviewer recommendation 5 (CX, ST).
  - Drop the round 2 resource numbers (ZO).
- **Current-position table** (ST). Rounds 2-6 repeat each other and conflict. Add one table (question, decided or open, which round superseded which) and archive the rounds below it.
- **Out of scope** (CX). The section is empty. Candidates: generic mechanism discovery, OS functions Limelight already ships, and editing values while enabled.
- **Decision 1 wording** (ZO). "Updated by every deploy", not "installed".
- **Recommendation 2, removing `WebServer.start`** (CD, IR). Add the side-by-side migration step.
- **Next-robot carry-over** (CX, ZO). Name an owner and a trigger.

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

### Round 4: params, tuning and the path back to git

Six new reviews with the params system added. Options are not chosen; the team decides.

Reviewers: Design Critic (DC), Codebase Details (CD), Zoom Out (ZO), Comprehension/UX (CX), Implementation Readiness (IR), Structural (ST). All six completed. The brief was concept4.md, which adds the params system and the planned "pull back to git" step. The team intends the app to tune param values through these files if the reviewers agree. This document lays out the options and does not choose answers. The team decides.

### Summary of agreement

- **Edit path (Q1):** all six recommend that the app edit values only through the existing `/Tuning/` NT transport and `SaveParams`, with the robot program as the only writer of robot-local JSON. That is the reviewers' answer to "if the reviewers agree": they agree with tuning through params, by that path.
- **TUNING_MODE (Q4):** all six recommend replacing the compile-time constant with a runtime gate the robot enforces, off by default, and changing the CI test to check that default.
- **Runtime (Q3):** all six say params do not push the service towards Java. The service gets thinner, which strengthens the Python choice.
- **Back to git (Q2):** all who addressed conflicts want the save to record its base and the pull to report conflicts rather than overwrite (DC, IR, ST, ZO in a simpler form). All want hard-stop values kept out of unreviewed auto-commits on `event*` branches (DC, ZO, CX, IR, ST).

### What this round changes from earlier rounds

- **Language:** round 3 already leaned to system Python. Round 4 makes the lean stronger and unanimous. If the robot program owns all param logic, the service is only a file, log and network helper plus an NT viewer (ZO, ST, DC, IR, CD). Java stays the counter-option, for one language across the code and reuse of WPILib's DataLog reader (ST), or sharing ParamStore parsing (CD). ST warns that sharing code would tie the service's release cycle to the robot's.
- **Packaging:** round 3 converged on "install the service only when it changes". Round 4 keeps that, but the vehicle splits:
  - **Gradle-installed systemd unit, gated on a hash:** DC, ZO and IR. They use .ipk only if the tile proves worth it.
  - **.ipk, gated on a hash:** ST.
- **Change path:** round 2 and round 3 said "NT commands while disabled, logged as inputs" in general terms. Round 4 names the concrete mechanism already in the code (`/Tuning/`, `SaveParams`, `ParamsInputs`, `liveChangesAllowed`).
- **The crash case is sharper:** tuning is unavailable when the robot program is down. Reviewers split on whether that matters (see Q1).
- **Built-in UI:** ZO repeats round 3's "link to the built-in UI, build only tuning and log download". That remains a dissent from the team's decision that the app is the single starting place and replaces tools where possible.

---

### Q1. Edit tuning through params? Which path?

| Option | Backed by | Strongest argument | Strongest counter |
|---|---|---|---|
| `/Tuning/` NT transport plus `SaveParams`. The robot program is the only writer. The service or browser is only an NT client. | All six | The robot already handles everything a second path would rebuild: bounds, applying on the main thread, the disabled/no-FMS gate, logged inputs (`Params/Values`, `Params/Sources`, so replay works), and hard-stop stamps (`ParamStore.stamped`). `ParamTransport` is already the place new clients plug in (ZO). | It needs the robot program running. If a bad value crashes the program at boot, nobody can fix it from the app (DC, CX, ST, CD, IR). ZO considers this acceptable: with no robot program there is nothing to tune. |
| The service writes robot-local JSON directly | No one. CD, ZO and ST call it a blocker. | Works after a crash. | It makes two writers of one format. The service would have to copy the schema, bounds, stamp rules and `.moved`/`.refused` handling (ST: the old app's stale-copy failure). The robot sees the change only at next boot, with no logged cause, which breaks replay and the audit trail (DC, CD, ZO, ST). |
| A new command path | No one | | Reinvents `ParamTransport` (ZO). |

**Crash-case options**, offered as a narrow fallback alongside the NT path:
- **Quarantine (DC, ST):** the service may only rename a robot-local file to `.disabled`, so the git values come back. A person must confirm it, and the robot logs it at the next boot. There is still only one writer of values.
- **Read-only view (CX, IR):** a view-only display of the robot-local files, with diff and download. CX adds a reviewed "revert to git" action.
- **Nothing (ZO):** the crash case needs logs and files, not params.
- **Open question (CX):** who the tuning user is in the crash state, a student or an AI session, and what the app must do then, is not stated in the concept.

**Hard-stop confirmation:** all six agree it is enforced on the robot, not only in the browser. The app shows old value, new value and failure mode and requires a typed confirm. The robot checks a confirm token or a second logged "confirm" NT write. CD adds that the confirmation (person, old, new, failure-mode text) is logged as an input and written into the saved JSON, so the pull-to-git PR carries it.

**Gaps in the existing params code, needed for the app** (mostly CD, echoed by others):
- **Save result:** `SaveParams` has no acknowledgement. Add a logged result such as `Params/Save/{requested, result, files, error, time}` (CD, DC).
- **Who changed it:** each request carries the client and a reason, logged as an input, for example `Params/Requests/{client, key, requested, accepted, why}`. Without it there is no way to tell which client made a change (CD).
- **Boot-only values:** swerve offsets and other boot-only values need a "pending reboot" state published by the robot, so the app doesn't present them as active. ZO suggests the app offer to restart the robot program; no hot reload (CD, DC, IR, ST, CX, ZO).
- **Refusals:** each refused request (wrong type, out of bounds) is echoed back, so a slider doesn't silently snap back (CD).
- **Why edits are blocked:** log why edits are refused (tuning off, enabled, FMS attached, program not running) as an input. CD notes `liveChangesAllowed` reads RobotState directly today. CX wants the app to show the reason instead of simply greying out controls.
- **Per-value display (CX):** effective value, the layer it came from, pending, needs reboot, hard stop.

**Facts to check:**
- Whether `SaveParams` reports its outcome anywhere a client can read.
- Whether `liveChangesAllowed` inputs are logged in a way replay can use.
- Whether a bad robot-local value can actually crash boot, or whether ParamStore refuses it and falls back.

### Q2. Getting saved values back into git

| Option | Backed by | Strongest argument | Strongest counter |
|---|---|---|---|
| Gradle `pullTuning` (fetch over the deploy SSH connection into `params/robots/<robot>/`) plus a deploy check that stops while unpulled values exist, with an override flag | ZO, DC, ST, CX | One mechanism, a normal git diff, and the check takes under 1 s (DC). | A hard block at an event costs matches. The override becomes a habit, and values get lost silently (CX, ST, DC). |
| Same, but the check warns and blocks only for hard-stop groups | IR; DC on event branches (never block an event deploy) | Avoids a new event failure mode. | Values that aren't hard stops can sit unpulled indefinitely. |
| The pit service also offers a diff or patch download, and a "copy prompt for AI session" option | DC (as a fallback), CX, CD | Works for laptops without the repo or the right branch, and keeps the app as the single starting place (CX). | It's the same diff in another wrapper. An AI session can run `pullTuning` itself (ZO, YAGNI). Two paths to keep in sync (CD). |
| The robot publishes one export of the values that differ from git, which every consumer reads | ST, CD (the service exposes the local-versus-deployed diff even without the robot program) | One source for Gradle, the app and AI. | A new interface to build. |

**Conflicts.** The reviewers differ on how much merging to do:
- **Three-way merge (DC blocker; IR and ST major):** the save records its base, either per-key git values or the GitSHA of the build that saved. The pull does a three-way compare:
  - local equals base: the value is stale, so drop it;
  - git equals base: take the local value;
  - all three differ: conflict, so stop.
  IR suggests storing the base as a reserved key, the way the stamp is stored, and treating old files with no base as conflicts. ST wants one merge library in `tools/`, with tests, which every consumer calls, so that three implementations don't drift (ST blocker).
- **Print and let the person choose (ZO):** never auto-merge.

**Stale overrides (DC, major):** after a pull and a deploy, the pulled local values must be cleared or renamed once git matches them. Otherwise later git edits never take effect.

**Hard stops:** never merged automatically (ZO). `pullTuning` prints old value, new value, failure mode and stamp for the PR (DC, ZO, IR).

**Event branches:** `eventDeploy` runs `git add -A` (IR, ZO). The options raised:
- Run `pullTuning` before the auto-commit, so values land on the event branch and reach main through the normal PR (DC).
- Pulled values get their own commit, and hard-stop groups get a separate commit flagged for review (CX, ST).
- Exclude hard-stop groups or stop the deploy (ZO). Refuse unless a confirm flag is given (IR).
- Commit only params JSON, never Java or spectrumLib (ST).

**Facts to check:**
- How `eventDeploy` stages and commits today.
- Whether ParamStore's legacy (pre-per-robot) local path is still in use. `pullTuning` must read both paths (IR).
- Deploy-check time over SSH.

### Q3. Does params change the runtime and packaging answers?

**Runtime:** all six agree there is no change towards Java. The params design makes the service thinner. System Python (`http.server` or Flask, already on the image) uses about 20-40 MB, against 100 MB or more for a second JVM.
- **Strongest counter:** a second language for AI maintainers, and no CI setup for a Python service yet (DC). DC suggests following the `tools/` unittest pattern and vendoring a pinned NT4 client. Java would share the DataLog reader or ParamStore parsing (ST, CD).
- **ST requirement:** the service and the robot share only a versioned NT/HTTP contract, never code.
- **ZO alternative:** the browser tunes over NT4 directly (WebSocket on port 5812), so the service never touches params at all. DC also treats the service as only an NT client, and ST as an NT relay and viewer.

**Packaging:** there is agreement on installing only when the service's hash changes. The vehicle splits:

| Option | Backed by | Argument | Counter |
|---|---|---|---|
| Systemd unit installed by Gradle | DC, ZO, IR | Fits the 10-15 s budget, and avoids the robot program restart that a package install causes. | No tile on the built-in page, which loses the shortcut goal (CX). |
| .ipk | ST | The tile and the package manager know about the app. | Restarts the robot program when installed. Whether the tile links to the app is still unconfirmed. |

**IR rollout requirements:**
- Install into a versioned directory with a symlink switch.
- Deploying an older commit reinstalls that commit's service.
- A failed service install warns but never fails the robot deploy.
- Never install while the robot is enabled.

CX wants the deploy output to say "app unchanged, skipped" or "app updated, robot program restarted".

**Facts to check:** whether the package tile links to the app (still open from round 3); a measured CPU baseline (DC, CD, ZO, using `FullCycleMS` with and without the service).

### Q4. Replacing TUNING_MODE

All six agree: replace the compile-time constant with a runtime gate enforced on the robot. Tuning is accepted only while disabled with no FMS, and only after an explicit arm, logged as an input. FMS connecting turns it off. The CI test changes to check that the default is off, and IR says this must happen in the same PR, or main loses its only guard.

How the gate is armed differs:
- **NT request or command from the app while disabled and with no FMS,** logged, which turns off on enable or FMS (DC, CX). ST adds expiry after N minutes. DC adds that a driver station must be connected and that nothing tunable is published until armed. CD wants the arm to time out.
- **A robot-local switch read at boot** (a file in the robot-local folder, or a param that can't be set over NT), off by default (ZO).
- **An explicit unlock input each boot** (IR).

**Strongest counter (all six):** a compile-time false can't be left on at a match or switched on over the network. A runtime gate moves that safety into code that could have bugs. NT and HTTP have no authentication, so anyone on the pit network could try to arm it. DC wants the design to say this explicitly. The proposed mitigations are the disabled/no-FMS gate, the hard-stop confirm, and a visible "tuning on" banner.

**ST requirement:** one `TuningMode` authority in spectrumLib that the transport checks, with a test for every condition. The app only displays the gate's state and never decides it.

**Facts to check:** every caller of `TuningMode` and `Constants.TUNING_MODE`; which test enforces it today.

---

### Other issues raised

- **Renamed keys orphan values (IR blocker):** robot-local values for renamed or removed keys go unread, and the robot silently runs the git default (for swerve, a wrong offset). On load, log unknown keys as a warning input and show them in the app. `pullTuning` reports them and never drops them. Define a key-rename procedure (an alias or migration entry) before the app writes anything.
- **Version skew (IR blocker, repeated from round 3):** the robot publishes its git SHA, the params list with bounds, and a hash of the log-key guide. The service reads its key list from that. On a mismatch the app goes read-only.
- **No copies of robot facts in the app (ST blocker, ZO):** group names, the hard-stop list and bounds come from `Params/` metadata. A versioned contract document plus contract tests in `tools/app-test`, so a renamed key fails CI.
- **Visibility of unpulled values (CD):** the robot logs `Params/UnpulledCount` at boot. The app and the deploy check show the same list.
- **Paths defined once (IR):** robot-local and legacy paths are published by the robot, and migration happens on first save only.
- **Built-in UI and scope (ZO):** the front page links to the built-in tools for each pit job, and the app builds only tuning and log download. This is the round 3 "link out" view, recorded as a dissent from the team's decision.
- **Device discovery (CX):** needs a stated method (mDNS, ARP, a list in the log) so it doesn't become a hand-kept list.
- **Port (IR):** move off 5800, keep a redirect from WPILib's WebServer during the transition, and delete the old pages in one PR.
- **Service health (CD):** `/health` reports the deploy's git SHA, and the service keeps a rotating log of its requests.
- **CPU limits (DC):** systemd `CPUQuota=10%`, `Nice=10`, `MemoryMax`, measured before relying on them.
- **AI parity (ZO):** if every write is a documented NT topic, AI parity comes for free and no separate API is needed.
- **Glossary (CX):** define "hardware stamp", "eventDeploy" and "pull back to git". CX found no definition of eventDeploy.

### Round 5: team proposals for install and pulling tuned values

Six new reviews of the team's proposals. Options are not chosen; the team decides.

Reviewers: Design Critic (DC), Codebase Details (CD), Zoom Out (ZO), Comprehension/UX (CX), Implementation Readiness (IR), Structural (ST). All six completed. The brief was concept5.md.
- **Proposal A:** install the service once as an .ipk, then each Gradle deploy updates its files and restarts the service only when they changed.
- **Proposal B:** on every deploy, copy to the laptop the robot's saved values that differ from git, print a summary and carry on. Ask a person when an interactive console exists; otherwise the choice is made later.

This document lays out the options and does not choose answers. The team decides.

### How the reviewers judged the team's proposals

| Reviewer | Proposal A | Proposal B |
|---|---|---|
| DC | Adopt with changes: the .ipk owns only a launcher stub; Gradle owns the app files | Adopt with changes: stop the deploy only for unresolved hard-stop values |
| CD | Adopt, provided the service reports exactly what it runs (`/version`) | Adopt as proposed: never stop the deploy, but log and show what is pending |
| ZO | Reject the split: Gradle owns everything; an optional stub .ipk for the tile | Adopt with changes: warn, stop only for hard stops, drop the interactive prompt, no queue |
| CX | Adopt with changes: every deploy checks the package and repairs it | Adopt with changes: warn and carry on; stop only for a hard-stop difference not yet acknowledged |
| IR | Blocker as written. Adopt with changes: stub launcher .ipk installed by a separate task, never inside `deploy` | Adopt: never stop, even for hard stops; print hard-stop diffs and never merge them automatically |
| ST | Reject the split: one owner, either an .ipk reinstalled when its hash changes or a Gradle unit with no .ipk | Adopt with changes: warn; stop only for unconfirmed hard-stop values; the prompt is optional and the same as the later command |

**Proposal A in short:** five of six (all but CD) say the split ownership, as written, breaks. The fix most of them name is a stable launcher stub, an idea DC, ZO and IR reached independently. ST rejects any split.

**Proposal B in short:** all six accept "copy, summarise, carry on" as the base. They split 4 to 2 on stopping for hard-stop values: DC, ZO, CX and ST want a stop, in different forms; CD and IR do not.

---

### Q1. Proposal A: install and update the service

**The problem (DC, ZO, CX, IR, ST).** If opkg owns files that Gradle later overwrites, opkg's recorded version and file list no longer match what is on disk.
- An OS update or a reinstall from the tile quietly restores old code.
- An uninstall from the tile runs `prerm`, which stops and disables the unit and deletes files that Gradle expects to be there.
- IR adds that the example `postinst` sleeps about 3 s plus whatever opkg itself takes.
- ZO calls this the old app's stale-build failure again.
- CD alone sees it as acceptable, provided `/version` reports the opkg version, the file hash and a mismatch flag.

**Options:**

| Option | Backed by | Strongest argument | Strongest counter |
|---|---|---|---|
| **Launcher stub.** The .ipk installs only a unit file, an icon and a stub that runs `/home/systemcore/fm-app/current/`. Gradle owns `fm-app/`, which opkg never lists. | DC, IR; ZO as the way to keep a tile | The stub's content never changes, so its recorded version can't go stale. A reinstall can't roll the app back, and an uninstall removes only the tile. | Still two installers, plus a one-off install step per robot. The tile's link to the app is unconfirmed (DC, IR). |
| **Gradle only.** A systemd unit installed by the deploy's `CommandArtifact`; no .ipk. | ZO (simplest), ST (one of its two options); DC and IR as the counter | One owner, nothing to go stale. | No tile, which loses the main entry point (ST). No OS-native lifecycle. Depends on the deploy user being able to install a unit; if it can't, a stub .ipk becomes mandatory (ZO). |
| **.ipk only.** The deploy builds the .ipk and runs `opkg install` when the content hash changes. | ST (the other option) | One owner, and the tile is kept. | Each change costs a robot program restart. `opkg` permission is unknown. ST counters that the deploy already restarts the program. |
| **Proposal A plus check-and-repair on every deploy.** If the package is missing or its version differs, the deploy runs the install; if `opkg` is not permitted, it prints one line and does not fail. | CX; CD broadly accepts A as written | No one-off install to forget, and an uninstall from the tile gets reversed. | Every deploy depends on `opkg` permission that hasn't been checked, and an install stops the robot program. |

**Where reviewers disagree:** whether a normal deploy may run `opkg install`.
- **Never (IR, ZO, DC):** IR wants a separate task, for example `installAppPackage`, gated on a `SHIM_VERSION`. The deploy only detects a missing unit (`systemctl cat`) and prints how to fix it.
- **Only when the hash changes (ST).**
- **As a repair step when the package is missing or differs (CX).**

**Detecting a change, and agreed details** (DC, CD, ZO, IR, ST, CX):
- Hash the app tree at build time and compare it with a hash or manifest file on the robot in one SSH call. Upload and restart only on a mismatch.
- Use hashes, not mtimes, because Windows checkouts and Gradle rebuilds change mtimes (DC, ST).
- Use SHA-256 (DC, IR).

**Version check** (all six):
- The service reports its manifest hash and git SHA. The robot code publishes its build SHA to NT. On a mismatch the app shows a banner ("app from build X, robot code from build Y").
- The mismatch is a warning, not a lockout, because the app must still work after a crash or a partial deploy (DC, ST).
- IR: an old service with new robot code degrades to read-only. The service re-reads `deploy/app-manifest.json` on each request. Keep `previous/` next to `current/` with a symlink so a bad update can be rolled back without a redeploy.
- CD: `/version` also reports the dirty flag and the opkg version.

**User-level restart (DC):** a `systemctl --user` unit or a narrow sudoers rule, so the deploy can restart the service.

**Facts to check** (all six say a bench check gates the design):
- Whether the `systemcore` user can run `opkg install` (`sudo -n opkg`, or the API on port 4803).
- Whether it can install or restart a system or user systemd unit.
- Whether the tile links to `X-Port` (`X-Has-UI`).
- What an OS update does to `/home/systemcore/fm-app` and to units.

### Q2. Proposal B: pull back to git on deploy

**Stopping the deploy:**

| Option | Backed by | Strongest argument | Strongest counter |
|---|---|---|---|
| Stop for every difference | No one (DC and ST give it as the counter) | The only option that forces a decision. Under VS Code tasks it fails clearly, where a warning goes unread (DC, CX, ST). | Blocks every VS Code and AI deploy, because those can't prompt (CD, IR). |
| Warn; stop only for hard stops | DC (unresolved hard stops, with flags `-PacceptRobotHardStops` / `-PdiscardRobotHardStops`), ZO (unless `-PacceptRobotParams` or `pullTuning`), CX (only a difference not yet acknowledged; acknowledging is a flag or a website click), ST (only a hard-stop local value with no matching hardware stamp) | Without it, stale local hard-stop values can override new git values at the next boot (DC). | Still blocks AI and VS Code deploys in that case (CD). |
| Never stop. Hard-stop diffs are printed as old → new with the failure mode | CD, IR | Nothing is lost: robot-local values survive deploys and keep winning on the robot. | A warning scrolls past. CD's mitigation is a pending count on the app's front page and in the log. |

**Interactive prompt:**
- **Drop it (ZO):** it doesn't work under VS Code tasks.
- **Keep it optional, but as the same code path as the later command, e.g. `pullTuning --choose` (ST).**
- **Never prompt without a console (CX).**
- DC, CD and IR don't address it directly.

**Where the copy is saved.** All six agree it must be in a gitignored path or outside the repo, because `eventDeploy` runs `git add -A`. ST and IR want a test that checks the path is ignored. Reviewers disagree on the folder:
- **Under `build/` (CD, ZO, CX, ST).**
- **Not `build/`, because `clean` wipes it (DC, IR).** DC suggests `.robot-params/<robot>/`; IR suggests `tuning-pulls/<robot>/<timestamp>/` plus a `latest.json`.

They also disagree on whether copies build up:
- **One timestamped copy per deploy, kept (DC, CD, CX, IR, ST).**
- **One copy, overwritten each time, with no queue or state (ZO).**

**One machine-readable record (DC, CD, ST, IR).** DC suggests `pending.json`; CD lists the fields: path, git value, robot value, base, group, hard-stop flag, save time, robot, whether the stamp matches. Gradle, the website and AI all read it, and nobody parses console text. CD also wants each deploy's summary kept, so "what did the robot have before this deploy?" can be answered later.

**If nobody chooses** (all six agree): the robot-local values keep winning on the robot, so nothing is lost. Each deploy repeats the summary. DC and ST add each entry's age ("3 deploys, 6 days unresolved"). Never clear values nobody chose. CD wants the robot to log, as inputs, which layer won for each param and a count of overrides, with the count shown on the app's front page. CX wants one fixed line until someone settles it, for example "3 robot values not in git (1 hard-stop). Run `./gradlew reviewParams`", with the same line on the website.

**Hard stops** (all six agree): never merged automatically. Write a fragment ready for the PR with old value, new value and failure mode; ST adds the hardware stamp. DC wants the failure-mode text to come from the param's declaration, so each hard-stop param needs that string. CD and ZO would leave a blank placeholder for it instead.

**Clearing robot-local values once git matches** (all six agree it must happen, exact matches only; IR and ST say never by tolerance or age). Reviewers disagree on who clears:
- **The robot, at boot (DC).**
- **The next deploy, while the robot program is stopped (CD, CX, IR, ST).** CD renames cleared entries to `.merged` and logs it, following ParamStore's existing rename convention.
- **`SaveParams` prunes equal entries on the next save (ZO).**

**event\* branches** (blocker for DC, CD, IR, ST):
- DC notes that `eventDeploy`'s `git add -A` runs before the deploy (`createVersionFile.dependsOn(eventDeploy)`, build.gradle about lines 203-233). So any tracked file written during a deploy ends up, unreviewed, in the next auto-commit.
- **Agreed rule:** no deploy writes tracked files. Only an explicit merge command writes `src/main/deploy/params/` (DC, IR).
- ZO: don't auto-pull on event branches.
- IR: run the pull after `eventDeploy`'s commit, or write outside the tree.

**ST requirements:** every deploy-time step has a timeout, and the deploy carries on if the robot can't be reached for the pull. DC adds that a hashing or SSH failure in the app step should warn, never fail the robot deploy.

**Facts to check:** the exact task order of `eventDeploy`, `createVersionFile` and the deploy; whether `.gitignore` covers the chosen folder; how long one SSH pull takes.

### Q3. Conflicts when git changed after a save

| Option | Backed by | Strongest argument | Strongest counter |
|---|---|---|---|
| Three-way merge against a recorded base. Take the robot value if only the robot changed; drop the robot value if only git changed; show both if both changed. | DC, CD, CX, IR, ST | Without a base, "git changed" and "robot changed" look the same, and every git edit looks like a conflict (IR, ST). | A file-format field every tool must keep stable (ST, CX). |
| Show both values and let a person choose; record only the git SHA | ZO | Params are scalars, so there's nothing to merge, only "which number". | Without a base, an unchanged robot value can wrongly look like a deliberate override of a newer git change (ZO's own counter). |

**Where reviewers disagree on the base:**
- **The git value at save time, per key (DC, CX, ST).**
- **The deployed git SHA from BuildConstants, plus the branch (IR).**
- **Both, plus the dirty flag (CD).** CD's reason: a base taken from a dirty working tree is misleading.

**Other conflict details:**
- An entry with no base (a legacy file) counts as a conflict (DC).
- Never take a side automatically for a hard-stop group (IR).
- IR accepts "show both" if the base is still recorded, for later use.

**Reviewers disagree on how Proposal B affects conflicts:**
- **Rarer or cheaper:** values come back, and bases are refreshed, on every deploy (DC, CD, ZO, IR). IR adds that B doesn't remove conflicts, because a laptop on another branch deploys too.
- **More common:** copies wait longer before anyone chooses (CX).
- **Matters more:** pulls happen often, so without a base every git edit shows up as a conflict (ST).

### Q4. Show the robot-versus-git difference on the website?

**All six say yes, read-only.** It offers "copy patch" and "copy AI prompt" buttons; ZO adds a copyable `./gradlew pullTuning --only ...` command. The website never writes to git, because the robot has no repo or credentials.

**Requirements raised:**
- One shared JSON format and one decision file, so Gradle and the website can't disagree (DC, CD, CX, IR). Version the format (IR).
- ST: one diff implementation shared by Gradle and the service, or the two will drift. The service has no git checkout, so label its git side "as deployed".
- DC: hard-stop rows show the failure mode and need a click to confirm.
- CX: the AI prompt names the values, the base and the hard-stop rule.

**Counter:** two places to settle one decision (CD, CX). ZO notes students without laptop git skills are blocked without a downloadable patch, which is why most reviewers include the patch.

### Q5. Params while the robot program has crashed

**All six recommend a read-only diff plus "disable local file":** rename the file to `.disabled`; IR suggests `<group>.json.disabled-<time>`.
- **Argument:** a bad local value (a swerve offset, a soft limit) is the likely cause of a crash at boot. The rename is reversible and needs no SSH (DC, ZO, CX, IR).
- **Counter (all six name it):** the rename writes robot config outside the robot program, which skips AdvantageKit's input logging. ZO notes that a read-only view is purer, and SSH already exists for emergencies.

**Mitigations:**
- The service logs the rename (DC, CD, IR, ST).
- The robot program logs at boot which local files are present and which have been disabled (CD, CX, IR, ST).
- A page-click confirmation that names the file and says the change takes effect at the next start (CX, IR, ST).
- Scope limited to robot-local params files (CX).
- No editing of values while the program is down (DC, ST).

---

### Other issues raised

- **Which layer won (CD blocker):** log, as an input, which layer supplied each param, plus a count of robot-local overrides, so a stale override can be told apart from a git value.
- **Service observability (CD):** journald logging, plus `/health` reporting uptime, restarts, CPU and RSS.
- **CPU (DC, CD, ZO):** still no baseline. Measure loop timing with the service idle and during a log download. Set `Nice=10` and `CPUQuota`.
- **Python drift (DC):** system Python and its pip packages change with OS updates. Use only the standard library, or a pinned venv in the app tree.
- **TuningMode transitions (CD):** log on and off changes, with the reason (website request, FMS attach, boot), as inputs.
- **Log download integrity (CD):** report file size and hash after the download, so a truncated transfer is caught.
- **No param logic in the service (ZO, ST):** reuse the ParamStore JSON layout and `/Tuning/`. The service only lists and reads files. No copies of robot facts.
- **One versioned JSON schema (ST, IR)** for robot-local params, the pulled copy and the diff, with tests in `tools/`.
- **Terms a newcomer will hit (CX):** define hard-stop group and hardware stamp; say which component records the version (the opkg status database); name the "choose later" command and how long a copy stays valid; explain how the website tuning button relates to `TUNING_MODE`, which a test forces off on main.

### Implementation details still undecided

- The stub's path, its `SHIM_VERSION` and the format of the API schema version (IR).
- The manifest hash scope and file name (`.hash`, `manifest.json`, `version.json` and `app-manifest.json` are all proposed).
- The name of the separate install task and whether the deploy ever calls it (IR: `installAppPackage`).
- The folder and name of the pulled copy, how long copies are kept, and whether copies accumulate or one is overwritten.
- The name of the decision file (`pending.json`) and its fields.
- The base field name and content in robot-local files (per-key value, git SHA, branch, dirty flag).
- Who clears matching robot-local values (the robot at boot, the deploy, or `SaveParams`), and the name for a cleared file (`.merged` or deletion).
- The names of the hard-stop override flags (`-PacceptRobotHardStops`, `-PdiscardRobotHardStops`, `-PacceptRobotParams`) and how an acknowledgement is stored.
- The command names (`pullTuning`, `pullTuning --choose`, `pullTuning --only`, `reviewParams`).
- Where the failure-mode text comes from: the param declaration or a blank placeholder.
- Where the new tasks sit relative to `createVersionFile` and `eventDeploy`.
- The rename suffix for a disabled file and the name of the log the robot reads at boot.

### Round 6: robot-specific pages and guided procedures

Six new reviews with the robot-specific framing. Options are not chosen; the team decides.

Reviewers: Design Critic (DC), Codebase Details (CD), Zoom Out (ZO), Comprehension/UX (CX), Implementation Readiness (IR), Structural (ST). All six completed. The brief was concept6.md. Its givens:
- The app lives in the same repo and build as the robot code.
- Pages are hand-made for one robot.
- Guided procedures run motion steps in utility mode, started by the driver with buttons. The website never commands motion.
- Tuning goes through the params system.

This document lays out the options and does not choose answers. The team decides.

### Agreement and disagreement at a glance

**All six agree:**
- Nothing today stops a page from reading a key the code no longer has. CI should check each page's declared keys against generated lists.
- A key check can't catch a changed meaning (units, sign, inversion). A scripted sim per procedure is the proposed answer.
- The robot publishes the procedure state (`Procedure/Active`, step, state, results). The page reads it and never starts motion.
- Each test has a timeout and an abort, and the driver gets a dashboard and rumble cue.
- Values read only at boot show "saved, reboot to apply" until the logged source confirms them.

**Where they disagree:**
- **Where the shared part lives (Q2):** this repo until a second robot exists; spectrumLib; or a separate repo.
- **Where the capture gate runs (Q3):** on the robot, as a logged input; or in a shared page-side framework that reads logged timestamps.
- **Whether the page may arm or select a test while disabled (Q4),** or the driver chooses with buttons only.

---

### Q1. Keeping robot-specific pages correct

**What exists today** (reported by DC, CD, ZO, CX, IR, ST):
- CI runs `make_log_key_guide.py --check` (log keys only), the Python tool tests, `node --test tools/app-test/*.test.mjs` and scripted sims checked by `ci_assert.py`.
- The `app-test` suite has three tests (analysis, camera-cal, links). None compares a page's keys with `log-keys.json`.
- Nothing generates a list of params, `/Tuning/` keys or the NT topics pages read.
- CD's example: `swerve-align/main.js` hard-codes `/AdvantageKit/RealOutputs/Swerve/Align/`, so renaming `SwerveAlignment.KEY_PREFIX` passes CI today. The AGENTS.md "update the page in the same PR" rule is honour-system only.
- `SimScript` has `drive`, `shoot` and `auto:` scripts, but no utility-mode script (CX).

**Key-existence check: all six back the same core design.**
- Each page declares the keys it reads in one place: a `KEYS` export, a `reads` list or `pages/<page>/keys.js`.
- A node test checks those keys against `log-keys.json` plus a new generated `params-keys.json` built from the `Params.group(...)` declarations.
- **NT topics a page reads:** they need a source too. ZO and DC put them in the generated output. ST wants a `procedures.json` from a robot-side `Procedure` registry. IR says they should come from a generator or LogKeyGuide.
- **Strongest argument:** this turns the honour-system rule into a CI failure. It reuses the existing generator pattern and AGENTS.md's "NT prefixes at the top of each page" rule (ZO).
- **Strongest counter (DC, CD, CX, IR, ST):** the generator is a regex over Java that silently skips calls it can't parse (its own "ponytail" note). Keys built at runtime (`Mechanisms/<name>/...`) can escape the check, so it may give false confidence.

**Mitigations raised for the generator gaps:**
- Match `<name>` templates by pattern, and treat a key missing from every list as missing, so gaps fail closed (DC, IR).
- Make the generator fail on calls it can't parse, at least in mechanisms that have pages (CD, ST).
- Pages never type a key outside their manifest (ST).
- Roll the test out warn-only for one PR, then make it fail (IR).
- CX wants an actionable error, for example "page `hood-limits` reads `/Tuning/hood/max`, which no code declares; did you rename it to ...?". The check runs in `./gradlew build`.
- DC offers a cheaper alternative and calls it weaker: the robot publishes `/App/Schema` at runtime and the page refuses missing keys. That catches the problem only on the robot, not in the PR.

**Behaviour drift (a changed unit, sign, inversion or limit):**

| Option | Backed by | Argument | Counter |
|---|---|---|---|
| One scripted sim per procedure (`FM_SIM_SCRIPT=procedure:<name>`). Its wpilog is replayed through the page's capture logic in a node test. | DC, CD, CX, IR | Tests the page's own logic against the robot's behaviour. | Sim models of hand-moved mechanisms are weak and "prove wiring, not physics" (IR). Slower CI. Upkeep for pages used a few times a season (CX). |
| One sim script per procedure that presses the utility buttons, with `ci_assert.py` checking the logged result keys | ZO, ST | Uses tooling the team already runs. | Doesn't exercise the page's JavaScript. Sim models may not exist (ST). |
| Fixture logs per page, taken from the real robot and regenerated on purpose | CD (as its counter) | Real data, cheaper than sim models. | Has to be refreshed by hand. |

**Units:** store the unit with each declared key and compare it with LogKeyGuide's metadata (CD, ZO, CX). ZO notes this covers unit changes. Sign and direction changes still need the sim.

**CI cost (DC):** run the procedure sims only when `subsystems/**` or `app/pages/**` change.

**Facts to check:** how many logged keys the generator misses today; whether sim models exist for each mechanism that will have a procedure; CI time per sim script.

### Q2. Shared versus per-robot, and where the shared part lives

**What is shared, what is per robot.** All six broadly agree on the split.
- **Shared:** service, log download, version banner, tuning-mode button, params diff, widgets, and a procedure framework (step engine, capture guard, hard-stop confirm).
- **Per robot:** pages, CAD assets, step text, sim scripts and manifests.

**Where the shared part lives** (reviewers disagree):

| Option | Backed by | Strongest argument | Strongest counter |
|---|---|---|---|
| Keep it all in this repo, in a separate folder (`app/core`, `app/lib`); copy it forward to the next robot; extract it only once a second robot exists | ZO, CX, IR | One consumer gives no evidence about the right abstraction (YAGNI). No toolchain or vendoring step. | Copies drift, and fixes for one robot never reach the other (ZO, CX, IR state this themselves). |
| JS shell in its own repo, vendored into `app/vendor/` as one file per release; Java half in spectrumLib | DC, ST | A versioned, single source. Fits the one-file vendor rule. | A second repo adds version skew and a vendoring chore. AGENTS.md bans spectrumLib edits on `event*` branches, so a framework bug can't be fixed at an event (DC). Heavy for a student team (ST). |
| JS framework copied into spectrumLib's app side, with a version string | CD | Lives with the code already shared between robots. | spectrumLib is itself copied per repo, so it drifts the same way. |

**The Java half (the procedure state publisher):** DC, ZO, CD and ST place it in spectrumLib. ZO puts it next to `checks/` (the existing `Check`/`PreMatchChecks` framework, which already runs on the main thread, reads only logged inputs, and reports without acting). ST puts it next to params. CX and IR would keep the shared part in this repo now and move it to spectrumLib once a second robot uses it.

**Requirements raised:**
- The shared layer names no mechanism, key literal or CAN fact (CX, ST).
- A `lib/VERSION` plus a contract document of the NT and JSON shapes the shared layer expects. The contract schema is versioned, so an old shell refuses a newer contract (IR, ST).
- The banner shows the framework version, the robot git SHA and a hash of `log-keys.json` (CD).
- Land the shared framework and the key-manifest CI on `main` before the season, not during an event (IR).

### Q3. Procedure safety and data

**Where the capture gate runs.** Reviewers disagree:

| Option | Backed by | Strongest argument | Strongest counter |
|---|---|---|---|
| **On the robot.** A capture request goes over NT and is handled on the main thread. The robot checks `connected`, sample freshness and steadiness, logs a verdict and reason as an input or output, and the page only displays it. | DC, ZO, ST | The TXMAN1 rule is about code acting on readings, and writing a param is an action, so replay must see the gate (ZO). One `CaptureGate`, no copies per page (ST). Can build on the `Check` pattern (ZO). | Every procedure change needs a Java change and a deploy, which slows pit iteration. Page-side checks are faster to change, and nothing is written until a person saves anyway (DC, ST). |
| **In a shared page-side procedure framework** (not each page). It refuses capture while any listed `connected` input is false or a timestamp is older than N cycles, and shows the reason. | CD, CX, IR | Faster to iterate. The plain-words reason belongs where the student looks. CX: staleness measured from logged input timestamps, not the browser clock. | Replay can't see a page-side gate. CD notes the NT4 client doesn't track how old each topic is today. |

**Agreed details under either option:**
- Each capture step lists the devices it depends on. It refuses when any is disconnected or stale and shows the reason in plain words, for example "Front-left encoder disconnected (CANivore). Capture disabled." (all six).
- `connected` must be true for the whole capture window, not only at the instant of capture. Debounce a flapping `connected` (DC).
- Samples must be steady: spread under a set tolerance over N samples (DC, ZO).
- Freshness limit: DC suggests 2 cycles, IR 3 cycles.
- A captured value carries its source key, sample count, spread and robot timestamp, so a real capture can be told apart from zeros (CD).

**Hard stops** (all six): show old value, new value and failure mode; a person clicks to confirm; apply through `/Tuning/` and `SaveParams` while disabled; log the confirmation as an input. DC says this click is distinct from the tuning-mode toggle. CD and IR note the source of the failure-mode text is still undecided, and IR says this blocks the swerve page.

**Values read only at boot (swerve offsets)** (all six): show "saved, reboot to apply" until a logged input confirms the change after the reboot. Reviewers name different inputs for that check: `Params/Sources/swerve/*` reporting the robot-local layer (CD, ZO, ST), or `Params/Values` compared with the saved file (IR). ST wants a `requiresReboot` flag in `params-keys.json`. IR: don't let a second capture build on a value not yet applied, and document disabling the local file as the rollback.

**Other risks:**
- An NT disconnect mid-step counts as a fault, not as "the last value" (DC).
- Two browsers running one procedure: the robot needs a single owner token (DC).
- Refuse capture when the `$hardware` stamp or the manifest SHA doesn't match (IR).
- A fixed staleness limit falsely rejects readings on a loaded bus, but per-device limits add tuning students must understand (CX, IR).

### Q4. Coordinating a page with a utility-mode test

**What exists today:**
- `utilityInit` only logs a "utility" event, and `utilityPeriodic` and `utilityExit` are empty.
- `Util.testMode` is a bare trigger, so there is no test identity to read (CD, DC, ZO, ST).
- `Gamepad` already maps buttons (ZO).

**Agreed by all six:** the robot publishes the procedure state, set on the main thread. The page reads it and never starts motion.
- Proposed keys: `Procedure/Active` (test id), `Step`, `State` (idle / armed / running / done / aborted / timeout), start and end times or a sequence counter, `Result/*`, and `AbortReason`.
- IR names them `UtilityTest/*` instead.
- Each test has a timeout and stops when the button is released or the robot is disabled.
- Leaving utility mode mid-test sets "aborted", and an FMS attaching clears the selection (DC, ZO).
- The driver sees the step on the dashboard and gets controller rumble at start and end. The page shows "Waiting for the driver: enable utility mode, press A".

**Accepting only fresh results:** the page accepts results only from a run it saw start, using `StartTs` (DC) or a `Seq` counter that advanced after the page armed (IR, ZO). This keeps a stale result off the page.

**Which button to press:** the expected button comes from the same binding table the robot uses, generated into a deploy JSON (CD) or shared with the controls guide (ZO). ST wants one robot-side registry as the only source for steps, bindings and results, so a page can't name a test that has no binding.

**Who selects the test** (reviewers disagree):

| Option | Backed by | Strongest argument | Strongest counter |
|---|---|---|---|
| The page arms or selects the procedure while disabled, as an NT command logged as an input. In utility mode the driver's button runs only the armed test. | DC, ST | Fewer driver mistakes. The page and the robot agree on what runs. ST: the page shows a fault for "utility mode but nothing armed". | Arming from the website is an indirect way of picking the motion, close to the "website never commands motion" decision (ST's own counter; CX). |
| Arming happens only on the page; nothing is sent to the robot. The robot's choice of test reads only button inputs. | IR | Keeps the robot's logic free of page state. | The driver can still press the wrong button. The page only notices afterwards, by the test id. |
| The driver chooses with buttons only; the page only subscribes | ZO, CD, CX | Purest reading of the decision, and the smallest protocol. | Depends on the driver pressing the right button. ZO would accept a logged "requested test" NT field that the robot reads only while disabled, only if it selects and never starts. CX calls a page request "clearer for the student". |

**Inputs or outputs:** whether the state is logged as outputs or inputs.
- **Outputs (CX, IR, CD):** the page only reads them. Robot logic must not read them back, and the test choice must come from button inputs (IR).
- **Logged inputs or outputs computed from logged inputs (DC, ST):** so replay is deterministic.

**Inferring the test from mechanism logs:** DC and ST reject having the page infer which test runs from currents or velocities. It's fragile and breaks replay. CD gives the cheaper generic option ("last utility command name plus start and end time") as its counter, but notes it can't carry results.

**Facts to check:** whether the driver dashboard can show the procedure step; the rumble API on the 2027 alpha; how `Gamepad` bindings could be enumerated for a generated binding table.

---

### Other issues raised

- **One shared "propose a value, then a person confirms" path (ZO):** the procedure pages and the AI API both need it. Build it once.
- **Tuning mode on main (IR):** the `TUNING_MODE` test that fails PRs to `main` must be changed (a boot default of false plus a runtime request) before any procedure page ships.
- **Rebuilding existing pages (IR):** keep the old `swerve-align`, `can`, `power` and `cameras` pages served until each new page passes its sim test, then delete the old one in the same PR.
- **NT staleness in the client (CD):** `lib/nt4.js` needs to track when each topic last updated.
- **NT load (DC):** about 10 keys per active procedure, which is small. CI time is the main cost.
- **Glossary on pages (CX):** define "utility mode", "hard-stop group", "`$hardware` stamp" and "capture" on first use.
- **Brief scope (ZO):** the "implementation details still undecided" list (file and flag names) is too detailed for this stage of the concept.

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
5. Tuning mode is turned on by a NetworkTables request from a button on the website. It has no timer: the robot network is secured at events and the robot is only powered while people work on it. It starts off at every boot, since the request is not saved. It turns off when an FMS attaches.
6. Install with a launcher-only package: an `.ipk` installed once holds only the systemd service file, the icon and a small launcher that never changes; the launcher runs the app from a folder only Gradle writes to. Each Gradle deploy updates that folder and restarts the service only when a hash of its files changed. The package manager and Gradle never own the same files. Still to check on a SystemCore: whether the deploy user can restart the service and run `opkg install`, and whether the tile links to the app.
7. Pulling tuned values back: every deploy copies the robot's saved values that differ from git into a folder git ignores, prints a summary and carries on. It only warns, including for hard-stop values (four reviewers wanted a stop for those; the team chose a warning). No terminal prompt. Only an explicit command writes the params files in git.
8. Conflicts (git changed after a value was saved): show both values and let a person choose; no automatic merge.
9. The website shows a read-only robot-versus-git diff with "copy patch" and "copy AI prompt", and never writes to git.
10. While the robot program has crashed: show the read-only diff, the errors and the program log, and allow disabling a bad saved file by renaming it (confirmed by a person, logged by the robot at next boot). Usually the fix is a redeploy; the app's job is to show what went wrong.
11. Guided tuning procedures, each with its own page, using live readings: swerve offsets (the page reads each wheel's angle, has the user spin each wheel and point it forward, then captures); soft limits (the user moves a mechanism to each end and the page records the positions); direction checks; others later. Pages can use graphics from the robot CAD. Most steps are hand-move with the robot disabled. Some steps ask for the robot to be enabled so the page can read what happens; those run in utility mode (the 2027 name for test mode), and the driver presses controller buttons to run each test. The website never commands motion itself.
12. The app is built for one specific robot, once that robot exists, in the same repo as its code. Pages are hand-made for that robot's mechanisms (for example a page for the hood's limits, with its own steps and CAD graphics), not generic pages that discover mechanisms. This is why the app does the work itself instead of linking to general tools. Open: how robot-specific pages stay in step with the code (the reviewers' copy-of-robot-facts concern), and what carries over to the next robot.
13. Hard-stop confirmation stays a click on a page (decision 4); no driver-controller confirm. The team tunes and runs without a controller routinely. Five final-round reviewers flagged that a page click cannot prove a person, not an AI session, confirmed; the team accepted that.
14. Stale saved hard-stop values: the robot reports at boot when a saved hard-stop value differs from git, and a banner stays on the app and the driver dashboard until it is resolved.
15. Fix `lib/nt4.js` to keep each topic's last-update time, so pages can show data age and refuse stale readings.
16. The service is written in Java, in its own source set, with no dependency on robot classes, the HAL or Phoenix.
17. Gradle generates a module of keys, params and button bindings from the Java during the build. The drift check runs in CI on push and can be run locally; a local build does not stop a push.
18. Utility-mode tests: the website starts its own guided session (which steps, which parts of the robot to show, which buttons to ask for); it never commands the robot. The driver presses controller buttons to make anything move. Common tests have driver-controller bindings; niche tests are guided from the website the same way. Consistent with decision 11.
19. The shared Java part (procedure state, capture check) can live in spectrumLib. On `event*` branches, spectrumLib may take fixes for real bugs that are causing problems, but not minor tweaks or new features. AGENTS.md's event-branch rule ("no spectrumLib edits") is stricter than this and needs rewording.

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
