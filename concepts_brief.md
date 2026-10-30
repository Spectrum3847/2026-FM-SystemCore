# Robot App

* **Author:** Allen Gregory
* **Date:** 2026-10-29
* **Status:** requirements brief, ready for an independent review. Built only from the team's own
  words and from facts checked against the repo or vendor sources. Earlier review rounds are not
  part of this document (the previous version is in git history at commit `1d4f1d9`).

## Overview

The Robot App is a web app for one specific robot, kept in the same repo as that robot's code. It is
for the team's 2027 robot on SystemCore and will be built once that robot exists. Students and
programmers use it to see the robot's controls, tune it, diagnose problems and get files off it.
It is meant to be the one place they start, and AI sessions directed by a person can use it too.

## Requirements, in the team's words

Each item quotes what the team said, then states what it requires. Where a quote answers a question,
the question is given in brackets.

### Where it runs and how it ships

* "Run on the PC and be able to access so a shortcut on the SystemCore website would be best."
  Opened from a browser on a laptop or phone; ideally linked from SystemCore's own web UI.
* "Deploys when the robot code deploys and updated in the same build."
  "I thought it was obvious that it was the same repo that's been there from the start it has to
  update based on the robot code changes."
  Same repo, same build, updated by every robot deploy.
* [Should it work when the robot program has crashed?] "Yes to crashed but not when not deployed
  (you can deploy a separate thing each time code deploys)."
* "10-15 extra seconds is fine, mostly don't want to take 1.5-2x as long as it currently does."
* "Still need to not use CPU." No CPU measurement of a normal robot exists.
* "This page setup is being made for a future 2027 robot so the current systemcore version doesn't
  really matter, it's what will work on the latest one."
* Install: "install with ipk to get the icon and then gradle deploys do the updating so only have to
  ipk install once on that image (unless deploy can do that)." Clarified: "Launcher only package is
  what I thought I meant", an `.ipk` holding only a launcher, so the package and Gradle never own
  the same files.

### Who uses it

* "Pit students and programmers mostly, mostly laptop but phone is very helpful. Being able to see
  live things during practice or tuning, etc."
* It is used in the pit and during practice and tuning, and for diagnostics.
* "AI will maintain it."

### What it does

* "Displays the controllers and buttons layouts, other information about the code and the system
  and how to use it." Clarified: "it shows the controller displays, and can explain how to tune
  certain things ... It should help explain how to use its tools, tool tips, guides, etc. but not
  like tutorials or quizzes, etc." It "shouldn't explain what an intake is": it explains its own
  tools, not the robot's mechanisms.
* "Tuning help for things like swerve module angle, gyro scalar tuning, other diagnostic and tuning."
* Guided procedures: "the webpage for setting swerve positions ... can read wheel values so it can
  say ... the user needs to spin each wheel and direct the user to point each one forward, etc. Same
  for tuning things like softlimits having the user move a system to its limits and recording the
  values, checking directions, etc. All can have their own pages, graphics from robot cad, etc."
* "Webpage says hand move for most things but it can also ask for the robot to be enabled for it to
  read certain logs, maybe it's integrated into test mode. The driver would press buttons to do
  things." Clarified: "It doesn't command the robot to do anything it starts itself asking for
  certain buttons, displaying certain parts of the robots, etc. Driver still hits the buttons to do
  the moving." And: "most common have duplicates on the driver controllers but niche things start
  from the website."
* "Robot network help, what devices are on the robot network, links to their configuration pages
  like camera configuration pages, etc." Devices: "Systemcore, limelights, Jetson, the radio, that
  should be most but could have networked sensors or other team laptops." Discovery is required:
  "someone plugs in an unconfigured device and need to know where it's webpage is to configure it." [After the review
  suggested a fixed device table with scans put off:] "It needs to scan devices."
* "Log downloading, viewing the system files and transferring things, etc." [What needs to move?]
  "Largely just logs off the robot, there isn't a convenient way to do it, you have pull up winscp
  because the other tools fail sometimes, or know how to do it in the command line."
* "Things you can do thru should be able to be controlled by AI agents usually." [Run alone?] "I
  don't expect the agent to run by itself we mean a ai session being directed by a pit student or
  coach to figure things out and make the robot better." [After the review asked API or documentation
  only:] "AI interface is API."
* While the robot program has crashed: "Likely try for diff and disable bad file, the likely outcome
  is we are redeploying code anyway if it's running while code crashed but displaying errors and
  things is nice."

### Scope against other tools

* "Other apps exist but those are all different places and that's annoying and hard to teach, you
  have to know every one of those places, the web site at a minimum should explain what tool to use
  for each thing but should replace the tools where possible."
* "It's a hyperspecific app for that specific robot. That's why just linking to the other tools
  doesn't make sense."
* The current plan and first version (`docs/robot-app-plan.md`, `src/main/deploy/app/`) are one
  candidate: "I want to make sure the current plan is the best plan possible, we don't care if we
  delete the whole work already done."

### Tuning values (params)

* "The param files and how that all links back to tuning values getting pulled back from the robot
  automatically, so this website can modify the param values directly for tuning." "We still need
  to build the bring back to git part but the idea is there, this app can use those files if the
  reviewers think it's correct."
* Turning tuning on: "Network tables request that can be hit from the website." [After the review
  found the compile-time `TUNING_MODE` gate blocks this on `main`:] "Website request has to be able to
  work." No timer: "Wifi
  network does have security at events, so its not security risk and robot should only be on when
  people are working on it." "Turn off when fms attaches."
* Pulling values back: "Can the deploy ask what to do with the changes in the terminal? Or can it
  save a copy of the old params if there are changes then ask which one to keep after? Seems like
  there are various ways to solve this correctly."
* Hard-stop confirmation: the team tunes without a driver controller ("we tune and run without a
  controller for things all the time"), so confirmation cannot require one.

### Rules

* Custom dashboard is a requirement: "There isn't any reason to really use elastic for most things
  when we can custom build a dashboard served by the robot." "Building the widgets is minutes of
  work ... we can cache the page on the laptop so it's only the data sending over the network that
  is going there anyway." "Camera video comes from our custom photonvision software we can make the
  website show those."

* Power, CAN and Cameras pages: "Replace power, can, cameras, ours will have way more detail for the
  robot, list what's on each can branch, list how power is being used since we have the different
  power sources, currents, etc."

* The app is a separate Python web app: "Being a separate python web app makes the most sense now, the
  systemcore has python, don't have to compile/build anything, just send the python code. Every
  system with wpilib installed can still build without any prerequisites except tests, etc."

* The app moves off port 5800: "we know the port has to change ... that's always going to change."

* `spectrumLib` on `event*` branches: "Fixes to spectrum lib have to be able to made at events ...
  it's just that you shouldn't make minor tweaks, or feature updates." AGENTS.md now says so.
* The name is the Robot App.

## Team proposal: params sync and reconciliation (v2)

Written by the team. It answers the "pulling values back" requirement above and replaces choices
2-4 below.

### Goals

Sync tuned values from `/home/systemcore/params/` back to git without colliding with
`eventDeploy` (`git add -A`), without blocking headless or AI deploys, without leaving stale
overrides on the robot after code changes, and without depending on the robot's clock to decide
which value wins.

### Clock

During deploy, over the live SSH session, Gradle sets SystemCore's clock if it is behind the
laptop: `if [ $(date +%s) -lt <laptopEpoch> ]; then sudo -n date -s @<laptopEpoch> || true; fi`.
Nothing is written to disk, so a later reboot without a deploy never restores a stale time. No
browser or NT clock sync. Time is for display only (`savedAtEpoch`, "saved 12 min ago"); it never
decides which value wins. A save with no valid clock records `savedAtEpoch: null`, never 1970.
To verify on 8515: whether SystemCore's clock starts at 1970 without a Driver Station, and whether
`LogStorage.java:214` is affected.

### Deploy steps (in order)

1. `fetchRobotParams`: reads `/home/systemcore/params/` over SSH into `.robot-inbox/<robot>/`
   (git-ignored, so `eventDeploy` never stages it). It never writes to the robot. Never blocks or
   prompts; if the robot is unreachable the deploy continues.
2. `generateDeployMeta`: writes `build/generated/deploy-meta/deploy-meta.json` (under `build/`, so
   the working tree stays clean), deployed to the robot's deploy directory as its own GradleRIO
   artifact. Contents:
   * `gitSha`, `gitDirty`, `dirtyParamFiles` (from `git status --porcelain`);
   * `ancestorChecks`: for each `savedAtGitSha` found in `.robot-inbox/`: `git cat-file -e
     <sha>^{commit}` fails -> `"unknown"`; else `git merge-base --is-ancestor <sha> HEAD` exit 0 ->
     `"ancestor"`, exit 1 -> `"diverged"`;
   * `fetchedFileHashes`: `{ "<group>.json": "<sha256>" }` for every file successfully copied into
     the inbox.
3. Deploy code and `deploy-meta.json`.
4. End-of-deploy banner in the terminal: clean overrides (`base -> value`), conflicts, hard-stop
   groups, and a loud line whenever a value just deployed will be overridden by a robot save at
   boot (rows 5 and 7 below), with the command to resolve it (`./gradlew applyRobotParams`).

### Saved file (`/home/systemcore/params/<robot>/<group>.json`)

```json
{
  "_meta": { "robot": "alpha", "hardwareStamp": "a1b2c3d4",
             "savedAtGitSha": "8f3d1a2", "savedOnDirtyDeploy": false },
  "values": { "Elevator/kP": { "value": 4.2, "baseDeployedValue": 3.5, "savedAtEpoch": 1761815700 } },
  "_superseded": {},
  "_orphaned": {}
}
```

`/Tuning/SaveParams`: a key whose live value equals the deployed value is removed; otherwise it
writes `value`, `baseDeployedValue` (kept from an earlier save in the same deploy), `savedAtEpoch`,
and `_meta` (`savedAtGitSha` and `savedOnDirtyDeploy` from `deploy-meta.json`, `hardwareStamp` for
`swerve` and `soft-limits`).

### Robot boot reconciliation

On the main thread at boot; resolved values and their sources are logged as AdvantageKit inputs so
replay matches. First, `_superseded` entries are cleared only for files whose current hash matches
`fetchedFileHashes`. Then each saved key goes through these rules; the first match wins:

1. Hardware stamp mismatch (`swerve`, `soft-limits`): refuse the file, alert, use the deployed value.
2. Key not in the deployed schema: move to `_orphaned`, warn, ignore.
3. `saved.value == deployedValue`: delete the key (the value is in the deployed code).
4. `saved.baseDeployedValue == deployedValue`: keep the override (`saved.value` wins).
5. Base differs and `savedOnDirtyDeploy`: keep the override and flag `DIRTY_CONFLICT`.
6. Base differs and (`<group>.json` in `dirtyParamFiles` or ancestry `"ancestor"`): deployed value
   wins; move the robot's key from `values` to `_superseded`.
7. Base differs and ancestry `"diverged"`, `"unknown"` or missing: keep the override and log
   `BRANCH_MISMATCH` or `UNKNOWN_SAVE_COMMIT`.

### Promoting the inbox to git

`./gradlew applyRobotParams`, or an AI session following AGENTS.md, reads `.robot-inbox/<robot>/`
(including `_superseded`, to recover an overridden value). Clean overrides (base equals the working
tree value, not saved on a dirty deploy) are written into the tracked params files for `git diff`
review. Dirty-deploy saves, conflicts, and hard-stop groups need a person's confirmation; hard-stop
changes also get the AGENTS.md PR block (old value, new value, failure mode).

### 2027 and AdvantageKit rules

2027 APIs only (`Timer.getTimestamp`, `MatchState` / `RobotState`); `System.nanoTime` for elapsed
time during init. Logs through `LogStorage`. Anything robot code reads goes through logged inputs on
the main thread.

## Choices to re-check

The team chose these from options that were framed by earlier, contaminated review rounds. They are
open again; a review may confirm or replace them.

1. ~~The service is written in Java.~~ Replaced: the team chose a separate Python web app (Rules).
2. ~~Pulling tuned values back only warns at deploy, with no terminal prompt.~~ Replaced by the params sync proposal.
3. ~~When git changed after a value was saved, both values are shown and a person chooses.~~ Replaced by the params sync proposal.
4. ~~The robot reports at boot when a saved hard-stop value differs from git, with a banner until
   resolved.~~ Replaced by the params sync proposal.
5. `lib/nt4.js` keeps each topic's last-update time.
6. Gradle generates a module of keys, params and button bindings from the Java; a drift check runs
   in CI and locally.
7. The shared Java part lives in `spectrumLib`.
8. Hard-stop changes are confirmed by a click on a page.
9. ~~The existing Power, CAN and Cameras pages are rebuilt.~~ Decided: replace them with robot-specific pages (Rules).

## Open questions

1. What runs the app on SystemCore, in what language, and how it is installed and updated by a
   deploy within the time budget.
2. How the pages are built (libraries, build step or none) for code maintained by AI.
3. How pages stay correct as the robot code changes.
4. How guided procedures get live readings, refuse bad data, and work with driver-button tests in
   utility mode.
5. How tuning edits reach the robot and how saved values get back into git.
6. How a person confirms a hard-stop change, given the team tunes without a controller.
7. How network discovery finds devices, including unconfigured ones on a default address.
8. Log download and file viewing/transfer: what, which direction, and how.
9. What it can do while the robot program has crashed.
10. What it replaces versus explains and links to.
11. How AI sessions use it.
12. Answered: a package with `X-Has-UI` and `X-Port` gets a tile linking to that port's root (see
    Verified facts).
13. What carries over to the next robot.
14. How the custom dashboard is tested before it is used in matches.

## Verified facts

From Limelight's public SystemCore OS repo ([systemcore-os-public](https://github.com/LimelightVision/systemcore-os-public), README release notes and `package_examples/`):

* System Python has been on the image since OS alpha 11 ("venv support and pip support to system
  python"; gunicorn, flask-sqlalchemy, requests). Limelight controls its version.
* Add-on packages are `.ipk` files with a systemd unit and a control file (`X-Port`, `X-Has-UI`,
  `X-Auto-Start`, `X-Icon-Path`, also `X-Https`, `X-Services`). The web UI shows packages as tiles.
  The image ships `opkg-utils`.
* Installing a package stops the robot program and restarts it after. Not true on our robot (below).
* SystemCore's web UI already has per-service log download, a support bundle with optional robot
  logs, CAN utilisation, a cameras portal, ports and firewall views, storage use and a live robot
  program log.
* Ports (third-party teardown): 4800 web UI, 4803 package manager, 4900 VS Code server, 4901 web
  terminal, 5800 CAN sniffer, 5812 NT4. Our robot differs on 4800 and 5800 (below).

From 8515's SystemCore (2026-10-29, OS `6eab208`):

* Installed packages: `Blocks` 0.2.2, `elastic_dashboard` 2027.0.0-alpha8, `canivore-usb` and
  `canivore-usb-kernel`. `opkg` and `opkg-build` are in `/bin`. Package metadata lives in
  `/var/lib/opkg/info/<name>.{control,list,postinst,prerm,postrm}`.
* Elastic is the closest model for the app: a static web app served by a small Python script.
  Control file: `X-Port: 5803`, `X-Display-Name: Elastic`, `X-Has-UI: true`, `X-Auto-Start: true`,
  `X-Icon-Path: /usr/share/elastic_dashboard.png`. Its unit
  `/etc/systemd/system/elastic_dashboard.service` runs `python3 start_elastic.py` as root from
  `/usr/local/bin/elastic` with `Restart=always`, `ProtectSystem=strict` and `ProtectHome=true`.
  `postinst` runs `daemon-reload`, `enable` and `start`; `prerm` stops and disables it.
* Blocks runs gunicorn from its own venv in `/opt/blocks` on port 9001.
* The web UI page is on port 80. Port 4800 is `diagnosticsproc` (a backend, not a page) and 4803 is
  `packagemanager`.
* A tile shows only when the package has a UI and a port above 0. It links to
  `http://<robot>:<X-Port>/`, the root with no path (`https` when `X-Https` is set). The icon is
  served by the package manager from `X-Icon-Path`. So a shortcut in SystemCore's web UI works, and
  the app must answer at `/`.
* Install paths: the web UI uploads to the package manager (`POST /packages/upload`, or
  `/packages/upload_async` with `/packages/install_status/<id>`). The `systemcore` user (the one
  GradleRIO deploys as) has passwordless sudo, so a deploy can also run `sudo opkg install` over
  SSH.
* A Gradle deploy can install a package (tested 2026-10-29, branch
  `robot-app/ipk-install-test`). Gradle builds the `.ipk` itself (an ar archive of `debian-binary`,
  `control.tar.gz`, `data.tar.gz`; no tools needed on the laptop). A `FileArtifact` copies it and
  runs `sudo opkg install --force-reinstall` after the copy. The test package `robot-app` holds only
  a systemd unit (Python's `http.server` as `systemcore`, serving `/home/systemcore/deploy` on 5808)
  and an SVG icon, so Gradle keeps owning the files it serves. Install took 3 s and the service came
  up. The package manager's list
  (`http://<robot>:4803/api/packages/launcher`) showed it with its icon, and its tile in the web UI
  opened `http://10.85.15.2:5808/`. Remove it with `sudo opkg remove robot-app`.
* Installing or removing a package does not touch the robot program. A web UI upload (saved to
  `/tmp/package_uploads/`, then plain `opkg install`, then two systemd reloads) and an `opkg remove`
  over SSH both left it running. A Gradle install stops it only because GradleRIO stops the robot
  program before any artifact runs.
* Port 5800 is SystemCore's `visionserver`, as are 5802 and 5805-5807. The robot program's Java
  listens only on 5810 (NetworkTables). Ports in use from 5800 to 5837: 5800-5803, 5805-5807, 5810,
  5812, 5815-5817, 5819, 5820, 5822, 5825-5827, 5832, 5835-5837.
* SystemCore has 4 GB of RAM (team). Java runs on it.

From this repo:

* The current first version is served by `WebServer.start(5800, ...)` inside the robot program
  (`Robot.java:294`), so it stops when the robot program stops, and on SystemCore it collides with
  `visionserver` on 5800. `LimelightHelpers` port forwarding
  uses 5800-5819.
* Params (`spectrumLib/params`, README "Params"): values declared in code; layers are the code
  default, `src/main/deploy/params/<group>.json`, `params/robots/<robot>/<group>.json`, the robot's
  own `/home/systemcore/params/<robot>/<group>.json` (not wiped by a deploy), then live changes over
  NT `/Tuning/`. Live changes need `Constants.TUNING_MODE` (false; a test fails a PR to `main` that
  leaves it on), apply on the main thread only while disabled with no FMS, and are logged as
  inputs. `/Tuning/SaveParams` writes values that differ from git to the robot's folder. `swerve`
  and `soft-limits` files carry a hardware stamp; swerve offsets are read once at boot. Nothing yet
  copies saved values back into git.
* `eventDeploy` runs `git add -A` and commits on `event*` branches before a deploy.
* `lib/nt4.js` passes each value's timestamp to its callback but does not store it per topic.
* `tools/make_log_key_guide.py` generates `log-keys.json`; CI checks it. No check compares the keys
  a page reads with the code.
* The robot's CAD is in `advantagescope-custom-assets/Robot_FM/`.

## Glossary

* **SystemCore**: the 2027 FRC robot controller, made by Limelight.
* **`.ipk`**: SystemCore's add-on package format.
* **Hard-stop value**: a value AGENTS.md says needs a person's confirmation to change (CAN IDs,
  inversions, swerve offsets, current limits, soft limits, gear ratios, brake/coast, zero positions
  and others).
* **Params**: tuned values declared in code and layered from files (see Verified facts).
* **Utility mode**: the 2027 name for test mode.
* **FMS**: the Field Management System that runs official matches.
* **Logged input**: a value AdvantageKit records each loop so replay reproduces it.
