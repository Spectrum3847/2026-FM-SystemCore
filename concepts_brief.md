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
  "someone plugs in an unconfigured device and need to know where it's webpage is to configure it."
* "Log downloading, viewing the system files and transferring things, etc." [What needs to move?]
  "Largely just logs off the robot, there isn't a convenient way to do it, you have pull up winscp
  because the other tools fail sometimes, or know how to do it in the command line."
* "Things you can do thru should be able to be controlled by AI agents usually." [Run alone?] "I
  don't expect the agent to run by itself we mean a ai session being directed by a pit student or
  coach to figure things out and make the robot better."
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
* Turning tuning on: "Network tables request that can be hit from the website." No timer: "Wifi
  network does have security at events, so its not security risk and robot should only be on when
  people are working on it." "Turn off when fms attaches."
* Pulling values back: "Can the deploy ask what to do with the changes in the terminal? Or can it
  save a copy of the old params if there are changes then ask which one to keep after? Seems like
  there are various ways to solve this correctly."
* Hard-stop confirmation: the team tunes without a driver controller ("we tune and run without a
  controller for things all the time"), so confirmation cannot require one.

### Rules

* `spectrumLib` on `event*` branches: "Fixes to spectrum lib have to be able to made at events ...
  it's just that you shouldn't make minor tweaks, or feature updates." AGENTS.md now says so.
* The name is the Robot App.

## Choices to re-check

The team chose these from options that were framed by earlier, contaminated review rounds. They are
open again; a review may confirm or replace them.

1. The service is written in Java (rather than Python or another runtime).
2. Pulling tuned values back only warns at deploy, with no terminal prompt.
3. When git changed after a value was saved, both values are shown and a person chooses.
4. The robot reports at boot when a saved hard-stop value differs from git, with a banner until
   resolved.
5. `lib/nt4.js` keeps each topic's last-update time.
6. Gradle generates a module of keys, params and button bindings from the Java; a drift check runs
   in CI and locally.
7. The shared Java part lives in `spectrumLib`.
8. Hard-stop changes are confirmed by a click on a page.
9. The existing Power, CAN and Cameras pages are rebuilt.

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
12. Whether a shortcut can be added to SystemCore's web UI.
13. What carries over to the next robot.

## Verified facts

From Limelight's public SystemCore OS repo ([systemcore-os-public](https://github.com/LimelightVision/systemcore-os-public), README release notes and `package_examples/`):

* System Python has been on the image since OS alpha 11 ("venv support and pip support to system
  python"; gunicorn, flask-sqlalchemy, requests). Limelight controls its version.
* Add-on packages are `.ipk` files with a systemd unit and a control file (`X-Port`, `X-Has-UI`,
  `X-Auto-Start`, `X-Icon-Path`, also `X-Https`, `X-Services`). The web UI shows packages as tiles.
  Whether a tile links to the package's page is unconfirmed. The image ships `opkg-utils`.
* Installing a package stops the robot program and restarts it after.
* SystemCore's web UI (port 4800) already has per-service log download, a support bundle with
  optional robot logs, CAN utilisation, a cameras portal, ports and firewall views, storage use and
  a live robot program log.
* Ports (third-party teardown): 4800 web UI, 4803 package manager, 4900 VS Code server, 4901 web
  terminal, 5800 CAN sniffer, 5812 NT4.
* SystemCore has 4 GB of RAM (team). Java runs on it.

From this repo:

* The current first version is served by `WebServer.start(5800, ...)` inside the robot program
  (`Robot.java:294`), so it stops when the robot program stops. `LimelightHelpers` port forwarding
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
