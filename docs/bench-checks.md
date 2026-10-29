# Bench checks

What the sim can't show: CAN drops, motor faults, reboots, the Jetson, and the real cameras. Run the
checks a PR touches on the robot before an event, and say which ones you ran under "How I verified".

Robot on blocks, wheels off the ground, someone at the Driver Station. Open the log in AdvantageScope
afterwards (or `python tools/wpilog_summary.py <log>`). Every key below is described in the log's
`LogKeyGuide` metadata and in [log-keys.md](log-keys.md).

## Every deploy

- [ ] `/RealMetadata/GitSHA` matches the commit you deployed, and `LogKeyGuide` is present.
- [ ] `Events` starts with `disabled`. Enable teleop, then disable: `teleop`, then `disabled`.
- [ ] `Scheduler/RobotPeriodic` (worst loop each second) stays under 10 ms while disabled.

## CAN

- [ ] **A drop.** While disabled, unplug the CANivore's USB cable for about 5 s, then plug it back in.
  - `Mechanisms/<name>/Connected` goes false, then true again.
  - `/CAN/CANivore/StatusOK` goes false.
  - `Summary/CAN/CANivore/Drops` goes up by 1.
  - `Events` shows the disconnect alerts raised and cleared.
- [ ] **A reboot.** Enabled with nothing moving, pull one mechanism motor's breaker for 2 s and put it back.
  - `Mechanisms/<name>/ResetOccurred` is true for one loop.
  - `Mechanisms/<name>/TroubleFaults` shows `BootDuringEnable`.
  - The summary lists the device with a reboot and that fault (`Summary/Devices/Faulted`).
- [ ] **No false faults.** After a normal practice run, `Summary/Devices/Faulted` is 0. Homing against a stop must not show up here (that's a current limit, not a fault).

## Power

- [ ] While enabled, the battery sags under load (a fast spin, or the intake stalled briefly):
  `Runtime/BatteryVoltage` drops, and if it reaches brownout, `Events` shows `brownout (<V>)` and
  `brownout over`.

## Jetson and cameras

- [ ] **A reboot.** Power-cycle the Jetson while disabled.
  - `Vision/Jetson/BootId` changes, and the "Jetson rebooted" alert shows.
  - `Vision/Jetson/UptimeSeconds` restarts near 0.
- [ ] The Jetson's diagnostics have real values: `Vision/Jetson/InputVoltageV` (about 12 V or more), `Eth/LinkUp` true, `ClockSource` `robot`.
- [ ] Each camera's `Health/State` reads `streaming`. Unplug one camera: it goes to `disconnected`, then back to `streaming` when plugged in.

## Heading

- [ ] **Still.** Disabled, parked where LL-Back sees two or more tags, for 60 s: `Vision/BackHeadingCorrections` doesn't go up.
- [ ] **Driving.** After driving a few laps in front of the tags: corrections are rare (TXMAN1 playoffs had 19-33 a match before the fix, which was the bug). Shots don't drift left or right over the run.

## Driver inputs

- [ ] Press the operator's hood and drive-angle d-pad trims: each press shows in `Events` (`hood angle offset ... deg`).
- [ ] Press a few bindings: `Commands/Events` shows named commands (`SuperStructure.setState.INTAKE_FUEL`), not `InstantCommand`.

## Match summary

- [ ] After a practice match with the FMS (or a field timer), `<log>.summary.json` sits next to the log, and its `can`, `devices` and `alerts` sections match what happened.
