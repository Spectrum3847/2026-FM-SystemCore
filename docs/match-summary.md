# Match summary

The robot keeps a short summary of each run as it happens
([`MatchSummary`](../src/main/java/frc/spectrumLib/telemetry/MatchSummary.java), in spectrumLib so
every Spectrum robot can use it). It is in two places:

- **Log keys** `Summary/...`, in every log, practice or match, updated once a second.
- **A file** `<log name>.summary.json` next to the log (`akit_26-10-16_20-11-57_txman1_q34.wpilog`
  gets `akit_26-10-16_20-11-57_txman1_q34.summary.json`), written only for FMS matches (match
  type not None). It is about 2 KB, and the pit laptop, the log-triage skill and the log site can
  read it before parsing the log.

The file is written on the loop each of these happens, and each write replaces the last:

| `writtenOn` | When |
| --- | --- |
| `autoEnd` | auto is disabled |
| `dsLost` | the Driver Station stops answering (in an FMS match this loop or the last, since the match type can go with the DS) |
| `eStop` | the robot is E-stopped (or A-stopped) |
| `teleopEnd` | teleop is disabled: the end of the match, or a drop within it |

So a robot E-stopped in auto, dead after auto, or rebooted mid-match still has a file: the matches
most worth one are the ones that never reach the end of teleop. A reboot starts a new log, so the
log from before it keeps the file written up to then.

The scope is the whole log, from boot. The robot is powered off between matches, so that is one
match plus the time in the queue.

Every value comes from logged inputs (motor inputs, the Driver Station, the loop clock, the
`Runtime/BatteryVoltage` and `Runtime/BrownedOut` inputs) or from code that runs on them (vision
gates, alerts), so replaying a log recomputes the same summary under `ReplayOutputs/Summary/`.
Replay never writes the file.

The file is built on the main thread and written by a background thread: a
`.tmp` file, synced to disk, then renamed over the target, so a power cut leaves the old file or
the new one, never half of one. `Summary/File` logs the file name written, or why it failed.

## Log keys

| Key | Meaning |
| --- | --- |
| `Summary/Battery/MinVolts` | lowest battery reading (readings at or below 1 V are no reading) |
| `Summary/Battery/SecondsUnder8V`, `SecondsUnder7V` | time below each, summed per loop |
| `Summary/Brownouts` | times the controller browned out (0 where the brownout API fails) |
| `Summary/Loop/EnabledLoops`, `Overruns`, `LongestMs` | enabled loops only; an overrun is a loop over 1.25 periods |
| `Summary/DS/Drops`, `DropsWhileEnabled`, `SecondsWithout` | Driver Station drops after it first connected |
| `Summary/CAN/<bus>/Drops`, `DownSeconds` | the bus counts as down while more than half its devices that have ever connected are not answering |
| `Summary/Devices/Drops`, `Reboots` | totals over every tracked CAN device |
| `Summary/PeakTempC/<mechanism>` | hottest reading while the motor was connected |
| `Summary/Vision/<source>/Accepted`, `Rejected` | the source's running counts since boot |
| `Summary/Auto/Routine`, `Ended`, `Seconds` | the auto that ran, why it ended and how long it ran, as `Auton/EndReason` and `Auton/EndElapsedSeconds` logged them (`FINISHED`, `END_OF_PERIOD`, `DISABLED_BY_FMS_EARLY`, `DS_DISCONNECTED`, `E_STOP`, ...; see `AutoEnd.Reason`); empty while it runs |
| `Summary/Alerts/ErrorsRaised`, `WarningsRaised` | times an error or warning alert turned on (seen once per loop, so an alert cleared and set again within a loop is not raised again) |
| `Summary/Alerts/Raised` | each distinct error and warning text, as `error x2: <text>` |
| `Summary/MatchesEnded` | FMS teleop ends seen |
| `Summary/File` | the last file written, or the error |

Durations credit at most 0.5 s per loop, so a stall does not count as time spent below a voltage.

## File (`"version": 2`)

Times (`...At`, `...Start`, `...End`, the start of each event string) are robot seconds, the log's
own timestamps, so they can be found in AdvantageScope. A value that never happened is `null`.

Trimmed from TXMAN1 Q34 replayed through this code (the full file is 4.3 KB), where the CANivore
dropped for 0.6 s in teleop:

```json
{
  "version": 2,
  "writtenAt": 679.733,
  "writtenOn": "teleopEnd",
  "match": {"event": "TXMAN1", "type": "QUALIFICATION", "number": 34, "replay": 1,
            "alliance": "RED", "station": 1, "matchTimeAtEnd": 0.0},
  "phases": {"autoStart": 516.676, "autoEnd": 532.154, "teleopStart": 539.724,
             "teleopEnd": 679.733},
  "battery": {"minVolts": 7.181, "minAt": 632.463, "secondsUnder8V": 2.26, "secondsUnder7V": 0.0},
  "brownouts": {"count": 0, "seconds": 0.0, "firstAt": null},
  "loop": {"enabledLoops": 15538, "overruns": 2, "overrunMs": 12.5, "longestMs": 109.669,
           "longestAt": 609.413},
  "ds": {"drops": 1, "dropsWhileEnabled": 0, "secondsWithout": 19.55,
         "events": ["472.27 drop", "491.82 back"]},
  "can": {"SystemCoreCAN0": {"devices": 2, "drops": 0, "downSeconds": 0.0, "events": []},
          "CANivore": {"devices": 16, "drops": 1, "downSeconds": 0.63,
                       "events": ["608.86 drop", "609.49 back"]}},
  "devices": {"Hood": {"bus": "CANivore", "drops": 1, "reboots": 0,
                       "events": ["608.84 drop", "609.62 back"]},
              "Pigeon": {"bus": "CANivore", "drops": 1, "reboots": 0,
                         "events": ["608.87 drop", "609.49 back"]}},
  "neverConnected": [],
  "vision": {"LL-Left/MT1": {"accepted": 1770, "rejected": 2942}},
  "auto": {"routine": "TBTB Full - Right", "ended": "END_OF_PERIOD", "seconds": 15.478},
  "alerts": {"errorsRaised": 7, "warningsRaised": 8, "infosRaised": 2,
             "raised": [{"level": "error", "firstAt": 608.874, "count": 1,
                         "text": "Pigeon IMU Disconnected"}]},
  "peakTempC": {"IndexerBed": 61.0, "Launcher": 33.0, "Hood": 26.0}
}
```

- `devices` lists only devices that dropped or rebooted, with up to 8 events each
  (`moreEvents` counts the rest). `neverConnected` lists devices that never answered.
- `alerts.raised` lists each distinct error and warning text once, up to 20 (`moreTexts` counts
  the rest). Info alerts are only counted.
- `auto.ended` is the reason `AutoEnd` logged as `Auton/EndReason`, verbatim, and `auto.seconds`
  its elapsed time; `ended` is `null` if the auto had not ended when the file was written.
- Adding a field does not change `version`; renaming or removing one, or changing what one means,
  does. Version 2: `auto.ended` became `AutoEnd`'s reason (version 1 had `finished` or `cut off`),
  and `writtenOn` was added.

## Using it on another robot

Create it right after the logger starts, so alerts raised during startup are counted, then
register what the robot has and call it once per loop after the command scheduler:

```java
matchSummary = new MatchSummary(logFolderOrNullInReplay);
matchSummary.addMechanism(launcher, busLabel);           // leader and followers
matchSummary.addDevice("Pigeon", "CANivore", () -> in.gyroConnected, () -> in.gyroResetOccurred, null);
matchSummary.addVisionSource(s.getName(), s::getAcceptedCount, s::getRejectedCount);
// autonomousInit(): matchSummary.autoStarted(name);
// when the auto command ends: matchSummary.autoEnded(reason, seconds);
// robotPeriodic(), after the scheduler: matchSummary.periodic(batteryVoltsInput, brownedOutInput);
```

In the sim, `FM_SIM_MATCH=Q12` makes the Driver Station report an FMS match, so a scripted run
writes the file into `logs/`.
