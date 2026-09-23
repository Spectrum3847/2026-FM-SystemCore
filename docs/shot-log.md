# The shot log

One row per volley, so a practice session is a dataset instead of a memory. Ported from the
offseason bot's shot record (2026-Spectrum `2026-offseason-bot` eb2d86e, `docs/tools/shot-log.md`
there), extended with FM's shot gate and the pose's trust. Written by
[ShotLog](../src/main/java/frc/robot/subsystems/ShotLog.java), called from `SuperStructure`.

```bash
python tools/shot_log.py logs/akit_XXXX.wpilog > shots.csv          # what the robot did
python tools/shot_log.py logs/akit_XXXX_sim.wpilog --replay         # what replayed code did
```

## When a row is written

On the **rising edge of the feed**: the first loop the shot gate lets fuel into the flywheel, and
so the last loop on which the aim was still a prediction. A launch the gate opens, closes (a stall,
the heading swinging out) and opens again is two rows. When the feed closes an **end row** is
written under `ShotLog/End/`, joined to its row by index.

AdvantageKit records a value only when it changes, so a key that is the same as the last shot's
has no record at this shot's time. **Read a row as each key's last value at or before the row's
`ShotLog/Index` record**; `tools/shot_log.py` does exactly that.

## `ShotLog/*`, one row per volley

| Key | Meaning |
|---|---|
| `Index` | Volleys since boot. The row's key, and the one key on the dashboard (Shooting tab, "Shots"), so you can see rows being written. |
| `TimestampSeconds`, `MatchTimeSeconds` | Robot clock; match clock, for lining a row up with video. |
| `Alliance`, `Mode`, `SuperState` | Which alliance, `Auto` or `Teleop`, and which launch state. |
| `Kind`, `SetShot` | `Calculated` or `SetShot`, and which spot (`Tower`, `LeftTrench`, `RightTrench`). |
| `FeedReason` | Why the feed opened: `Ready` (the gate), `Override` (operator Y) or `Timeout` (held 1 s). |
| `Bypassed` | For `Override` and `Timeout`, what was not ready (`FlywheelNotReady`, `NotAimed`, ...). |
| `SecondsWaited` | From entering the launch state to this feed: what the gate cost. |
| `FeedShot`, `Model` | Feed (passing) shot or hub shot, and the fit it was evaluated on (`HubModelStandstill` for a set shot). |
| `DistanceMeters` | Real distance to the target (a set shot: the spot's range). Bin on this. |
| `LookaheadDistanceMeters` | The shoot-on-move virtual distance the model was evaluated at. |
| `RadialVelocityMs`, `TangentialVelocityMs` | Launcher velocity toward and across the target; the model is a surface in distance and radial velocity. Empty for a set shot. |
| `RobotSpeedMs`, `TimeOfFlightSeconds`, `InRange` | Measured speed; the model's flight time; distance inside the fit. |
| `WantedRPM`, `ActualRPM` | Flywheel target and measured. |
| `WantedHoodDeg`, `ActualHoodDeg` | Hood target (trim included) and measured. |
| `HeadingErrorDeg`, `AimToleranceDeg` | Fused heading minus the aim target, and the gate's tolerance at this range. Empty for a set shot (the driver aims). |
| `HoodTrimDeg`, `AimTrimDeg` | The operator's live trims. |
| `PoseTrusted`, `SecondsSinceFusedEstimate`, `SeedConfirmed` | Whether the distance can be believed: vision under 3 s old, how old, and whether the disabled seed was confirmed. |
| `Pose` | Fused robot pose. |

## `ShotLog/End/*`, when the feed closes

| Key | Meaning |
|---|---|
| `Index` | The row this ends. |
| `BurstSeconds` | How long the feed stayed open. |
| `FlywheelDips` | Dips of the flywheel more than 150 RPM below target, re-armed within 75 RPM (3467's flywheel-drop idea): roughly, balls. **CALIBRATE** against video before trusting it; the sim's flywheel does not feel fuel. |
| `MinRPM` | Lowest flywheel speed during the volley. |

## The outcome

As on the offseason bot, there are no made/missed buttons: the operator's D-pad trims are the
outcome signal (a shot going long gets the hood trimmed down). Join `ShotCalc/HoodAngleOffsetDegrees`
changes after a row on time. The offseason bot also persisted the trims through Preferences and
logged a row per trim press (`ShotCalc/Trim/*`); that is not ported.

First reading worth doing by hand (the offseason's advice): a bias in where shots land that is
constant across distance is an exit-speed error; one that grows with distance is an angle error.
