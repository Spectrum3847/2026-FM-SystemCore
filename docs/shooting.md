# Shooting

## Aim feedforward

While aiming (`PILOT_AIM_AT_TARGET`: pilot X, every RT launch, and every auto `launch()`), the
heading request gets a feedforward: how fast the bearing to the hub is turning, `-v_tangential
/ distance` (581's form), in rad/s counter-clockwise, clamped to +/-2 rad/s (2910's clamp). The
heading PID alone only turns once it has fallen behind. In the `shoot` sim, strafing past the hub
at about 1 m/s from 2.5 m, the heading error went from a steady 4-5 deg to about 1 deg.

Differentiating `ShotCalculator`'s drive angle instead makes the aim oscillate (6-9 deg mean
error). That angle includes the shoot-on-move yaw offset, which follows the measured velocity, and
the turning itself disturbs that velocity. Logged as `Swerve/Aim/TargetRateFeedforward` (deg/s) and
`Swerve/Aim/HeadingErrorDeg`.

## Shot readiness gate (drivers: this changes how RT feels)

**FM on 2026 `main` fed the moment RT was pulled**, whether or not the flywheel was up to speed,
the hood had arrived or the robot had finished turning to the hub. With the gate, the indexers hold the fuel
(tower stopped, bed slow-indexing as when intaking) until the shot is ready, then feed at full
speed. Drivers should practise with it: the first ball can come a few tenths later than they are
used to, and pre-spinning with X (track target) before RT takes most of that away. Logic in
[ShotGate](../src/main/java/frc/robot/subsystems/ShotGate.java), from the offseason bot's feed gate
(dcb88dc, 067af02, ddd564a) with the drivetrain's heading in place of its turret.

| | Start a volley | Keep feeding |
|---|---|---|
| Hood | within 1 deg, and moving under 20 deg/s | within 3 deg |
| Heading (while the drivetrain aims) | within `atan((hub radius - ball) / distance)` (4499), 2-10 deg; 10 deg for feed shots (9470) | twice the start tolerance |
| Range | distance inside the model's fit, only while vision is under 3 s old | not checked |
| All of it | held for 0.04 s | |

Flywheel speed is not checked: waiting for it slowed FM's shots.

- **Timeout**: held for 1.0 s in a launch state, it feeds anyway for the rest of that launch, so a
  stuck sensor or an unreachable target can delay a shot by a second but never stop one. This is
  what keeps autos safe (their `launch()` window is 2.5 s).
- **Bypass**: **operator Y** (without LB; LB+Y is the intake reset) feeds while held.
- Leaving the launch state resets it: the next volley re-earns the start window.

On the dashboard (Match and Shooting tabs): `Shooting/Gate/Ready` (also lit while tracking with X,
before RT) and `Shooting/Gate/BlockedReason`: `None`, `NoShot`, `HoodNotZeroed` (see
[Hood homing](#hood-homing-drivers-no-more-pushing-the-hood-down)), `HoodNotReady`, `NotAimed`,
`InvalidShot` or `Settling`. Also logged: `Shooting/Gate/Feeding`, `Shooting/Gate/FeedReason`
(`Ready`/`Override`/`Timeout`/`Held`), the errors against each target, and running counts.
`tools/wpilog_summary.py` prints each launch, how long it was held and by what.

In the sim the hood overshoots on Motion Magic, which is why its speed is checked. **CALIBRATE
all of it from the first practice log** (`Shooting/Gate/FlywheelErrorRPM`, logged for reference only,
`Shooting/Gate/HoodErrorDeg`, `Shooting/Gate/HeadingErrorDeg`, `Shooting/Gate/SecondsHeld`).

## Hood homing (drivers: no more pushing the hood down)

**The hood homes itself at auto and teleop start; pushing it down before power-on is no longer
needed.** FM used to take wherever the hood rested at boot as 9 deg. At TXMAN1 it did not always
rest on its stop, and in four matches every hood angle read 2.5 to 11 deg low.

Now the hood does not trust its angle until it has found the bottom stop
([HoodHoming](../src/main/java/frc/robot/subsystems/hood/HoodHoming.java)): it drives down at a small
voltage until it stops moving, then sets its position to the stop. That runs at auto start, at
teleop start unless it is already zeroed, and briefly each time the hood is sent home (stow, force
home) and gets within 2 deg of the stop. After that re-home it rests on the stop in brake instead
of holding a setpoint against it. It only moves while enabled.

Until the first homing succeeds the hood holds in brake and refuses setpoints, and the shot gate
reports `HoodNotZeroed` (its 1 s timeout still feeds). On the Match tab, **Hood Zeroed** is red
until then; that is expected before the first enable. Logged as `Hood/Zeroed` and
`Hood/HomingState` (`NotZeroed`, `Homing`, `Zeroed`, `TimedOut`). If an attempt runs 3 s without
finding the stop, a "Hood homing timed out" alert goes up and the next auto or teleop start tries
again.

**CALIBRATE** (`HoodHoming.Params.fm()`): homing voltage -1.5 V, stalled under 4 deg/s for 0.2 s,
timeout 3 s, re-home within 2 deg of home. Bench checklist, robot on blocks, AdvantageScope on
`Hood/PositionDegrees`, `Hood/RPM`, `Hood/StatorCurrent`, `Hood/HomingState`:

- [ ] Lift the hood to the top by hand (coast with A while disabled, then brake with B), enable
      teleop. It must travel all the way to the stop and read `Zeroed`. Time it: well under the
      3 s timeout. If it stalls part way, raise the voltage; if it slams the stop, lower it.
- [ ] Check the stall current against the 60 A stator limit, and that the hood does not bounce
      off the stop (a bounce under 4 deg/s for 0.2 s zeroes on the bounce).
- [ ] With the hood on the stop, `Hood/PositionDegrees` reads 9.0. Check 9 deg against CAD or an
      angle gauge; if the stop is not at 9 deg, change `stopDeg`.
- [ ] Aim with X, release: the hood comes home, homes once (`Homing` for about 0.25 s), then sits
      `Zeroed` with low current.
- [ ] Unplug the hood's CAN, enable: it must not zero (`NotZeroed`, gate `HoodNotZeroed`).

## Set shots (fixed shots for when the pose is gone)

From the offseason bot (463c465, fa1376b), without its turret and without its hub-face shot (FM
cannot make it). Park at a known spot, point the **back** of the robot (FM's launcher side) at the
hub, and hold the chord: the hood and flywheel go to FM's own hub model evaluated at that spot's
range standing still, with the operator's hood trim, and the gate feeds once the hood is
there. Nothing reads the pose and the aim is not checked: as on the offseason bot, **the driver
aims by parking**, with translation slowed to the launch states' 10%. Everything is in
`ShotCalculator.SetShot` (spots per alliance from `Field`, rotated about the field centre for red).

| Chord (pilot, teleop) | Spot (blue; red is rotated) | Heading | Range | Hood / RPM (no-ceiling model, trim -2) |
|---|---|---|---|---|
| **LB + A** | Tower: intake against the tower's field-facing wall on its centreline, (1.525, 3.746) m | 185.3 deg: square to the wall, then about 5 deg counter-clockwise | 3.11 m | 14.7 deg / 2074 RPM |
| **LB + X** | Left trench: in the lane, just clear of the trench on the alliance side, (3.609, 7.430) m | 106.7 deg | 3.54 m | 16.4 deg / 2129 RPM |
| **LB + B** | Right trench: the mirror, (3.609, 0.639) m | -106.7 deg | 3.54 m | 16.4 deg / 2129 RPM |

Hood and RPM follow the Hub Model chooser and the hood trim live; the table is the default model.
`Shooting/Gate/SetShotHeadingErrorDeg` logs the fused heading against the spot's (information only).
**CALIBRATE** before relying on them: FM's bumper-to-centre (`FM_HALF_LENGTH_METERS`, 0.42 m from
PathPlanner's 0.84 m frame), and where the tower's centreline really is (`TOWER_CENTRE_Y_METERS`:
the offseason code says it follows tag 31, FM's `Field` says the field centreline; the range barely
changes, the heading does). In the `shoot` sim: tower 5 of 8, each trench 7-8 of 8.

## Shot log

One row per volley under `Shooting/Log/`, written on the loop the gate opens the feed: distance, radial
and tangential velocity, hood and RPM target and actual, heading error and tolerance, the gate's
reason (and what an override or timeout bypassed), how long it waited, pose trust
(`SecondsSinceFusedEstimate`, seed confirmed), set shot or calculated, alliance, mode and match
time; an end row gives the volley's length and its flywheel dips (roughly, balls; calibrate first).
From the offseason bot's shot record (eb2d86e). Schema: [shot-log.md](shot-log.md).

```bash
python tools/shot_log.py logs/akit_XXXX.wpilog > shots.csv
```
