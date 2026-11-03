# Pose estimation: findings and improvement list

Where TagEkf's error comes from, what has been fixed, what is being tried, and what has to be
measured on each robot. Evidence is from 8515's TXMAN1 logs (13 quals and 7 test runs,
robotlog.spectrum3847.org), replayed with `TagEkfTuner`.

## Tools

```bash
python tools/slip_log.py robotlogs/X.wpilog                 # wheel scrub and wheel-vs-gyro yaw slip
python tools/export_tagekf_events.py robotlogs/X.wpilog     # once per log, for the tuner
./gradlew tagEkfTune -Plogs=robotlogs/DIR                   # noise sweep
./gradlew tagEkfTune -Plogs=robotlogs/DIR -Pmode=priors     # slip detector, bumps, walls/hubs
./gradlew tagEkfTune -Plogs=robotlogs/DIR -Pmode=priors -Pblackout=5   # same, tags off 5 s of 20
./gradlew tagEkfTune -Plogs=robotlogs/DIR -Ptrace=<log> -Psweep=false   # one log, every 2 s
```

Gradle needs WPILib's 2027 JDK (`JAVA_HOME=C:/Users/Public/wpilib/2027_alpha5/jdk`).

`TagEkfTuner` replays logged odometry and tags through `TagEkf` the way the robot does: odometry
through `OdometryGuard`, the tilt odometry scale and bump noise scale from `Swerve`, Limelight MT1
fiducials and Orin PhotonLib results through `Vision.addTagFrame`. Every 4th frame is held out.
Scored on:

- **held-out error** (p50, p90): each held-out tag's innovation, in metres, against the filter at
  its capture time;
- **NIS**: Mahalanobis² of those innovations; 2 means the filter's uncertainty is honest;
- **fed accept**: share of fed tags the gate accepted;
- **lost**: share of multi-tag MT1 poses the estimate was more than 1 m from.

**Cameras:** these results come from 8515's TXMAN1 setup, three Limelights plus two Orin
PhotonVision cameras, and most of the tag signal was the Limelights'. The future robot runs
PhotonVision only (Jetson cameras, plus SystemCore cameras as backup) with more and better cameras,
so re-tune the noise model, the tilt policy and the priors on its logs. The tuner already reads
Orin PhotonLib results; SystemCore cameras will need the same export. Orin-only replay on these
logs is much worse (r20 and r4 lost 75-84%), mostly because of the frozen Orin timestamps and
untuned Orin mounts below, not because the method is wrong.

Only logs from Q30 on carry per-tag data (Limelight fiducials, Orin results). Q30 is excluded from
tuning: LL-Back's MT1 pose sits 172° and 0.7 m from both side cameras, so that camera was mounted
or configured differently that match.

## What the logs showed

| Finding | Evidence | Status |
|---|---|---|
| Wheels count 13-97% more rotation than the gyro (typically 30%), and lead it by ~110 ms | All 20 logs; gyro heading matches vision (0.7-1.5° robust), wheels don't (2-14°) | TagEkf already takes heading from the gyro |
| Wheels overstate distance ~10% against vision | Median wheel/vision distance 1.02-1.18 over 0.1-0.5 s windows | Not changed: the tuner's wheel scale has no clear optimum while vision is this strong |
| TagEkf lost lock when tilted | r19 sat at a steady 6-8° for 13 s at a time (high-centred on the right blue bump, wheels spinning 14 m while vision showed it still); every tag above 5° was dropped | **Fixed**: tags dropped only above 20°, noise scaled by tilt in between. Lost 11.9% → 0.3% |
| A CAN drop moved the replayed pose | First tuner bypassed `OdometryGuard` | Tuner now uses the guard |
| Orin capture timestamps 1000x too small in replay | TXMAN1's PhotonVision put microseconds in the field this PhotonLib reads as `captureTimestampNanos` | Tuner works around it. **Open** in robot code: AdvantageKit replay of these logs rejects every Orin tag |
| Orin results with a frozen capture time | r4, r20: most results `sequenceID=0`, same timestamp | **Open**: Jetson pipeline bug; robot should alert |
| Live, Orin-Left lost 2615 of 3066 tags to the gate | r20 robot log | Mostly timestamps and mounts; recheck after calibration |
| `frc.rebuilt.Field` initialization cycle | Touching `Field.BlueHub` before `Field` throws in `Field.BlueToRed` | Worked around in `FieldPriors`; **open** in `Field` |
| Pigeon mounted turned around | Its X/Y acceleration sits ~175° from the wheels' on all 10 logs; `MountPoseYaw` is 0 | **Open**: yaw is unaffected; anything reading Pigeon X/Y acceleration is backwards |

## Improvement list

Ranked by expected payoff. Each is an optional TagEkf input, scored in replay before it goes on a
robot.

| # | Idea | Status |
|---|---|---|
| 1 | Tilt policy: feed tags while steadily tilted | **Done** (`Vision.addTagFrame`) |
| 2 | Slip detector: Pigeon acceleration vs wheel acceleration; inflate odometry noise while they disagree. Catches being pushed and spinning in place, which the four-module scrub term can't see | Built (`SlipDetector`), **off**: see results |
| 3 | Bump crossings as position: a steady tilt near exactly one bump fixes x to that bump | Built (`FieldPriors`), **off**: see results |
| 4 | Walls and hubs: the robot's centre can't be inside a hub or within half its size of a wall | Built (`FieldPriors`), **on** |
| 5 | Hub occlusion: a tag "seen" through a hub is a misread | Idea |
| 6 | Expected-tag check: from the estimate, predict which tags each camera should see; many missing or unexpected tags means the estimate is wrong, a better relocalize trigger than "20 rejections in 1 s" | Idea |
| 7 | Wall contact: high drive current, ~zero speed, estimate near a wall: touching it (distance, and heading for a flat side) | Idea; needs drive current as an input (below) |
| 8 | Trench passages: position across the trench is known to a few cm | Idea |
| 9 | Per-camera yaw bias state in TagEkf, estimated online | Idea |
| 10 | Known start poses as a pre-match camera-mount check | Idea (seeding already uses them) |

## Further ideas (future robot)

Numbered as first proposed; team decisions in the last column.

| # | Idea | Decision / notes |
|---|---|---|
| F1 | Tag corners instead of per-tag bearing and range | Tested: loses on TXMAN1 until mounts are calibrated (results below). Retest on the new robot |
| F2 | Per-tag noise from pixel size, viewing angle and calibration | With 0.2-0.5 ms exposures, motion blur drops out |
| F3 | Rolling-shutter correction | **Not needed**: global-shutter cameras |
| F4 | Exposure-centred timestamps | **Already done**; measure each camera's lag (spin test) |
| F5 | Reject tags with high corner reprojection error | Idea |
| F6 | Camera clock sync and drift check | PhotonVision already syncs to the SystemCore clock (to confirm). Still alert on drift or frozen capture times (TXMAN1 r4/r20 had frozen Orin timestamps) |
| F7 | Online per-camera mount estimate; **detect knocked cameras and turn them off** | Mount solving already done, more planned. `CameraHeadingCheck` flags cameras whose heading disagrees by >15°, pre-match (NO-GO); `VisionGates` rejects solves with roll/pitch beyond limits. **Not done**: in-match detection of a knocked camera from TagEkf's per-camera residuals (`EKF/Cameras/<cam>/BearingResidualsDeg`) and switching it off automatically |
| F8 | Cross-check cameras that see the same tag | Write it efficiently: compare per-camera residual means, not pairs of tags |
| F9 | Camera coverage | Goal: 360° or close, at least 4 tag cameras, possibly more |
| F10 | Per-module slip and scale | Yes: weight distribution differs per wheel |
| F11 | Commanded motion, voltage and current in the prediction | Yes. Also steer: commanded vs measured module angle |
| F12 | Fuse both gyros (Pigeon + SystemCore IMU), each calibrated | Yes: redundant gyros |
| F13 | Gyro bias from temperature, estimated while still | Yes: cheap |
| F14 | Fixed-lag smoother (look-back correction) | **Built** (`TagEkf.smoothedAt`), scored below; log both poses |
| F15 | 3-D state while tilted | Yes, to work out |
| F16 | Several pose hypotheses after a big hit | Yes |
| F17 | Self-checking uncertainty (widen when NIS stays high) | **On** (`TagEkf.setAdaptive`), scored below |
| F18 | Per-event tag-map calibration | **The plan** (`FIELD-CALIBRATION-PLAN.md`) |
| F19 | Field features besides tags (hub, bump, wall edges) | Research: FRC 900 and others have tried it |
| F20 | Pose regression check in CI | Idea |
| F21 | Ground-truth drill at events (taped spots) | Idea |

## Results: items 2-4

From `-Pmode=priors` on the 9 tuning logs, as-built noise, 20° tilt policy.

**With tags flowing normally, none of the three changes anything.** TagEkf already accepts 98% of
tags and is lost 0.3% of the time; held-out p90 stays at 0.241 m with bumps (135 fixes) and
walls/hubs (400-1400 pushes). The slip detector makes it worse at every setting (p90 0.25-0.28 m):
it flags 45k-117k loops, because Pigeon and wheel acceleration correlate at only ~0.6, and the
extra odometry noise makes the filter lean on noisy tags.

**They matter when tags are missing.** With fed tags blacked out 5 s of every 20 s
(`-Pblackout=5`; held-out tags still scored, so this measures drift):

| Input | Lost | Held-out p90 |
|---|---|---|
| None | 9.6% | 0.86 m |
| Wall/hub (std 0.05 m, half-size 0.40 m) | 7.0% | 0.73 m |
| Bump (std 0.2 m) | 8.2% | 0.80 m |
| Slip (4 m/s², hold 0.5 s, noise x20) | 7.8% | 0.77 m |
| All three (best of each) | **5.2%** | **0.66 m** |

Decision:

- **Walls/hubs: on** in `Vision` (std 0.05 m, half-size 0.40 m). Better at every setting tried in
  blackout (7.0-7.5% lost), neutral otherwise. Logged as `Localization/EKF/FieldConstraints`.
- **Bumps: built, off.** Helps at 0.2 m, hurts at 0.35 m: not consistent enough. Revisit once
  the Pigeon mount is fixed and with a better crossing model (which edge, which direction).
- **Slip detector: built, off.** Needs the Pigeon mount fixed, and drive current logged as an
  input, so wheel acceleration isn't the only slip evidence.

The blackout window is fixed (`t mod 20 < 5`), so these numbers carry some luck in which manoeuvres
fall in a gap; re-run with new logs before trusting small differences.

## Results: smoothing and self-checking uncertainty

`-Pmode=smooth` (add `-Pblackout=5` for outages). Held-out tags are scored twice: against the
real-time estimate at capture, and against the RTS-smoothed estimate 0.5 s later
(`TagEkf.smoothedAt`, `innovationSmoothed`). Self-checking uncertainty (`TagEkf.setAdaptive`):
before each frame, if its tags' mean NIS is above a threshold, P grows in proportion to the excess.

| | Normal p90 / lost | Blackout 5 of 20 s p90 / lost |
|---|---|---|
| As built | 0.241 m / 0.3% | 0.856 m / 9.6% |
| Smoothed (0.5 s look-back) | 0.226 m | 0.745 m |
| Self-checking, NIS > 10, gain 1 | 0.243 m / 0.2% | 0.672 m / 6.0% |
| Both, smoothed | 0.230 m | 0.602 m |

Decision:

- **Self-checking: on** in `PoseFusion` (NIS > 10, gain 1). Neutral normally, lost 9.6% -> 6.0% in
  outages. Every threshold tried (4-10) gave about the same blackout gain; 10 is the gentlest.
- **Smoother: built, not logged on the robot yet.** 6% better p90 normally, 13% in outages, at
  0.5 s lag. Next: log `EKF/SmoothedPose` with the time it refers to, and decide whether the shot
  calculator's look-back should use it. Replay can produce it for any log in the meantime.

## Results: tag corners instead of bearing and range

`-Pmode=corners`: Orin frames fed as each tag's four detected corner pixels (8 measurements per
tag, projected through the logged intrinsics and 8-term distortion; `TagEkf.addCornerFrame`,
`CornerSighting`), against the same frames as PhotonVision's per-tag bearing and range. Same
held-out tags.

| Fed | Bearing/range | Corners, best setting |
|---|---|---|
| Orin only | p90 7.5 m, lost 35% | p90 9.9 m, lost 49% (32 px) |
| Orin + Limelights | p90 0.24 m, lost 0.3% | p90 0.34 m, lost 1.6% (32 px) |

**Corners lose on these logs, and the reason is the mounts, not the method.**

- The camera model is right: projecting through PhotonVision's own per-tag solve lands within
  1 px of the detected corners (`-Pmode=cornercheck`).
- Against the estimate, the corners sit off by a steady amount per camera: Orin-Left 12-21 px in
  u (about 1-1.7° of yaw), Orin-Right 35-43 px in v (about 3° of pitch), with 17-35 px RMS on
  top. Bearing and range use only the horizontal direction and the range, so a pitch error barely
  touches them; corners use every pixel, so they need mounts (pitch, roll, height) good to a
  fraction of a degree. A 1° mount error is ~13 px at these focal lengths.
- A -3° pitch fix on Orin-Right (`-Pmountfix=0,0,0,-3`) halved its v bias but didn't close the
  gap: with uncalibrated mounts the best pixel std is 32 px, which throws away corners' advantage.

Retest on the new robot once camera mounts are calibrated (the first item below). Expected to
win there: global-shutter cameras, exposure-centred timestamps, and 0.2-0.5 ms exposures mean
almost no motion blur, so corner pixels stay at ~1 px even at speed.

## Camera notes (future robot)

- Global shutter: no rolling-shutter correction needed.
- Timestamps are already exposure-centred; what remains is measuring each camera's lag (spin test).
- Exposure moving to 0.2-0.5 ms: motion blur becomes negligible, so per-tag noise can depend on
  pixel size and viewing angle alone.

## What we have to know about each robot

Measure these per robot, per event if anything was touched. Most can be measured from tags.

| What | Why | How |
|---|---|---|
| **Camera positions and orientations**, every camera | Bearing and range are only as good as the mount; a 1° yaw error is 3.5 cm per 2 m of range | Park in front of a tag pair at known spots; solve each camera's mount from its tags against the field map. `MountCheck` / `CameraHeadingCheck` / the Orin's `Mount/*` estimates already do parts of this live |
| **SystemCore IMU and Pigeon orientation** | Gyro yaw is unaffected by a 180° yaw mount, but acceleration (slip detection, bump detection, wall contact) is. TXMAN1's Pigeon is ~175° off its configured mount | `ImuMountCheck`: drive straight and turn; fit IMU acceleration and rate to the wheels (`Swerve`'s `MountCheck/*` outputs) |
| **Wheel diameter, and slip at each acceleration** | Odometry scale; the wheels read ~10% long at TXMAN1 | Drive straight at a pair of tags from far to near at several speeds and accelerations; compare wheel distance to vision distance. Repeat after tread changes |
| **Camera measurement lag**, every camera | A 10 ms error is 4 cm at 4 m/s, and 0.6° per 1 rad/s of turn | Spin test: rotate in place in front of tags; fit the lag that makes vision heading match the gyro. Also: drive past tags at constant speed and fit the lag on position |
| **Gyro scale** | Heading error per turn: TagEkf's `headingVarPerRadian` assumes ~0.3% | Ten spins in place, compare the gyro to vision heading |
| **Limelight onboard mount settings** match the robot code | Q30: LL-Back's own MT1 pose was 172° off | Pre-match: all Limelights' MT1 poses agree within 0.2 m |
| **Orin capture-time units and sync** | Microseconds vs nanoseconds; frozen timestamps | Pre-match: Orin capture time within 0.1 s of robot time, `sequenceID` advancing |

## Logging requirements

Anything a pose gate or the EKF reads must be a logged **input**, at the rate it's used, or replay
can't reproduce it.

- **Drive motor current**: now part of the pose gates (wall contact, push detection). Today it is a
  ~20 Hz *output* (`RealOutputs/Swerve/Currents/...`, logged on change). Log each module's drive
  stator current as an input with every odometry loop.
- Limelight MT1 fiducials and Orin results: logged from TXMAN1 Q30 on; keep them.
- Pigeon pitch, roll, their rates and X/Y/Z acceleration: inputs every loop (they are).
- Orin result metadata (capture and publish times, sequence id): in the logged bytes; keep it.
