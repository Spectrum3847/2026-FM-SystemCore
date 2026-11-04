# Pose sources: design

Besides getting FM onto SystemCore, this repo takes robot-pose estimates from several independent
sources, gates each one with its own rules, and logs each one well enough to compare offline. The
goal is **the comparison, not the fusion**: which source to trust, when, and what log evidence
shows it.

This follows the handoff brief (`HANDOFF-fm-pose-sources.md`, 2026-09); its decisions hold, except
that fusion is two estimators compared side by side:

- Fusion runs WPILib's `SwerveDrivePoseEstimator` **and** an EKF alongside it, both fed the same
  odometry samples. The EKF is logged for comparison only; the robot drives on WPILib's (see
  [EKF alongside WPILib](#ekf-alongside-wpilib)).
- Fusion runs where the drivetrain loop runs. FM's drivetrain is on SystemCore, so fusion is too.
- The Orin runs PhotonVision (the 971 CUDA detector fork), read with PhotonLib.
- Comparison happens offline against logs; AdvantageKit's deterministic replay is why AKit is used.
- Thresholds are ported from the 2026 code with their comments; change them only with log evidence.

## The pieces

All in [`frc.spectrumLib.localization`](../src/main/java/frc/spectrumLib/localization):

| Class | Job |
|---|---|
| `PoseObservation` | One measurement: source, kind, capture time, `Pose3d`, tag count (−1 for none), avg tag distance, target size, max ambiguity, tracking flag. The 2026 `VisionFieldPoseEstimate` promoted out of `Vision`. |
| `PoseSourceIO` / `PoseSourceInputs` | The device layer. The IO fills the inputs; the inputs are an AKit `LoggableInputs`, logged **before any gate**. In replay the IO is `NONE` and the inputs come back from the log. |
| `Gate` | One named rejection rule. A source declares its own chain. The first gate to fire names the rejection. |
| `StdDevModel` | How much to trust a frame that passed: returns std-devs and a tier name, or refuses ("Integration Criteria not Met"). |
| `PoseSource` | Device + gates + model + a dashboard enable switch. Runs every loop whether enabled or not. |
| `OdometryGuard` | Screens each odometry sample: a module counts only when connected, not a dead 0 read, and within 1.5x top speed of its last accepted reading. Motion comes from the modules used (two or more, least squares); heading from the Pigeon, then SystemCore's onboard IMU, then the wheels. A CANivore drop freezes odometry and vision holds the pose. |
| `PoseFusion` | The fused estimator, a never-corrected odometry track, **one shadow estimator per source**, and the tag EKF. |
| `TagEkf` / `TagSighting` | The all-camera EKF and one tag seen by one camera. |

In [`frc.robot.subsystems.vision`](../src/main/java/frc/robot/subsystems/vision):

| Class | Job |
|---|---|
| `Vision` | Builds every source, runs them each loop, applies the fusion policy, seeds the pose while disabled, runs the gross-heading safety net, pushes heading to the Limelights. |
| `VisionGates` | The AprilTag gate chain and the Limelight / Photon trust models, with the 2026 rationale. |
| `LimelightIO` | One Limelight's MegaTag1 **or** MegaTag2. Each Limelight is two sources. |
| `PhotonIO` | One PhotonVision camera (multi-tag solve if present, else best single tag). |
| `SimVision` | PhotonLib `VisionSystemSim` for every AprilTag camera, Limelight emulation on top of it. |

## Every loop

```
Swerve.updateInputs()    odometry samples (250 Hz, queued off CTRE's thread) -> logged inputs
                         -> OdometryGuard -> PoseFusion
Vision.periodic()
  every source: update()     device -> logged inputs (raw, pre-gate)
  every source: process()    gates + trust model -> verdict per frame, logged
  PoseFusion.apply()         accepted frames -> the source's shadow track, and the fused pose
                             if (source enabled on the dashboard) && (fusion policy allows now)
  disabled: seed every track from the best Limelight's MegaTag1
  enabled:  gross-heading safety net
Swerve.afterVision()     keep CTRE's own estimate aligned; log all tracks
CommandScheduler.run()   everything else reads the fused pose
```

## Why shadows

A disabled source's measurements are still gated and still logged, but "would this source have been
right?" needs more than its raw frames: a single frame's error says nothing about what a stream of
them would do to a pose estimate, and an enabled source's errors are hidden inside the fused pose it
is correcting. Each shadow is odometry plus *only that source*, so after a match
`Localization/Shadow/*/Pose` is a set of complete, directly comparable "what if we had only trusted
X" tracks. Seeds and explicit resets are applied to every track so they start together.

## EKF alongside WPILib

The WPILib estimator takes each camera frame as one whole robot pose with fixed odometry trust.
`TagEkf` (the approach FRC 971 / 1868 run in AOS) takes every tag from every camera, both Orin
cameras and the three chassis Limelights, into one estimate. It runs on the same odometry, seeds
and resets as the fused pose and is **logged only**: it never moves the robot pose.

- **Per-tag measurements.** Each tag is its own correction: the bearing to the tag's centre and the
  distance to it, from the camera. Orin tags come from PhotonLib's per-target camera-to-tag
  translation; Limelight tags from the raw fiducials' `txnc`/`tync` and distance to camera. Bearing
  and distance are not ambiguous even for a single tag (only the tag's facing is), so single and far
  tags are used at a low weight instead of being thrown away. Noise grows with distance squared,
  angle off the optical axis and robot speed. A tag whose innovation fails a chi-squared test is
  rejected; if most tags keep failing for a second, the covariance is widened so the cameras can
  pull it back (`Relocalizations`).
- **Odometry noise that changes.** Each 250 Hz odometry sample predicts the pose from the module
  deltas and the Pigeon's heading change, with uncertainty that grows with distance driven and more
  when the modules disagree with each other or the gyro (slip), or the robot is on a bump. Heading
  gets very little process noise; the Pigeon drifts about 0.4 deg/min.
- **Timing.** The EKF keeps about 1.5 s of states; a frame is applied at its capture time and
  odometry and later frames are replayed over it, so the result does not depend on arrival order
  and AdvantageKit replay regenerates it.

Logged under `Localization/EKF/`: `Pose`, `StdDevs` (m, m, deg), `ErrorVsFusedMeters`,
`HeadingErrorVsFusedDeg`, `ErrorVsTruthMeters` (sim), tag counts and `RejectionCounts/*`, and per
camera `Cameras/<camera>/BearingResidualsDeg` and `RangeResidualsMeters` for the tags that passed.
A camera whose bearing residuals sit off zero has a mount yaw error. Its noise values are starting
points (`TagEkf` constants), to tune in replay against match logs.

Limelight per-tag data is logged as `Localization/Sources/LL-<camera>/MT1/Fiducials`. Logs from
before it replay with Orin tags only.

Scripted sim drive, 2026-10-17: EKF 0.011 m mean error against truth, fused pose 0.020 m.

## Gates per source type

AprilTag sources (Limelight MT1/MT2, SystemCore cameras, Orin), from 2026 `main` + offseason:

| Gate | Threshold | Why |
|---|---|---|
| No Targets in View | 0 tags | |
| High Ambiguity | any tag > 0.9 | pose-flip risk |
| Out of Field | outside the carpet | |
| Rotation Speed | peak \|yaw rate\| over the last 0.3 s ≥ 1.6 rad/s | a frame arriving after a spin stops was captured during it |
| Target Size | Limelight `ta` ≤ 0.025 % | too far to trust |
| Roll/Pitch | > 5° | camera knocked or bad solve (MT2 uses the same frame's MT1 3-D solve) |
| Height | \|z\| > 0.75 m | the robot cannot leave the floor |
| Stale Estimate | older than 1.0 s | the estimator silently drops these otherwise |


## Fusion policy

- Limelights fuse while enabled in teleop, and in auto while the path asks for pose updates or the
  robot is launching (FM's 2026 policy).
- All chassis Limelights fuse, not only the best one (offseason).
- MegaTag1 until the disabled seed is **confirmed** (≥ 2 tags, heading holding within 3° for about a
  second of frames), MegaTag2 after (offseason; switchable at `Vision/ChassisUseMT2`).
- Heading is never fused while enabled (huge heading std-dev), so heading comes from the gyro. It is set by
  the disabled seed, by the gross-heading net (≥ 10° disagreement, stationary, for 1 s), and by the
  LL-Back correction: in the Limelight window, when the median LL-Back MT1 multi-tag heading error
  over at least 0.25 s and 3 frames is ≥ 2°, the heading is re-seeded. It has no stillness
  requirement and only moves the fused tracks, not the tag EKF.
- Orin and SystemCore cameras fuse whenever enabled and the robot is enabled; they start
  disabled.

## Simulation results

Scripted drive (`FM_SIM_SCRIPT=drive`), 2026-09-22, placeholder camera mounts for the Orin, the
SystemCore cameras and the Quest (since removed: FM is not pursuing the Quest). The sim feeds the robot odometry with 2% wheel-radius error and
0.05°/s gyro drift; "error" is against the sim's true pose. Frame error is the median per accepted
frame; shadow error is the mean over the run.

| Source | Frames | Accepted | Frame error (m) | Shadow error (m) | Top rejections |
|---|---:|---:|---:|---:|---|
| LL-Back/MT1 | 2028 | 398 | 0.002 | 0.010 | Criteria not Met, High Ambiguity, Rotation Speed |
| LL-Back/MT2 | 2028 | 1761 | 0.012 | 0.016 | High Ambiguity, Rotation Speed, Target Size |
| LL-Left/MT1 | 723 | 317 | 0.024 | 0.015 | Criteria not Met, Rotation Speed, Target Size |
| LL-Left/MT2 | 723 | 548 | 0.033 | 0.017 | Rotation Speed, Target Size, High Ambiguity |
| LL-Right/MT1 | 1864 | 1351 | 0.001 | 0.006 | Criteria not Met, High Ambiguity, Rotation Speed |
| LL-Right/MT2 | 1864 | 1568 | 0.010 | 0.018 | High Ambiguity, Rotation Speed, Target Size |
| Quest | 2985 | 2819 | 0.101 | 0.082 | Not Reset, Recent Reset |
| orin-front (now Orin-TopLeft) | 2607 | 2425 | 0.004 | 0.010 | Rotation Speed, Out of Field |
| orin-left | 1127 | 1016 | 0.020 | 0.009 | Rotation Speed |
| orin-right | 2538 | 2431 | 0.001 | 0.008 | Rotation Speed |

Fused pose: 0.014 m mean error. Odometry alone: 0.068 m mean, 0.157 m at the end.

Things this already shows, to check on the real robot:

- MegaTag2's shadows are a little worse than MegaTag1's in sim, because MT2 inherits the gyro drift
  through the heading the robot pushes. Real Pigeon drift is smaller than the sim's.
- PhotonLib's simulated per-tag ambiguity runs high (0.4–0.9 stationary), so "High Ambiguity" is
  overrepresented in sim. Do not tune that gate from sim.

## Adding a source

1. Write a `PoseSourceIO` that fills `PoseSourceInputs` from the device (and, for sim, one that
   generates frames, following the patterns in `SimVision`).
2. Pick or write its gate chain and a `StdDevModel` (in `VisionGates`).
3. Construct a `PoseSource` in `Vision`, add it to `allSources` and to a fusion-policy loop.
4. `PoseSource` logs it, gives it a shadow track and a dashboard switch.
