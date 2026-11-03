# Calibration solver: one drive, two tags, the whole robot

A short, fixed driving routine in front of two known AprilTags (a hub face's pair, both in view
together) and one offline solve over its log. The solve treats every robot pose during the drive,
plus everything we normally hand-enter or guess, as unknowns, and fits them all at once to the tag
corners, odometry, gyro and motor data. Output: each value with its uncertainty, compared against the
config, pass/fail where it matters, and suggested config values.

Why it helps: most pose problems we chase at events (a camera 5° off, a backwards Pigeon, wheels
reading 10% long, a swapped module) are small configuration errors. Each is easy to fix once
measured, and hard to see until it becomes a big problem. TXMAN1 had several at once (see
`pose-improvements.md`).

Two tags rather than one: their known spacing in the same image gives absolute scale and checks the
lens model, which a single tag can't. Check the pair's spacing and heights in the field map before
using it.

The tag map must be right first: the solver assumes the two tags are where the map says, so field
calibration (`FIELD-CALIBRATION-PLAN.md`) comes before robot calibration, or its errors leak into the
camera mounts.

## What one camera and two tags can measure

**The camera**

| Value | How the data shows it |
|---|---|
| Lens: focal length, principal point, distortion | The two tags' known spacing seen at many distances and positions across the image. Catches a calibration made at the wrong resolution |
| Mount: x, y, z, roll, pitch, yaw | Yaw/pitch/roll: steady image offsets. x/y: during a spin a wrong lever arm makes the camera's robot position trace a circle with radius equal to the error. Height: range and vertical angle disagree more with distance |
| Lag (capture to robot time) | Spins, with the gyro as reference: the time shift that makes vision heading match |
| Timestamp jitter, real frame rate, clock sync | Per-frame timing against robot time |
| Measured noise model | Tag error against distance, viewing angle and speed: replaces TagEkf's hand-set bearing and range noise for this camera |
| Usable limits | Distance and viewing angle where tags stay reliable, and where pose ambiguity starts flipping |

**Odometry and gyro**

| Value | How |
|---|---|
| Wheel radius | Straight drives toward and away from the tags: wheel distance vs vision distance |
| Per-module scale | The same, per module (tread wear, weight distribution) |
| Module positions (wheelbase, track) | Spins: wheel travel per gyro radian gives each module's radius; steer angles in a pure spin give the layout's proportions (independent of slip) |
| Steer encoder offsets | Straight drives at speed: see "Steer offsets from logs" below |
| Gyro scale, drift, mount orientation | Spins vs vision heading; drift while still; Pigeon/SystemCore IMU acceleration vs wheels (TXMAN1's Pigeon was turned around) |
| Odometry lag | Timing of wheel and gyro data vs vision |

**Traction and control**

| Value | How |
|---|---|
| Slip current | Acceleration ramps: the current at which vision acceleration falls behind wheel acceleration. That's the right `slipCurrent` |
| Acceleration, deceleration and speed limits without slip | The same ramps, measured against vision. Feed path-planning constraints and traction control |
| Drive feedforward (kS, kV, kA) | Voltage vs measured motion, like SysId but against the true motion |
| Steer response | Time to reach a commanded angle, overshoot |

**Wiring and configuration**

| Problem | How it shows |
|---|---|
| Inverted motor or encoder | Commanded direction vs measured wheel direction vs what vision sees |
| Swapped CAN IDs or modules | Command one module alone: which wheel's data moves, and vision shows which corner of the robot that is |
| Wrong drive or couple ratio | A consistent scale on one module; steering a module in place moving its drive encoder by other than the couple ratio |
| Voltage sag, brownout | During the hard ramps |

## With two or more cameras

- **Relative mounts:** two cameras seeing the tags at once must agree on one robot pose, which pins
  their relative mounts tighter than each alone.
- **One robot centre:** the spin centre seen by every camera; checks every mount shares one origin.
- **Swapped camera names:** a left/right swap in config can't be made to agree.
- **Relative lag:** each camera's lag against the others.
- **Coverage map:** which camera sees the tags at which heading and distance; finds gaps in 360°.
- **Per-camera quality ranking:** noise against distance and angle per camera (dirty lens, focus,
  bad calibration).
- **Tag map check:** whether the two tags' measured spacing and heights match the map.

## The routine (about 2-3 minutes)

| Segment | Main things it measures |
|---|---|
| Still in front of the tags, 10 s | Gyro drift, IMU mount (gravity), camera noise at rest |
| Slow spins in place, both directions | Module positions, camera x/y, camera lag, gyro scale |
| Straight toward and away from the tags, several speeds | Wheel radius, per-module scale, height and pitch, range noise, steer offsets |
| Strafe side to side in front of the tags | Camera yaw, bearing noise, steer offsets |
| Acceleration and braking ramps | Slip current, acceleration limits, feedforward |
| Figure-8 near the tags | Everything at once, coverage across headings |
| Each module commanded alone, briefly | Inversions, swapped CAN IDs, ratios |

## Steer offsets from logs

Each module's steer offset can be checked from any log, routine or match:

- **Relative offsets (module vs module):** when the robot drives straight at speed, all four
  modules' velocity vectors must agree with one rigid-body motion. A module whose offset is off by δ
  pushes sideways at δ × its speed; the twist fit's residual at that module, perpendicular to its
  travel, divided by its speed, measures δ. Wheels alone, no vision needed.
- **The common offset (all four together):** if all four are off by the same amount the robot still
  fits a rigid motion, but it drives at an angle to its heading. Vision or the gyro sees the
  wheels' travel direction differ from the true direction of travel, giving the shared part.
- **Slipped offsets:** the same estimate over time per log: a step change between matches (or
  within one, after a hit) means the encoder or magnet slipped.

## Implementation notes

- One nonlinear least-squares solve over the drive (bundle adjustment / hand-eye calibration):
  unknowns are the robot pose at each frame plus the per-robot values above; constraints are tag
  corners, odometry between frames, gyro, and motor data.
- Report each value with its uncertainty; flag values the routine didn't excite enough to measure.
- Reuses `TagEkfTuner`'s log export, `CornerSighting`'s camera model and `MountEstimator`'s
  conventions (camera-local corrections composed onto the mount).
- Prerequisites in logs: drive and steer current and applied voltage as per-loop inputs
  (`pose-improvements.md`, logging requirements).
